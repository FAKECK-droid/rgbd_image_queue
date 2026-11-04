#!/usr/bin/env python3
"""最新 RGB-D person 持续跟随；推理后台执行，失败后自动重新发起导航。"""
import math
import os
from pathlib import Path
import struct
import time
from concurrent.futures import ThreadPoolExecutor

for variable in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(variable, "1")

import cv2
import numpy as np
import rclpy
import torch
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from scout_msgs.msg import ScoutStatus
from sensor_msgs.msg import CameraInfo, PointCloud2
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from ultralytics import YOLO

from rgbd_image_queue.srv import GetImage


NAV2_NODES = (
    "map_server", "controller_server", "smoother_server", "planner_server",
    "behavior_server", "bt_navigator", "waypoint_follower", "velocity_smoother",
)
COSTMAP_SERVICES = (
    "/local_costmap/clear_entirely_local_costmap",
    "/global_costmap/clear_entirely_global_costmap",
)
HARDWARE_TOPICS = (
    ("base", ScoutStatus, "/scout_status"),
    ("cloud", PointCloud2, "/cloud_registered_body"),
    ("lio", Odometry, "/lio/odometry"),
    ("ekf", Odometry, "/odometry/filtered"),
)


def box_iou(a, b):
    width = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    height = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    intersection = width * height
    union = ((a[2] - a[0]) * (a[3] - a[1])
             + (b[2] - b[0]) * (b[3] - b[1]) - intersection)
    return intersection / union if union > 0.0 else 0.0


def select_person(boxes, confidences, previous_box):
    if previous_box is None:
        return int(np.argmax(confidences))
    return max(range(len(boxes)),
               key=lambda i: (box_iou(previous_box, boxes[i]), float(confidences[i])))


def center_point(box, depth, info):
    """只读取中心深度像素，避免转换/复制整张深度图；K/D 对应对齐后的彩色图。"""
    u, v = int((box[0] + box[2]) / 2), int((box[1] + box[3]) / 2)
    code, size, scale = {"16UC1": ("H", 2, 0.001), "32FC1": ("f", 4, 1.0)}[depth.encoding]
    order = ">" if depth.is_bigendian else "<"
    z = struct.unpack_from(order + code, depth.data, v * depth.step + u * size)[0] * scale
    if not np.isfinite(z) or z <= 0.0:
        return (u, v), None
    ray = cv2.undistortPoints(
        np.array([[[u, v]]], dtype=np.float64), np.asarray(info.k).reshape(3, 3),
        np.asarray(info.d) if info.d else None,
    )[0, 0]
    return (u, v), (float(ray[0] * z), float(ray[1] * z), float(z))


class PersonGoal(Node):
    def __init__(self):
        super().__init__("person_goal")
        workspace = Path.home() / "ros2_ws"
        for name, value in (
            ("model_path", str(workspace / "src/scout_mini_vln/rgbd_image_queue/test/yolo26n.pt")),
            ("device", "0"), ("confidence", 0.4), ("imgsz", 416), ("half", True),
            ("cpu_threads", 1), ("rate_hz", 3.0), ("visualize", False),
            ("retry_delay_sec", 0.3), ("clear_costmaps", True), ("follow_distance", 0.5),
            ("status_period_sec", 5.0),
            ("behavior_tree", str(Path(get_package_share_directory("rgbd_image_queue"))
                                  / "config/person_follow.xml")),
            ("service_name", "/rgbd_image_queue/get_image"),
            ("camera_info_topic", "/camera/color/camera_info"),
        ):
            self.declare_parameter(name, value)
        status_period = self.get_parameter("status_period_sec").value
        if not math.isfinite(status_period) or status_period <= 0.0:
            raise ValueError("status_period_sec must be finite and positive")
        cv2.setNumThreads(1)
        torch.set_num_threads(self.get_parameter("cpu_threads").value)
        self.visualize = self.get_parameter("visualize").value
        self.device = self.get_parameter("device").value
        self.confidence = self.get_parameter("confidence").value
        self.imgsz = self.get_parameter("imgsz").value
        self.follow_distance = self.get_parameter("follow_distance").value
        self.quantize = 16 if self.get_parameter("half").value and self.device != "cpu" else None
        self.get_logger().info(
            f"加载 YOLO 模型：{self.get_parameter('model_path').value}，device={self.device}")
        self.model = YOLO(self.get_parameter("model_path").value)
        self.get_logger().info("YOLO 模型已加载；等待图像开始推理。")
        self.bridge = CvBridge()
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="person_yolo")
        self.work_future = None
        self.image_future = None
        self.last_pair_id = None
        self.previous_box = None
        self.info = None
        self.display_frame = None
        self.display_dirty = False
        self.pending_goal = None
        self.latest_goal = None
        self.goal_pending = False
        self.goal_handle = None
        self.follow_enabled = True
        self.retry_after = 0.0
        self.clear_after = 0.0
        self.clear_futures = []
        self.active = dict.fromkeys(NAV2_NODES, False)
        self.nav2_states = dict.fromkeys(NAV2_NODES, "尚未查询")
        self.image_status = "尚未收到图像对"
        self.detection_status = "尚未完成推理"
        self.last_detection_at = None
        self.processed_frames = 0
        self.goal_publish_count = 0
        self.last_goal_at = None
        self.image_requested_at = self.inference_started_at = 0.0
        self.clear_started_at = self.action_requested_at = 0.0
        self.nav_status = "尚未发起 NavigateToPose"
        self.camera_frame = ""
        self.state_futures = {}
        self.state_clients = {
            name: self.create_client(GetState, f"/{name}/get_state") for name in NAV2_NODES
        }
        self.navigator = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.clear_clients = [
            self.create_client(ClearEntireCostmap, name) for name in COSTMAP_SERVICES
        ]
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.client = self.create_client(GetImage, self.get_parameter("service_name").value)
        self.publisher = self.create_publisher(PoseStamped, "/goal_update", 10)
        self.person_publisher = self.create_publisher(PoseStamped, "/person_position", 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.info_subscription = self.create_subscription(
            CameraInfo, self.get_parameter("camera_info_topic").value, self.on_camera_info, qos)
        # 仅启动时各收一条作为就绪信号，随后退订，尤其不持续反序列化点云。
        self.hardware_subscriptions = {
            key: self.create_subscription(
                msg_type, topic, lambda msg, key=key: self.on_hardware_ready(key), qos)
            for key, msg_type, topic in HARDWARE_TOPICS
        }
        self.state_timer = self.create_timer(1.0, self.poll_nav2)
        self.image_timer = self.create_timer(
            1.0 / self.get_parameter("rate_hz").value, self.request_images)
        self.control_timer = self.create_timer(0.05, self.control_tick)
        self.status_timer = self.create_timer(status_period, self.log_status)
        self.get_logger().info(
            f"等待外部硬件/Nav2 就绪；状态每 {status_period:g}s 汇总；"
            f"推理上限 {self.get_parameter('rate_hz').value:g}Hz。")
        self.log_status()

    def log_status(self):
        """只读取现有状态；不阻塞等待、不改变启动门限。"""
        missing = []
        if self.info is None:
            missing.append(f"CameraInfo 未收到：{self.get_parameter('camera_info_topic').value}")
        for key, _, topic in HARDWARE_TOPICS:
            if key in self.hardware_subscriptions:
                missing.append(f"硬件 {key} 未收到首条消息：{topic}")
        for name, client in self.state_clients.items():
            if self.active[name]:
                continue
            if not client.service_is_ready():
                state = f"/{name}/get_state 服务不可用"
            elif name in self.state_futures:
                state = f"等待 get_state 响应，上次状态={self.nav2_states[name]}"
            else:
                state = self.nav2_states[name]
            missing.append(f"Nav2 {name} 未 active：{state}")
        if not self.client.service_is_ready():
            missing.append(f"取图服务不可用：{self.get_parameter('service_name').value}")
        if not self.navigator.server_is_ready():
            missing.append("导航 action 不可用：/navigate_to_pose")
        if self.get_parameter("clear_costmaps").value:
            for name, client in zip(COSTMAP_SERVICES, self.clear_clients):
                if not client.service_is_ready():
                    missing.append(f"清理 costmap 服务不可用：{name}")
        if not self.camera_frame:
            missing.append("尚未取得相机 frame（CameraInfo/深度图 header.frame_id）")
        for source in dict.fromkeys((self.camera_frame, "base_link")):
            if not source:
                continue
            try:
                self.tf_buffer.lookup_transform("map", source, Time())
            except TransformException as error:
                missing.append(f"TF map <- {source} 未就绪：{error}")
        if missing:
            self.get_logger().warning("[未就绪] " + "；".join(missing))
        else:
            self.get_logger().info(
                "[全部就绪] 硬件首帧、CameraInfo、取图服务、Nav2 active、导航 action 和 TF 均已确认。")

        now = time.monotonic()
        image = self.image_status
        if self.image_future is not None:
            image += f"；等待取图响应 {now - self.image_requested_at:.1f}s"
        if self.work_future is not None:
            image += f"；推理处理中 {now - self.inference_started_at:.1f}s"
        age = ("尚无检测结果" if self.last_detection_at is None
               else f"距上次结果 {now - self.last_detection_at:.1f}s")
        self.get_logger().info(
            f"[检测] {self.detection_status}；{age}；累计处理 {self.processed_frames} 帧；[图像] {image}")

        if not self.follow_enabled:
            navigation = "已取消，自动跟随暂停（重启跟随节点可恢复）"
        elif self.goal_handle is not None:
            navigation = "NavigateToPose 已接受、执行中"
        elif self.goal_pending:
            navigation = f"等待 NavigateToPose 接受，已等待 {now - self.action_requested_at:.1f}s"
        elif now < self.retry_after:
            navigation = f"等待自动重试，剩余 {self.retry_after - now:.1f}s；{self.nav_status}"
        elif self.latest_goal is None:
            navigation = f"尚未启动：尚无已发布的有效目标；{self.nav_status}"
        else:
            navigation = f"等待目标更新/启动条件；{self.nav_status}"
        if self.clear_futures:
            done = sum(future.done() for future in self.clear_futures)
            update = f"清理 costmap {done}/2 完成，已等待 {now - self.clear_started_at:.1f}s"
        elif self.pending_goal is not None:
            update = "已有待发布目标，等待依赖就绪"
            if self.get_parameter("clear_costmaps").value:
                update += "并清理 costmap"
            if now < self.clear_after:
                update += f"（重试等待 {self.clear_after - now:.1f}s）"
        else:
            update = "没有待发布的新目标"
        goal = f"/goal_update 累计发布 {self.goal_publish_count} 次"
        if self.latest_goal is not None:
            position = self.latest_goal.pose.position
            goal += (f"，最近目标 map=({position.x:.2f}, {position.y:.2f})"
                     f"，距上次发布 {now - self.last_goal_at:.1f}s")
        self.get_logger().info(f"[导航] {navigation}；{update}；{goal}")

    def on_camera_info(self, msg):
        self.info = msg
        self.camera_frame = msg.header.frame_id
        self.destroy_subscription(self.info_subscription)
        self.get_logger().info(
            f"[相机就绪] CameraInfo {msg.width}x{msg.height}，frame={msg.header.frame_id}")

    def on_hardware_ready(self, key):
        subscription = self.hardware_subscriptions.pop(key, None)
        if subscription is not None:
            self.destroy_subscription(subscription)
            topic = next(topic for name, _, topic in HARDWARE_TOPICS if name == key)
            self.get_logger().info(f"[硬件首帧就绪] {key}：{topic}")

    def poll_nav2(self):
        if all(self.active.values()):
            return
        for name, client in self.state_clients.items():
            if client.service_is_ready() and name not in self.state_futures:
                future = client.call_async(GetState.Request())
                self.state_futures[name] = future
                future.add_done_callback(lambda done, name=name: self.on_nav2_state(name, done))

    def on_nav2_state(self, name, future):
        self.state_futures.pop(name, None)
        try:
            state = future.result().current_state
        except Exception as error:
            self.active[name] = False
            self.nav2_states[name] = f"状态查询失败：{error}"
            self.get_logger().warning(
                f"Nav2 {name} 状态查询失败：{error}", throttle_duration_sec=5.0)
            return
        description = f"{state.label}(id={state.id})"
        if description != self.nav2_states[name]:
            self.get_logger().info(f"[Nav2 状态] {name}：{description}")
        self.nav2_states[name] = description
        self.active[name] = state.id == State.PRIMARY_STATE_ACTIVE

    def request_images(self):
        # 一次只取/处理一对；不在 ROS 回调中运行推理，也不堆积历史任务。
        if (self.image_future is not None or self.work_future is not None
                or self.info is None or not self.client.service_is_ready()):
            return
        request = GetImage.Request()
        request.operation = GetImage.Request.LATEST_PAIR
        self.image_requested_at = time.monotonic()
        self.image_future = self.client.call_async(request)
        self.image_future.add_done_callback(self.on_images)

    def on_images(self, future):
        self.image_future = None
        try:
            response = future.result()
        except Exception as error:
            self.image_status = f"取图请求失败：{error}"
            self.get_logger().warning(self.image_status, throttle_duration_sec=5.0)
            return
        if not response.success:
            self.image_status = f"取图服务返回失败：{response.message}"
            self.get_logger().warning(self.image_status, throttle_duration_sec=5.0)
            return
        self.camera_frame = response.depth.header.frame_id
        if response.pair_id == self.last_pair_id:
            self.image_status = f"图像对 pair_id={response.pair_id} 未更新，等待新图"
            return
        self.last_pair_id = response.pair_id
        self.image_status = f"已取得最新图像对 pair_id={response.pair_id}"
        self.inference_started_at = time.monotonic()
        self.work_future = self.worker.submit(self.detect, response, self.info, self.previous_box)

    def detect(self, response, info, previous_box):
        color = self.bridge.imgmsg_to_cv2(response.color, desired_encoding="bgr8")
        result = self.model.predict(
            color, classes=[0], conf=self.confidence, device=self.device,
            imgsz=self.imgsz, quantize=self.quantize, max_det=20, verbose=False)[0]
        # 一次 GPU -> CPU 传输拿到框和置信度。
        data = result.boxes.data.cpu().numpy()
        if len(data) == 0:
            return color, None, None, None, response.depth.header.frame_id, 0, 0.0
        index = select_person(data[:, :4], data[:, 4], previous_box)
        box = data[index, :4].copy()
        pixel, xyz = center_point(box, response.depth, info)
        return color, box, pixel, xyz, response.depth.header.frame_id, len(data), float(data[index, 4])

    def on_detection(self, detection):
        color, box, pixel, xyz, frame, count, confidence = detection
        self.last_detection_at = time.monotonic()
        self.processed_frames += 1
        if box is None:
            self.detection_status = "未检测到 person（数量=0）"
            self.show(color, None, None, "No person")
            return
        self.previous_box = box
        self.detection_status = (
            f"检测到 person {count} 人；所选框置信度 {confidence:.2f}，中心像素={pixel}")
        if xyz is None:
            self.detection_status += "；中心深度无效，本帧不更新目标"
            self.show(color, box, pixel, "Invalid center depth")
            return
        self.detection_status += f"，深度 {xyz[2]:.2f}m"
        point = PointStamped()
        point.header.frame_id = frame
        point.point.x, point.point.y, point.point.z = xyz
        try:
            # 在线跟随直接使用最新可用 TF，不等待历史图像时刻，也不做图像年龄检查。
            transform = self.tf_buffer.lookup_transform("map", frame, Time())
            robot = self.tf_buffer.lookup_transform("map", "base_link", Time())
        except TransformException as error:
            self.detection_status += "；TF 未就绪，本帧不更新目标"
            self.get_logger().warning(
                f"[TF 未就绪] {error}", throttle_duration_sec=5.0)
            self.show(color, box, pixel, "Waiting for TF")
            return
        mapped = do_transform_point(point, transform)
        person = PoseStamped()
        person.header.frame_id = "map"
        person.header.stamp = self.get_clock().now().to_msg()
        person.pose.position = mapped.point
        person.pose.orientation.w = 1.0
        self.person_publisher.publish(person)

        # 跟随间距只计算一次：先生成可供规划的停车点，行为树不再截短路径。
        # 已在跟随距离内时保持当前位置，不要求车辆后退。
        rx, ry = robot.transform.translation.x, robot.transform.translation.y
        dx, dy = mapped.point.x - rx, mapped.point.y - ry
        distance = math.hypot(dx, dy)
        scale = max(0.0, (distance - self.follow_distance) / distance) if distance > 0.0 else 0.0
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x, goal.pose.position.y = rx + dx * scale, ry + dy * scale
        yaw = math.atan2(dy, dx)
        goal.pose.orientation.z, goal.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        self.pending_goal = goal
        self.detection_status += (
            f"；person map=({mapped.point.x:.2f}, {mapped.point.y:.2f})，"
            f"距离 {distance:.2f}m，目标间距 {self.follow_distance:.2f}m；已生成目标")
        self.show(color, box, pixel,
                  f"person {distance:.2f}m  follow {self.follow_distance:.2f}m  depth {xyz[2]:.2f}m")

    def publish_goal(self):
        goal = self.pending_goal
        self.pending_goal = None
        goal.header.stamp = self.get_clock().now().to_msg()
        self.latest_goal = goal
        self.publisher.publish(goal)
        self.goal_publish_count += 1
        self.last_goal_at = time.monotonic()

    def control_tick(self):
        if self.work_future is not None and self.work_future.done():
            future, self.work_future = self.work_future, None
            try:
                self.on_detection(future.result())
            except Exception as error:
                self.last_detection_at = time.monotonic()
                self.detection_status = f"推理/检测结果处理失败：{type(error).__name__}: {error}"
                self.get_logger().error(self.detection_status, throttle_duration_sec=5.0)
        if not self.follow_enabled:
            return
        now = time.monotonic()
        if self.clear_futures and all(future.done() for future in self.clear_futures):
            futures, self.clear_futures = self.clear_futures, []
            try:
                for future in futures:
                    future.result()
            except Exception as error:
                self.get_logger().warning(f"清理 costmap 失败：{error}")
                self.clear_after = now + self.get_parameter("retry_delay_sec").value
            else:
                # 清理期间的新检测覆盖 pending_goal，只发布最新目标。
                self.publish_goal()
        if (self.pending_goal is not None and not self.clear_futures
                and now >= self.clear_after and all(self.active.values())
                and not self.hardware_subscriptions):
            if not self.get_parameter("clear_costmaps").value:
                self.publish_goal()
            elif all(client.service_is_ready() for client in self.clear_clients):
                self.clear_started_at = now
                self.clear_futures = [
                    client.call_async(ClearEntireCostmap.Request()) for client in self.clear_clients
                ]
        if (self.latest_goal is None or self.goal_pending or now < self.retry_after
                or self.pending_goal is not None or self.clear_futures
                or not all(self.active.values()) or self.hardware_subscriptions
                or not self.navigator.server_is_ready()):
            return
        request = NavigateToPose.Goal()
        request.pose = self.latest_goal
        request.behavior_tree = self.get_parameter("behavior_tree").value
        self.goal_pending = True
        self.action_requested_at = now
        self.nav_status = "NavigateToPose 请求已发送"
        self.navigator.send_goal_async(request).add_done_callback(self.on_goal_response)
        self.get_logger().info(
            "启动条件已满足；发起 NavigateToPose 持续跟随，"
            f"目标 map=({request.pose.pose.position.x:.2f}, {request.pose.pose.position.y:.2f})。")

    def retry_navigation(self, reason):
        self.goal_pending = False
        self.goal_handle = None
        self.nav_status = reason
        self.retry_after = time.monotonic() + self.get_parameter("retry_delay_sec").value
        if self.pending_goal is None:
            self.pending_goal = self.latest_goal
        self.active = dict.fromkeys(NAV2_NODES, False)
        self.poll_nav2()
        cleanup = "清理两张 costmap 后" if self.get_parameter("clear_costmaps").value else ""
        self.get_logger().warning(
            f"{reason}；等待至少 {self.get_parameter('retry_delay_sec').value:g}s，{cleanup}自动重试。")

    def on_goal_response(self, future):
        try:
            self.goal_handle = future.result()
        except Exception as error:
            self.retry_navigation(f"导航请求失败：{error}")
            return
        if not self.goal_handle.accepted:
            self.retry_navigation("NavigateToPose 被拒绝")
            return
        self.nav_status = "NavigateToPose 已接受"
        self.get_logger().info("NavigateToPose 已接受，开始持续跟随。")
        self.goal_handle.get_result_async().add_done_callback(self.on_goal_result)

    def on_goal_result(self, future):
        try:
            status = future.result().status
        except Exception as error:
            self.retry_navigation(f"读取导航结果失败：{error}")
            return
        if status == GoalStatus.STATUS_CANCELED:
            self.goal_pending = False
            self.goal_handle = None
            self.follow_enabled = False
            self.nav_status = "NavigateToPose 已取消，自动跟随暂停"
            self.get_logger().info("导航已取消，停止自动重发；重启节点可恢复跟随。")
            return
        label = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
        }.get(status, "UNKNOWN")
        self.retry_navigation(f"导航结束，状态 {label}({status})")

    def show(self, image, box, pixel, text):
        if not self.visualize:
            return
        if box is not None:
            x1, y1, x2, y2 = (int(value) for value in box)
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(image, pixel, 4, (0, 0, 255), -1)
        cv2.putText(image, text, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
        self.display_frame, self.display_dirty = image, True


def main():
    rclpy.init()
    node = PersonGoal()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.02)
            if node.visualize and node.display_dirty:
                cv2.imshow("person_goal", node.display_frame)
                cv2.waitKey(1)
                node.display_dirty = False
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.worker.shutdown(wait=True, cancel_futures=True)
        executor.shutdown()
        node.destroy_node()
        if node.visualize:
            cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

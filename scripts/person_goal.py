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
    """在实际使用时解开 compressedDepth PNG，只对中心像素计算米制深度。"""
    u, v = int((box[0] + box[2]) / 2), int((box[1] + box[3]) / 2)
    # ROS compressedDepth 的 ConfigHeader 为 int32 + 两个 float32，之后为 PNG。
    pixels = cv2.imdecode(np.frombuffer(depth.data, dtype=np.uint8, offset=12),
                          cv2.IMREAD_UNCHANGED)
    value = int(pixels[v, u])
    if depth.format.split(";")[0] == "16UC1":
        z = value * 0.001
    else:  # 32FC1 使用头内量化参数恢复逆深度，0 对应无效深度。
        _, a, b = struct.unpack_from("<iff", depth.data)
        z = a / (value - b) if value else float("nan")
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
            ("behavior_tree", str(Path(get_package_share_directory("rgbd_image_queue"))
                                  / "config/person_follow.xml")),
            ("service_name", "/rgbd_image_queue/get_image"),
            ("camera_info_topic", "/camera/color/camera_info"),
        ):
            self.declare_parameter(name, value)
        cv2.setNumThreads(1)
        torch.set_num_threads(self.get_parameter("cpu_threads").value)
        self.visualize = self.get_parameter("visualize").value
        self.device = self.get_parameter("device").value
        self.confidence = self.get_parameter("confidence").value
        self.imgsz = self.get_parameter("imgsz").value
        self.follow_distance = self.get_parameter("follow_distance").value
        self.quantize = 16 if self.get_parameter("half").value and self.device != "cpu" else None
        self.model = YOLO(self.get_parameter("model_path").value)
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
        self.state_futures = {}
        self.state_clients = {
            name: self.create_client(GetState, f"/{name}/get_state") for name in NAV2_NODES
        }
        self.navigator = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.clear_clients = [
            self.create_client(ClearEntireCostmap, name) for name in (
                "/local_costmap/clear_entirely_local_costmap",
                "/global_costmap/clear_entirely_global_costmap",
            )
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
        self.get_logger().info("等待外部硬件/Nav2 就绪；后台推理，持续跟随，失败自动重试。")

    def on_camera_info(self, msg):
        self.info = msg
        self.destroy_subscription(self.info_subscription)

    def on_hardware_ready(self, key):
        subscription = self.hardware_subscriptions.pop(key)
        self.destroy_subscription(subscription)

    def poll_nav2(self):
        if all(self.active.values()):
            return
        for name, client in self.state_clients.items():
            if client.service_is_ready() and name not in self.state_futures:
                future = client.call_async(GetState.Request())
                self.state_futures[name] = future
                future.add_done_callback(lambda done, name=name: self.on_nav2_state(name, done))

    def on_nav2_state(self, name, future):
        self.active[name] = future.result().current_state.id == State.PRIMARY_STATE_ACTIVE
        self.state_futures.pop(name, None)

    def request_images(self):
        # 一次只取/处理一对；不在 ROS 回调中运行推理，也不堆积历史任务。
        if (self.image_future is not None or self.work_future is not None
                or self.info is None or not self.client.service_is_ready()):
            return
        request = GetImage.Request()
        request.operation = GetImage.Request.LATEST_PAIR
        self.image_future = self.client.call_async(request)
        self.image_future.add_done_callback(self.on_images)

    def on_images(self, future):
        self.image_future = None
        response = future.result()
        if not response.success or response.pair_id == self.last_pair_id:
            return
        self.last_pair_id = response.pair_id
        self.work_future = self.worker.submit(self.detect, response, self.info, self.previous_box)

    def detect(self, response, info, previous_box):
        color = cv2.imdecode(np.frombuffer(response.color.data, dtype=np.uint8),
                             cv2.IMREAD_COLOR)
        result = self.model.predict(
            color, classes=[0], conf=self.confidence, device=self.device,
            imgsz=self.imgsz, quantize=self.quantize, max_det=20, verbose=False)[0]
        # 一次 GPU -> CPU 传输拿到框和置信度。
        data = result.boxes.data.cpu().numpy()
        if len(data) == 0:
            return color, None, None, None, response.depth.header.frame_id
        index = select_person(data[:, :4], data[:, 4], previous_box)
        box = data[index, :4].copy()
        pixel, xyz = center_point(box, response.depth, info)
        return color, box, pixel, xyz, response.depth.header.frame_id

    def on_detection(self, detection):
        color, box, pixel, xyz, frame = detection
        if box is None:
            self.show(color, None, None, "No person")
            return
        self.previous_box = box
        if xyz is None:
            self.show(color, box, pixel, "Invalid center depth")
            return
        point = PointStamped()
        point.header.frame_id = frame
        point.point.x, point.point.y, point.point.z = xyz
        try:
            # 在线跟随直接使用最新可用 TF，不等待历史图像时刻，也不做图像年龄检查。
            transform = self.tf_buffer.lookup_transform("map", frame, Time())
            robot = self.tf_buffer.lookup_transform("map", "base_link", Time())
        except TransformException as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5.0)
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
        self.show(color, box, pixel,
                  f"person {distance:.2f}m  follow {self.follow_distance:.2f}m  depth {xyz[2]:.2f}m")

    def publish_goal(self):
        goal = self.pending_goal
        self.pending_goal = None
        goal.header.stamp = self.get_clock().now().to_msg()
        self.latest_goal = goal
        self.publisher.publish(goal)

    def control_tick(self):
        if self.work_future is not None and self.work_future.done():
            detection = self.work_future.result()
            self.work_future = None
            self.on_detection(detection)
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
        self.navigator.send_goal_async(request).add_done_callback(self.on_goal_response)
        self.get_logger().info("发起 NavigateToPose 持续跟随。")

    def retry_navigation(self, reason):
        self.goal_pending = False
        self.goal_handle = None
        self.retry_after = time.monotonic() + self.get_parameter("retry_delay_sec").value
        if self.pending_goal is None:
            self.pending_goal = self.latest_goal
        self.active = dict.fromkeys(NAV2_NODES, False)
        self.poll_nav2()
        self.get_logger().warning(f"{reason}；清理两张 costmap 后自动重试。")

    def on_goal_response(self, future):
        try:
            self.goal_handle = future.result()
        except Exception as error:
            self.retry_navigation(f"导航请求失败：{error}")
            return
        if not self.goal_handle.accepted:
            self.retry_navigation("NavigateToPose 被拒绝")
            return
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
            self.get_logger().info("导航已取消，停止自动重发；重启节点可恢复跟随。")
            return
        self.retry_navigation(f"导航结束，状态码 {status}")

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

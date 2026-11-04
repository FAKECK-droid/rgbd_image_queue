"""仅启动 RGB-D 队列和 person 跟随；底盘、传感器、定位与 Nav2 由用户单独启动。"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("rgbd_image_queue")
    workspace = Path.home() / "ros2_ws"
    parameters = {
        "rate_hz": ("3.0", float, "取图/推理频率上限，Hz"),
        "follow_distance": ("0.5", float, "base_link 到 person 位置的跟随间距，米"),
        "imgsz": ("416", int, "YOLO 推理输入尺寸"),
        "half": ("true", bool, "CUDA FP16 推理"),
        "cpu_threads": ("1", int, "PyTorch CPU 线程数"),
        "visualize": ("false", bool, "显示目标框、深度和 map 坐标"),
        "device": ("0", str, "CUDA 设备号或 cpu"),
        "confidence": ("0.4", float, "person 置信度门限"),
        "retry_delay_sec": ("0.3", float, "Action 结束/失败后重发的最小间隔"),
        "clear_costmaps": ("true", bool, "每次有效目标更新前异步清理两张 costmap"),
        "behavior_tree": (
            os.path.join(share, "config", "person_follow.xml"),
            str, "本次 NavigateToPose 使用的跟随树"),
        "service_name": ("/rgbd_image_queue/get_image", str, "取图服务名"),
        "use_sim_time": ("false", bool, "使用 ROS 仿真时钟"),
        "model_path": (
            str(workspace / "src/scout_mini_vln/rgbd_image_queue/test/yolo26n.pt"),
            str, "COCO YOLO 模型路径"),
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=description)
          for name, (default, _, description) in parameters.items()],
        DeclareLaunchArgument("python_executable", default_value=str(
            workspace / ".venvs/yolo-jetson/bin/python")),
        DeclareLaunchArgument("queue_config", default_value=os.path.join(
            share, "config", "object_follow_queue.yaml")),
        Node(
            package="rgbd_image_queue", executable="rgbd_image_queue_node",
            name="rgbd_image_queue", output="screen",
            parameters=[LaunchConfiguration("queue_config"), {
                "service_name": ParameterValue(LaunchConfiguration("service_name"), value_type=str),
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
            }]),
        Node(
            package="rgbd_image_queue", executable="person_goal.py", name="person_goal",
            prefix=["nice -n 5 ", LaunchConfiguration("python_executable")], output="screen",
            additional_env={
                "OMP_NUM_THREADS": "1", "OPENBLAS_NUM_THREADS": "1",
                "MKL_NUM_THREADS": "1", "NUMEXPR_NUM_THREADS": "1",
            },
            parameters=[{
                name: ParameterValue(LaunchConfiguration(name), value_type=value_type)
                for name, (_, value_type, _) in parameters.items()
            }]),
    ])

"""启动队列管理器；相机继续由已有导航/相机 launch 提供。"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("rgbd_image_queue")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=os.path.join(share, "config/queue.yaml")),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false", choices=["true", "false"]),
        Node(package="rgbd_image_queue", executable="rgbd_image_queue_node",
             name="rgbd_image_queue", namespace=LaunchConfiguration("namespace"), output="screen",
             parameters=[LaunchConfiguration("config"), {
                 "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
             }]),
    ])

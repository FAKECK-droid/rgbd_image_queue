#!/usr/bin/env python3
"""隔离 ROS 域的合成图像测试；不启动相机/雷达/底盘。"""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import time
import unittest

parser = argparse.ArgumentParser()
parser.add_argument("--binary", required=True)
options, remaining = parser.parse_known_args()
os.environ.setdefault("ROS_DOMAIN_ID", "229")
os.environ["ROS_LOCALHOST_ONLY"] = "1"

import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParametersAtomically, GetParameters
from sensor_msgs.msg import Image
from rgbd_image_queue.srv import GetImage


class QueueRosTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("queue_integration_test")
        cls.color = cls.node.create_publisher(Image, "/queue_test/color", 4)
        cls.depth = cls.node.create_publisher(Image, "/queue_test/depth", 4)
        cls.client = cls.node.create_client(GetImage, "/queue_under_test/get_image")
        cls.params = cls.node.create_client(SetParametersAtomically,
                                            "/queue_under_test/set_parameters_atomically")
        cls.getter = cls.node.create_client(GetParameters, "/queue_under_test/get_parameters")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        self.log = Path.cwd() / ("queue-" + self._testMethodName + ".log")
        self.stream = self.log.open("w")
        self.process = subprocess.Popen([
            options.binary, "--ros-args", "-r", "__node:=queue_under_test",
            "-p", "color_topic:=/queue_test/color", "-p", "depth_topic:=/queue_test/depth",
            "-p", "color_transport:=raw", "-p", "depth_transport:=raw",
            "-p", "queue_length:=3", "-p", "save_every_n_frames:=3",
            "-p", "max_image_age_sec:=10.0", "-p", "sync_tolerance_sec:=0.02",
            "-p", "purge_period_sec:=0.02",
        ], stdout=self.stream, stderr=subprocess.STDOUT)
        self.addCleanup(self.stop_process)
        self.assertTrue(self.client.wait_for_service(timeout_sec=5))
        self.assertTrue(self.params.wait_for_service(timeout_sec=5))
        deadline = time.monotonic() + 4
        while self.color.get_subscription_count() == 0 or self.depth.get_subscription_count() == 0:
            self.assertLess(time.monotonic(), deadline)
            rclpy.spin_once(self.node, timeout_sec=.02)

    def stop_process(self):
        self.process.send_signal(signal.SIGINT)
        self.process.wait(timeout=5)
        self.stream.close()
        self.assertEqual(self.process.returncode, 0, self.log.read_text())

    def call(self, operation):
        request = GetImage.Request()
        request.operation = operation
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=4)
        self.assertTrue(future.done(), "service timeout")
        return future.result()

    def configure(self, **values):
        request = SetParametersAtomically.Request()
        request.parameters = [Parameter(k, value=v).to_parameter_msg() for k, v in values.items()]
        future = self.params.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=3)
        self.assertTrue(future.done())
        return future.result().result

    def publish_pairs(self, count=9):
        for index in range(1, count + 1):
            now = self.node.get_clock().now().nanoseconds
            for color in (True, False):
                stamp = now + (0 if color else 3000000)
                image = Image()
                image.header.stamp.sec, image.header.stamp.nanosec = divmod(stamp, 10**9)
                image.header.frame_id = "camera_color_optical_frame"
                image.width = image.height = 2
                image.encoding = "rgb8" if color else "16UC1"
                image.step = 6 if color else 4
                image.data = bytes([index] * (image.step * image.height))
                (self.color if color else self.depth).publish(image)
            deadline = time.monotonic() + .065
            while time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=.005)
        # ApproximateTime 可能保留最后一对，等待下一帧证明其最优匹配。
        deadline = time.monotonic() + 2
        while self.call(GetImage.Request.STATUS).paired_count < count - 1:
            self.assertLess(time.monotonic(), deadline,
                            "paired_count=" + str(self.call(GetImage.Request.STATUS).paired_count))
            rclpy.spin_once(self.node, timeout_sec=.02)

    def test_pair_service_payloads_sampling_and_latest(self):
        self.assertFalse(self.call(GetImage.Request.QUEUE_PAIR).success)
        self.publish_pairs(12)
        latest = self.call(GetImage.Request.LATEST_PAIR)
        self.assertTrue(latest.success)
        self.assertTrue(latest.has_color and latest.has_depth)
        self.assertEqual(bytes(latest.color.data), bytes([latest.color.data[0]]) * 12)
        self.assertEqual(latest.color.data[0], latest.depth.data[0])
        self.assertAlmostEqual(latest.stamp_delta_sec, .003, places=6)
        queue = self.call(GetImage.Request.QUEUE_PAIR)
        self.assertTrue(queue.success)
        self.assertEqual(queue.pair_id, 1 + ((latest.paired_count - 1) // 3) * 3)
        self.assertLessEqual(queue.queue_size, 3)
        self.assertEqual(queue.color.data[0], queue.depth.data[0])
        for operation, color in [(GetImage.Request.LATEST_COLOR, True),
                                 (GetImage.Request.QUEUE_COLOR, True),
                                 (GetImage.Request.LATEST_DEPTH, False),
                                 (GetImage.Request.QUEUE_DEPTH, False)]:
            response = self.call(operation)
            self.assertTrue(response.success)
            self.assertEqual(response.has_color, color)
            self.assertEqual(response.has_depth, not color)
            self.assertEqual(len(response.depth.data) if color else len(response.color.data), 0)

    def test_unmatched_timestamps_do_not_form_pair(self):
        now = self.node.get_clock().now().nanoseconds
        for color in (True, False):
            image = Image()
            stamp = now + (0 if color else 10**9)
            image.header.stamp.sec, image.header.stamp.nanosec = divmod(stamp, 10**9)
            image.width = image.height = 1
            image.encoding = "rgb8" if color else "16UC1"
            image.step = 3 if color else 2
            image.data = bytes(image.step)
            (self.color if color else self.depth).publish(image)
        deadline = time.monotonic() + .25
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=.01)
        self.assertEqual(self.call(GetImage.Request.STATUS).paired_count, 0)

    def test_expiry_and_clear_preserve_latest(self):
        self.publish_pairs(6)
        self.assertTrue(self.call(GetImage.Request.CLEAR_QUEUE).success)
        self.assertEqual(self.call(GetImage.Request.STATUS).queue_size, 0)
        self.assertTrue(self.call(GetImage.Request.LATEST_PAIR).success)
        self.assertTrue(self.configure(max_image_age_sec=.1).successful)
        self.publish_pairs(6)
        deadline = time.monotonic() + .3
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=.01)
        response = self.call(GetImage.Request.STATUS)
        self.assertEqual(response.queue_size, 0)
        self.assertTrue(response.latest_available)
        response = self.call(GetImage.Request.LATEST_PAIR)
        self.assertTrue(response.success and response.exceeded_max_age)

    def test_dynamic_parameters_and_approved_ranges(self):
        self.publish_pairs(8)
        self.assertTrue(self.configure(queue_length=1, save_every_n_frames=1).successful)
        self.publish_pairs(4)
        response = self.call(GetImage.Request.QUEUE_PAIR)
        self.assertEqual(response.queue_size, 1)
        self.assertEqual(response.pair_id, self.call(GetImage.Request.LATEST_PAIR).pair_id)
        for values in [dict(queue_length=-1), dict(save_every_n_frames=0),
                       dict(max_image_age_sec=-1.0)]:
            self.assertFalse(self.configure(**values).successful)
        self.assertTrue(self.configure(queue_length=0).successful)
        self.publish_pairs(3)
        self.assertEqual(self.call(GetImage.Request.STATUS).queue_size, 0)
        self.assertTrue(self.call(GetImage.Request.LATEST_PAIR).success)


if __name__ == "__main__":
    unittest.main(argv=["queue_test", *remaining], verbosity=2)

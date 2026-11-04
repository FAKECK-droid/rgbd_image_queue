#!/usr/bin/env python3
"""仅提取消费函数验证协议，不初始化 ROS/YOLO。"""
import ast
from pathlib import Path
import struct
from types import SimpleNamespace
import unittest

import cv2
import numpy as np

source = ast.parse((Path(__file__).parents[1] / "scripts/person_goal.py").read_text())
functions = [node for node in source.body
             if isinstance(node, ast.FunctionDef) and node.name == "center_point"]
person = next(node for node in source.body if isinstance(node, ast.ClassDef) and node.name == "PersonGoal")
functions.append(next(node for node in person.body if isinstance(node, ast.FunctionDef) and node.name == "detect"))
namespace = {"cv2": cv2, "np": np, "struct": struct}
exec(compile(ast.Module(body=functions, type_ignores=[]), "person_functions", "exec"), namespace)
center_point = namespace["center_point"]


class CompressedConsumerTest(unittest.TestCase):
    def setUp(self):
        self.info = SimpleNamespace(k=[100., 0., 1., 0., 100., 1., 0., 0., 1.], d=[])
        self.box = [1, 1, 3, 3]

    def depth(self, value, encoding="16UC1"):
        ok, png = cv2.imencode(".png", np.full((4, 4), value, np.uint16))
        self.assertTrue(ok)
        return SimpleNamespace(format=encoding + "; compressedDepth png",
                               data=struct.pack("<iff", 0, 1000., 1.) + png.tobytes())

    def test_16bit_millimeters(self):
        pixel, xyz = center_point(self.box, self.depth(2500), self.info)
        self.assertEqual(pixel, (2, 2))
        np.testing.assert_allclose(xyz, (.025, .025, 2.5))

    def test_float_inverse_quantization(self):
        _, xyz = center_point(self.box, self.depth(501, "32FC1"), self.info)
        np.testing.assert_allclose(xyz, (.02, .02, 2.))

    def test_zero_depth_has_no_point(self):
        for encoding in ("16UC1", "32FC1"):
            self.assertIsNone(center_point(self.box, self.depth(0, encoding), self.info)[1])

    def test_no_person_skips_depth_decode(self):
        ok, jpeg = cv2.imencode(".jpg", np.full((4, 4, 3), 120, np.uint8))
        self.assertTrue(ok)
        data = SimpleNamespace(cpu=lambda: SimpleNamespace(numpy=lambda: np.empty((0, 6))))
        model = SimpleNamespace(predict=lambda *args, **kwargs:
                                [SimpleNamespace(boxes=SimpleNamespace(data=data))])
        node = SimpleNamespace(model=model, confidence=.4, device="cpu", imgsz=416, quantize=None)
        response = SimpleNamespace(color=SimpleNamespace(data=jpeg.tobytes()),
                                   depth=SimpleNamespace(data=b"not-a-png",
                                                         header=SimpleNamespace(frame_id="optical")))
        color, box, pixel, xyz, frame = namespace["detect"](node, response, self.info, None)
        self.assertEqual(color.shape, (4, 4, 3))
        self.assertIsNone(box)
        self.assertEqual(frame, "optical")


if __name__ == "__main__":
    unittest.main(verbosity=2)

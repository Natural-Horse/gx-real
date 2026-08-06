#!/usr/bin/env python3
"""CompressedImage(JPEG) -> sensor_msgs/Image 解压转发，供 rviz2 查看。

rviz2 的 Image 显示不支持 CompressedImage，需要先把相机 topic 转成原始 Image：

  /camera/{front,wrist}/image_raw/compressed  (JPEG)
        ->  /camera/{front,wrist}/image_raw    (bgr8)

用法（robodog 上，先起相机发布器）：
  cd ~/gx-real && source scripts/setup_env.sh
  python3 scripts/decompress_cameras.py
"""

from __future__ import annotations

import argparse

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge


class DecompressCameras(Node):
    def __init__(self) -> None:
        super().__init__("decompress_cameras")
        self.bridge = CvBridge()
        self.pubs: dict[str, object] = {}
        for name in ("front", "wrist"):
            self.create_subscription(
                CompressedImage,
                f"/camera/{name}/image_raw/compressed",
                lambda msg, n=name: self._cb(n, msg),
                5,
            )
            self.pubs[name] = self.create_publisher(
                Image, f"/camera/{name}/image_raw", 5
            )
        self.get_logger().info(
            "decompress -> /camera/front/image_raw, /camera/wrist/image_raw"
        )

    def _cb(self, name: str, msg: CompressedImage) -> None:
        try:
            arr = cv2.imdecode(
                np.frombuffer(bytes(msg.data), dtype=np.uint8), cv2.IMREAD_COLOR
            )
            if arr is None:
                return
            image = self.bridge.cv2_to_imgmsg(arr, encoding="bgr8")
            image.header = msg.header
            self.pubs[name].publish(image)
        except Exception:
            self.get_logger().warning(f"{name}: decode failed")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="相机压缩话题解压转发")
    return parser.parse_args()


def main() -> int:
    _parse_args()
    rclpy.init()
    node = DecompressCameras()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

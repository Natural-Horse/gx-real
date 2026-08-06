#!/usr/bin/env python3
"""无相机时用两张 JPEG 假扮 front/wrist 相机，用于 VLA 链路测试。

用法（在 robodog 上，先 source ROS 环境）：
  python3 scripts/publish_test_cameras.py \
    --front /tmp/sample_front.jpg \
    --wrist /tmp/sample_wrist.jpg \
    --rate 5.0

发布两个 CompressedImage topic：
  /camera/front/image_raw/compressed
  /camera/wrist/image_raw/compressed

仅用于管线联调，不包含真实相机标定/曝光等语义。
"""

from __future__ import annotations

import argparse

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class TestCameraPublisher(Node):
    def __init__(
        self,
        *,
        front_path: str,
        wrist_path: str,
        front_topic: str,
        wrist_topic: str,
        rate: float,
    ) -> None:
        super().__init__("test_camera_publisher")
        self.front_data = _read_jpeg(front_path)
        self.wrist_data = _read_jpeg(wrist_path)
        self.front_pub = self.create_publisher(CompressedImage, front_topic, 5)
        self.wrist_pub = self.create_publisher(CompressedImage, wrist_topic, 5)
        period = 1.0 / max(rate, 0.1)
        self.timer = self.create_timer(period, self._publish)
        self.get_logger().info(
            f"test cameras: front={front_path} wrist={wrist_path} rate={rate:.1f}Hz"
        )

    def _publish(self) -> None:
        now = self.get_clock().now().to_msg()
        self.front_pub.publish(_jpeg_msg(now, self.front_data))
        self.wrist_pub.publish(_jpeg_msg(now, self.wrist_data))


def _read_jpeg(path: str) -> bytes:
    with open(path, "rb") as stream:
        data = stream.read()
    if data[:2] != b"\xff\xd8":
        raise ValueError(f"not a JPEG file: {path}")
    return data


def _jpeg_msg(stamp, data: bytes) -> CompressedImage:
    msg = CompressedImage()
    msg.header.stamp = stamp
    msg.header.frame_id = "camera"
    msg.format = "jpeg"
    msg.data = data
    return msg


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="发布测试用 front/wrist JPEG 相机流")
    parser.add_argument("--front", required=True, help="front 相机 JPEG 路径")
    parser.add_argument("--wrist", required=True, help="wrist 相机 JPEG 路径")
    parser.add_argument("--front-topic", default="/camera/front/image_raw/compressed")
    parser.add_argument("--wrist-topic", default="/camera/wrist/image_raw/compressed")
    parser.add_argument("--rate", type=float, default=5.0, help="发布频率（Hz）")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    rclpy.init()
    node = TestCameraPublisher(
        front_path=args.front,
        wrist_path=args.wrist,
        front_topic=args.front_topic,
        wrist_topic=args.wrist_topic,
        rate=args.rate,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""真机 RealSense 相机发布器：RGB 流 -> JPEG -> CompressedImage topic。

在 robodog 上启动（先 source ROS 环境）：
  python3 scripts/publish_real_cameras.py \
    --front-dev /dev/video4 \
    --wrist-dev /dev/video10 \
    --rate 5.0

发布：
  /camera/front/image_raw/compressed
  /camera/wrist/image_raw/compressed

设备映射（按 robodog 当前枚举，需现场确认物理位置）：
  8086:1156（D436，腕部）彩色节点 -> --wrist-dev 默认 /dev/video10
  8086:0b3a（前置）       彩色节点 -> --front-dev 默认 /dev/video4
两个相机目前都在 USB2 上，只能出低分辨率模式，VLA 输入 224x224 够用。
"""

from __future__ import annotations

import argparse
import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class RealCameraPublisher(Node):
    def __init__(
        self,
        *,
        front_dev: str,
        wrist_dev: str,
        front_topic: str,
        wrist_topic: str,
        rate: float,
        jpeg_quality: int,
    ) -> None:
        super().__init__("real_camera_publisher")
        self.front_dev = front_dev
        self.wrist_dev = wrist_dev
        self.jpeg_quality = jpeg_quality
        self.front_pub = self.create_publisher(CompressedImage, front_topic, 5)
        self.wrist_pub = self.create_publisher(CompressedImage, wrist_topic, 5)
        self.front_cap = self._open_camera(front_dev)
        self.wrist_cap = self._open_camera(wrist_dev)
        period = 1.0 / max(rate, 0.1)
        self.timer = self.create_timer(period, self._publish)
        self.get_logger().info(
            f"real cameras: front={front_dev} wrist={wrist_dev} rate={rate:.1f}Hz"
        )

    @staticmethod
    def _open_camera(device: str) -> cv2.VideoCapture:
        # Jetson 上 OpenCV 默认可能走 GStreamer 后端读不了 /dev/video*，
        # 强制 V4L2。
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(f"无法打开相机设备 {device}")
        return cap

    def _publish(self) -> None:
        now = self.get_clock().now().to_msg()
        self._publish_one(self.front_cap, self.front_pub, self.front_dev, now, "front")
        self._publish_one(self.wrist_cap, self.wrist_pub, self.wrist_dev, now, "wrist")

    def _publish_one(self, cap, pub, device: str, stamp, name: str) -> None:
        ok, frame = cap.read()
        if not ok or frame is None:
            self.get_logger().warning(f"{name}({device}) 读取失败，尝试重开")
            cap.release()
            time.sleep(0.2)
            cap.open(device, cv2.CAP_V4L2)
            return
        ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = name
        msg.format = "jpeg"
        msg.data = encoded.tobytes()
        pub.publish(msg)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="真机 RealSense 相机 JPEG 发布器")
    parser.add_argument("--front-dev", default="/dev/video4")
    parser.add_argument("--wrist-dev", default="/dev/video10")
    parser.add_argument("--front-topic", default="/camera/front/image_raw/compressed")
    parser.add_argument("--wrist-topic", default="/camera/wrist/image_raw/compressed")
    parser.add_argument("--rate", type=float, default=5.0)
    parser.add_argument("--jpeg-quality", type=int, default=85)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    rclpy.init()
    node = RealCameraPublisher(
        front_dev=args.front_dev,
        wrist_dev=args.wrist_dev,
        front_topic=args.front_topic,
        wrist_topic=args.wrist_topic,
        rate=args.rate,
        jpeg_quality=args.jpeg_quality,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

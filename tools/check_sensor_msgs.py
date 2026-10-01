#!/usr/bin/env python3
"""订阅传感器话题并做“自洽性”检查。

只看到“有频率”不等于数据是对的 —— 这个脚本检查的是内容层面的自洽：
  * Image      : len(data) 是否等于 step * height（不相等说明转换时长度算错了，
                 RViz 会显示错乱，教材里的视觉代码也会读到垃圾）
  * PointCloud2: row_step == point_step * width、len(data) == row_step * height
  * CameraInfo : K/P 是否非零（全零说明内参没转过来）
  * Imu        : frame_id 是否非空、静止时 gravity 是否在合理范围

用法（容器内，仿真正在跑时）：
    python3 /workspace/my-slam/tools/check_sensor_msgs.py
退出码 0 = 全部通过，1 = 有检查项不通过。
"""

import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2


class Checker(Node):
    def __init__(self, timeout_sec=8.0):
        super().__init__('sensor_msg_checker')
        self.deadline = time.time() + timeout_sec
        self.pending = {
            '/camera/image': None,
            '/camera/depth_image': None,
            '/camera/camera_info': None,
            '/camera/points': None,
            '/imu': None,
        }
        self.create_subscription(Image, '/camera/image', self._wrap('/camera/image'), 10)
        self.create_subscription(Image, '/camera/depth_image', self._wrap('/camera/depth_image'), 10)
        self.create_subscription(
            CameraInfo, '/camera/camera_info', self._wrap('/camera/camera_info'), 10)
        self.create_subscription(
            PointCloud2, '/camera/points', self._wrap('/camera/points'), 10)
        self.create_subscription(Imu, '/imu', self._wrap('/imu'), 10)

    def _wrap(self, topic):
        def cb(msg):
            if self.pending.get(topic) is None:
                self.pending[topic] = msg
        return cb

    def done(self):
        return all(v is not None for v in self.pending.values()) or time.time() > self.deadline


def check_image(topic, msg):
    expected = msg.step * msg.height
    ok = len(msg.data) == expected
    print(f'  {topic:<22} {msg.width}x{msg.height} {msg.encoding:<8} '
          f'step={msg.step} len(data)={len(msg.data)} 期望={expected} '
          f'{"OK" if ok else "不一致!"}')
    return ok


def check_points(topic, msg):
    checks = {
        'row_step == point_step*width': msg.row_step == msg.point_step * msg.width,
        'len(data) == row_step*height': len(msg.data) == msg.row_step * msg.height,
    }
    ok = all(checks.values())
    detail = ', '.join(f'{k}={v}' for k, v in checks.items())
    print(f'  {topic:<22} {msg.width}x{msg.height} point_step={msg.point_step} '
          f'fields={len(msg.fields)} {detail} {"OK" if ok else "不一致!"}')
    return ok


def check_camera_info(topic, msg):
    ok = any(v != 0.0 for v in msg.k) and any(v != 0.0 for v in msg.p)
    print(f'  {topic:<22} {msg.width}x{msg.height} model={msg.distortion_model} '
          f'K/P 非零={"OK" if ok else "全零!"}')
    return ok


def check_imu(topic, msg):
    g = msg.linear_acceleration.z
    frame_ok = bool(msg.header.frame_id)
    # 静止时重力约 9.8，允许 xacro 里配置的噪声/偏置带来的偏差
    g_ok = 5.0 < abs(g) < 15.0
    print(f'  {topic:<22} frame_id={msg.header.frame_id!r} '
          f'linear_acceleration.z={g:.3f} '
          f'{"OK" if (frame_ok and g_ok) else "异常!"}')
    return frame_ok and g_ok


def main():
    rclpy.init()
    node = Checker()
    while not node.done():
        rclpy.spin_once(node, timeout_sec=0.5)

    results = []
    print('传感器消息自洽性检查：')
    for topic, msg in node.pending.items():
        if msg is None:
            print(f'  {topic:<22} 没收到消息（超时）')
            results.append(False)
            continue
        if isinstance(msg, Image):
            results.append(check_image(topic, msg))
        elif isinstance(msg, PointCloud2):
            results.append(check_points(topic, msg))
        elif isinstance(msg, CameraInfo):
            results.append(check_camera_info(topic, msg))
        elif isinstance(msg, Imu):
            results.append(check_imu(topic, msg))

    rclpy.shutdown()
    ok = all(results)
    print('结论：' + ('全部通过' if ok else '有检查项未通过'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())

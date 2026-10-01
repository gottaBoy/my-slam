#!/usr/bin/env python3
"""
验证 diff_drive_controller 的 wheel_separation 是否与 URDF 真实几何一致。

原理
----
配置里 open_loop: true，所以：
  * 控制器按 cmd_vel 和 wheel_separation(b_cmd) 算出左右轮目标转速
  * /odom 再用同一个 b_cmd 反算回来 => /odom 的积分角恒等于指令角（看不出问题）
  * 但机器人"物理上"真实转角 = 指令角 * (b_cmd / b_true)
因此唯一能分辨对错的判据是 **Gazebo 真值**（gz model -m <model> -p）。

用法
----
  python3 tools/verify_wheel_separation.py --duration 2.0 --w 0.6
"""

import argparse
import math
import re
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

CTRL = '/fishbot_diff_drive_controller'


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def gz_yaw(model):
    """从 Gazebo 取真值 yaw（rad）。"""
    out = subprocess.run(['gz', 'model', '-m', model, '-p'],
                         capture_output=True, text=True, timeout=20)
    txt = out.stdout + out.stderr
    rows = re.findall(r'\[(-?[\d.eE+-]+) +(-?[\d.eE+-]+) +(-?[\d.eE+-]+)\]', txt)
    if len(rows) < 2:
        raise RuntimeError('无法解析 gz pose 输出:\n' + txt)
    rpy = [float(v) for v in rows[-1]]      # 最后一行是 RPY
    return rpy[2]


def get_param(name):
    out = subprocess.run(['ros2', 'param', 'get', CTRL, name],
                         capture_output=True, text=True, timeout=20)
    return out.stdout.strip().splitlines()[-1]


class Verifier(Node):
    def __init__(self):
        super().__init__('wheel_sep_verifier')
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.odom = None

    def _on_odom(self, msg):
        self.odom = msg

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_odom(self, timeout=20.0):
        end = time.time() + timeout
        while self.odom is None and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.odom is None:
            raise RuntimeError('20s 内没有收到 /odom，仿真起来了吗？')

    def rotate(self, w, duration):
        """发布固定角速度，返回实际发布时长（秒）。"""
        period = 1.0 / 20.0
        t0 = time.time()
        end = t0 + duration
        msg = TwistStamped()
        msg.twist.angular.z = float(w)
        while time.time() < end:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)
        actual = time.time() - t0
        # 停车
        stop = TwistStamped()
        send = time.time() + 0.3
        while time.time() < send:
            self.pub.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)
        return actual


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default='fishbot')
    ap.add_argument('--w', type=float, default=0.6, help='角速度 rad/s')
    ap.add_argument('--duration', type=float, default=2.0, help='持续秒数')
    ap.add_argument('--settle', type=float, default=2.0, help='停车后等待秒数')
    args = ap.parse_args()

    rclpy.init()
    node = Verifier()
    try:
        node.wait_odom()
        node.spin_for(0.5)

        b = get_param('wheel_separation')
        odom0 = yaw_from_quat(node.odom.pose.pose.orientation)
        gz0 = gz_yaw(args.model)

        actual = node.rotate(args.w, args.duration)
        node.spin_for(args.settle)

        odom1 = yaw_from_quat(node.odom.pose.pose.orientation)
        gz1 = gz_yaw(args.model)

        d_odom = wrap_pi(odom1 - odom0)
        d_gz = wrap_pi(gz1 - gz0)
        cmd = args.w * actual

        def err(actual_v):
            if abs(cmd) < 1e-9:
                return float('nan')
            return (actual_v - cmd) / cmd * 100.0

        print()
        print('=' * 62)
        print(f'  wheel_separation 当前值(b_cmd) : {b}')
        print(f'  Gazebo 真值真实轮距(b_true)    : 0.20   (URDF y=±0.10)')
        print('-' * 62)
        print(f'  指令转角 (w*T)                 : {cmd:8.4f} rad')
        print(f'  /odom 积分角                   : {d_odom:8.4f} rad  误差 {err(d_odom):+7.2f}%')
        print(f'  Gazebo 真值角                  : {d_gz:8.4f} rad  误差 {err(d_gz):+7.2f}%')
        print('-' * 62)
        if abs(err(d_gz)) < 5.0:
            print('  判定: ✅ 真值跟随指令，轮距配置正确')
        else:
            print('  判定: ❌ 真值明显偏离指令，轮距配置错误')
            try:
                ratio = float(b) / 0.20
                print(f'        b_cmd/b_true = {ratio:.3f} => 真值应为 {cmd * ratio:.4f} rad')
            except ValueError:
                pass
        print('=' * 62)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())

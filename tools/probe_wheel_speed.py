#!/usr/bin/env python3
"""
用「轮子实际转速」反推控制器真正使用的 wheel_separation。

先给一条纯旋转指令 (v=0, w=W)，设备上应满足:
    w_left_wheel  = -W * b_eff / (2R)
    w_right_wheel = +W * b_eff / (2R)
  =>  b_eff = R * (w_right_wheel - w_left_wheel) / W

这与动力学/打滑无关，只看控制器下发的速度，因此可用来判定
「运行时 ros2 param set wheel_separation 是否真的生效」。
"""

import argparse
import subprocess
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState

CTRL = '/fishbot_diff_drive_controller'


def get_param(name):
    out = subprocess.run(['ros2', 'param', 'get', CTRL, name],
                         capture_output=True, text=True, timeout=20)
    return out.stdout.strip().splitlines()[-1].split(':')[-1].strip()


class Probe(Node):
    def __init__(self, left, right, radius):
        super().__init__('probe_wheel_speed')
        self.left, self.right, self.radius = left, right, radius
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.create_subscription(JointState, '/joint_states', self._cb, 10)
        self.wl = self.wr = None
        self.t = None

    def _cb(self, msg):
        if self.left in msg.name and self.right in msg.name:
            self.wl = msg.velocity[msg.name.index(self.left)]
            self.wr = msg.velocity[msg.name.index(self.right)]
            self.t = time.time()

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def wait_joint(self, timeout=20.0):
        end = time.time() + timeout
        while self.wl is None and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.wl is None:
            raise RuntimeError('收不到 /joint_states 里指定的关节')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--w', type=float, default=0.6)
    ap.add_argument('--radius', type=float, default=0.032)
    ap.add_argument('--left', default='left_wheel_joint')
    ap.add_argument('--right', default='right_wheel_joint')
    ap.add_argument('--dur', type=float, default=3.0, help='总发布时长')
    ap.add_argument('--steady', type=float, default=1.0,
                    help='只统计最后这段（稳态窗口）的样本')
    ap.add_argument('--label', default='')
    args = ap.parse_args()

    rclpy.init()
    node = Probe(args.left, args.right, args.radius)
    try:
        node.wait_joint()
        node.spin_for(0.3)

        msg = TwistStamped()
        msg.twist.angular.z = args.w
        steady_from = time.time() + (args.dur - args.steady)
        samples = []
        end = time.time() + args.dur
        while time.time() < end:
            msg.header.stamp = node.get_clock().now().to_msg()
            node.pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.03)
            if node.wl is not None and time.time() >= steady_from:
                samples.append((node.wl, node.wr))
        stop = TwistStamped()
        for _ in range(8):
            node.pub.publish(stop)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.03)

        if not samples:
            raise RuntimeError('稳态窗口内没采到样本')
        wl = sum(s[0] for s in samples) / len(samples)
        wr = sum(s[1] for s in samples) / len(samples)
        b_eff = args.radius * (wr - wl) / args.w
        spread = max(abs(s[1]) for s in samples) - min(abs(s[1]) for s in samples)

        print()
        print('=' * 62)
        if args.label:
            print(f'  # {args.label}')
        print(f'  参数里的 wheel_separation        : {get_param("wheel_separation")}')
        print(f'  由轮速反推的实际 wheel_separation: {b_eff:.4f}')
        print('-' * 62)
        print(f'  指令 w                          : {args.w:.4f} rad/s')
        print(f'  left_wheel_joint  速度           : {wl:+.4f} rad/s')
        print(f'  right_wheel_joint 速度           : {wr:+.4f} rad/s')
        pv = float(get_param('wheel_separation'))
        print(f'  若参数生效，轮速应为             : {abs(args.w) * pv / (2 * args.radius):.4f} rad/s'
              f'   (实测 {abs(wr):.4f})')
        print(f'  若用 URDF 真值 0.20，轮速应为    : {abs(args.w) * 0.20 / (2 * args.radius):.4f} rad/s')
        print(f'  稳态样本数 / 抖动                : {len(samples)} / {spread:.5f} rad/s')
        print('=' * 62)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())


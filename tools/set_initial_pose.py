#!/usr/bin/env python3
"""
设置 AMCL 的初始位姿。

为什么要这个工具：
  书上的 init_robot_pose 固定发 (0,0,0)。仿真刚起来时机器人确实在 (0,0) 且朝
  向 0，没问题；但只要机器人被转动过（例如做过轮距标定、手动 teleop 过），
  再发 (0,0,0) 就会让 AMCL 的朝向和真实朝向差很多，之后"导航成功"也只是在
  错误的坐标系里成功，没有意义。

用法:
  python3 tools/set_initial_pose.py --from-gz          # 以 Gazebo 真值为准
  python3 tools/set_initial_pose.py --x 0 --y 0 --yaw 0
"""

import argparse
import math
import re
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


def gz_pose(model):
    out = subprocess.run(['gz', 'model', '-m', model, '-p'],
                         capture_output=True, text=True, timeout=20)
    txt = out.stdout + out.stderr
    rows = re.findall(r'\[(-?[\d.eE+-]+) +(-?[\d.eE+-]+) +(-?[\d.eE+-]+)\]', txt)
    if len(rows) < 2:
        raise RuntimeError('无法解析 gz model 输出:\n' + txt)
    xyz = [float(v) for v in rows[-2]]
    rpy = [float(v) for v in rows[-1]]
    return xyz[0], xyz[1], rpy[2]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--from-gz', action='store_true',
                    help='用 Gazebo 真值作为初始位姿')
    ap.add_argument('--model', default='fishbot')
    ap.add_argument('--x', type=float, default=0.0)
    ap.add_argument('--y', type=float, default=0.0)
    ap.add_argument('--yaw', type=float, default=0.0)
    ap.add_argument('--cov', type=float, default=0.25,
                    help='x/y 方差（默认 0.25 = 标准差 0.5m）')
    args = ap.parse_args()

    if args.from_gz:
        x, y, yaw = gz_pose(args.model)
        src = 'Gazebo 真值'
    else:
        x, y, yaw = args.x, args.y, args.yaw
        src = '命令行指定'

    rclpy.init()
    node = Node('set_initial_pose')
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    c = [args.cov, args.cov, 0.0, 0.0, 0.0, args.cov / 4.0]
    for i, v in enumerate(c):
        msg.pose.covariance[i * 7] = v

    # 连发几次，避免订阅端尚未就绪导致丢失
    for _ in range(5):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.2)

    print(f'[set_initial_pose] 来源={src}')
    print(f'  x={x:.4f}  y={y:.4f}  yaw={yaw:.4f} rad ({math.degrees(yaw):.2f} deg)')
    print(f'  已发布到 /initialpose（frame=map）')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

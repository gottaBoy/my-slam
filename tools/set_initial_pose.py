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
from rclpy.duration import Duration
from rclpy.parameter import Parameter
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
    ap.add_argument('--backdate', type=float, default=0.3,
                    help='时间戳往前挪多少秒（默认 0.3）')
    ap.add_argument('--tries', type=int, default=10,
                    help='重复发布次数（默认 10）')
    ap.add_argument('--wall-clock', action='store_true',
                    help='不用仿真时间（默认用）')
    ap.add_argument('--stamp', choices=['zero', 'now'], default='zero',
                    help='时间戳用 0（tf2 会取最新可用变换，默认）还是当前时刻')
    args = ap.parse_args()

    if args.from_gz:
        x, y, yaw = gz_pose(args.model)
        src = 'Gazebo 真值'
    else:
        x, y, yaw = args.x, args.y, args.yaw
        src = '命令行指定'

    rclpy.init()
    node = Node('set_initial_pose')
    if not args.wall_clock:
        # 关键：AMCL 用仿真时间，如果用墙钟时间做时间戳，会被判成
        # 「extrapolation into the future」而**直接丢弃**这次初始位姿：
        #   [amcl]: Failed to transform initial pose in time (...)
        # 必须先切到仿真时间。切完要等一下让 /clock 到位。
        node.set_parameters(
            [Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        deadline = time.time() + 10.0
        while node.get_clock().now().nanoseconds == 0 and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        print(f'[set_initial_pose] use_sim_time=True，'
              f'当前仿真时间 {node.get_clock().now().nanoseconds / 1e9:.3f}s')
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    def make_msg():
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        # 时间戳这里踩过坑，把实测结论记下来，别再重复试错：
        #
        # odom->base_footprint 这条 TF 比仿真时钟滞后约 0.2~0.3s。因此无论用
        # 「当前时刻」、往前回退 0.2s / 0.3s、还是用零时间戳，AMCL 都会打：
        #   Failed to transform initial pose in time (Lookup would require
        #   extrapolation into the future. Requested time A but the latest
        #   data is at time B, when looking up transform from frame
        #   [base_footprint] to frame [odom])
        #
        # 但这行报错**不影响位姿生效**。三种时间戳策略各测一次，设完之后
        # /amcl_pose 与 Gazebo 真值的差分别是 0.016m / 0.053m / 0.036m；
        # 而且设定前 AMCL 一直在打「Please set the initial pose」（表示没有
        # 位姿），设定后立刻正常发布 map->odom —— 说明位姿确实被采纳了。
        #
        # 所以这里默认用零时间戳（tf2 对 time=0 的语义是「取最新可用变换」，
        # 不做外推），配合多次重复发布；那行报错当噪声看待即可。
        if args.stamp == 'zero':
            stamp = msg.header.stamp
        else:
            stamp = (node.get_clock().now()
                     - Duration(seconds=args.backdate)).to_msg()
            msg.header.stamp = stamp
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        c = [args.cov, args.cov, 0.0, 0.0, 0.0, args.cov / 4.0]
        for i, v in enumerate(c):
            msg.pose.covariance[i * 7] = v
        return msg, stamp

    first_stamp = None
    for _ in range(args.tries):
        m, st = make_msg()
        if first_stamp is None:
            first_stamp = st
        pub.publish(m)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(0.2)

    print(f'[set_initial_pose] 来源={src}')
    print(f'  x={x:.4f}  y={y:.4f}  yaw={yaw:.4f} rad ({math.degrees(yaw):.2f} deg)')
    if args.stamp == 'zero':
        print('  时间戳=0（tf2 取最新可用变换，避免外推失败）')
    else:
        print(f'  时间戳={first_stamp.sec}.{first_stamp.nanosec:09d}'
              f'（比当前早 {args.backdate}s）')
    print(f'  已发布 {args.tries} 次到 /initialpose（frame=map）')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

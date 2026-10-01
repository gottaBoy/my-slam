#!/usr/bin/env python3
"""
同时记录 Nav2 速度链上每一级的话题，用来定位「谁把速度清零了」。

链条（本项目实测拓扑，见 ros2 topic info）：
    controller_server  ->  /cmd_vel_nav
    velocity_smoother  ->  /cmd_vel_smoothed
    collision_monitor  ->  /cmd_vel            <- 下游就是差速控制器
    docking_server     ->  /cmd_vel            （只在对接时发）
    mybot_diff_drive_controller  <-- 订阅 /cmd_vel（BEST_EFFORT）

判读方法：
  * 每级都非零，但车不动        -> 物理问题（被卡住/轮子打滑）
  * 前级非零、某一级开始变 0     -> 就是那一级把指令吃掉了，去看它的日志
  * 第一级就是 0                -> controller_server 自己没输出，
                                   看它是不是认为路径无效/到了终点/被 costmap 挡住

用法（容器内）:
  python3 tools/probe_cmd_chain.py --seconds 200 --out /tmp/chain.txt
  # 另开一个终端下 goal
"""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

TOPICS = ['/cmd_vel_nav', '/cmd_vel_smoothed', '/cmd_vel', '/cmd_vel_teleop']


class Recorder(Node):
    def __init__(self):
        super().__init__('cmd_chain_recorder')
        self.latest = {}
        self.counts = {}
        for t in TOPICS:
            self.latest[t] = None
            self.counts[t] = 0
            self.create_subscription(
                TwistStamped, t, self._make_cb(t), 10)

    def _make_cb(self, topic):
        def cb(msg):
            self.latest[topic] = (
                msg.twist.linear.x, msg.twist.angular.z)
            self.counts[topic] += 1
        return cb


def fmt(v):
    if v is None:
        return '   -      -  '
    return f'{v[0]:+6.3f} {v[1]:+6.3f}'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=120.0)
    ap.add_argument('--out', default='/tmp/chain.txt')
    ap.add_argument('--period', type=float, default=0.5,
                    help='采样周期（秒）')
    args = ap.parse_args()

    rclpy.init()
    node = Recorder()

    header = ('wall_time       ' +
              '  '.join(f'{t:>15}' for t in TOPICS))
    lines = [header, ' ' * 16 + '  '.join(f'{"vx":>6} {"wz":>6}' for _ in TOPICS)]

    t0 = time.time()
    next_t = t0
    try:
        while time.time() - t0 < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.time()
            if now >= next_t:
                next_t = now + args.period
                vals = '  '.join(fmt(node.latest[t]) for t in TOPICS)
                lines.append(f'{now:.2f}  {vals}')
    except KeyboardInterrupt:
        pass

    with open(args.out, 'w') as f:
        f.write('\n'.join(lines) + '\n')

    print(f'写入 {args.out}，{len(lines) - 2} 个采样点，时长 {time.time() - t0:.1f} s')
    print()
    print('各话题收到的消息数:')
    for t in TOPICS:
        print(f'  {t:<20} {node.counts[t]}')
    print()
    print('=== 各话题的速度范围（非零才算"在发指令"）===')
    # lines[0] 和 lines[1] 都是表头，数据从 lines[2] 开始
    col = [ln.split() for ln in lines[2:]]
    for i, t in enumerate(TOPICS):
        idx = 1 + 2 * i
        vx = [float(p[idx]) for p in col if len(p) > idx + 1]
        if not vx:
            print(f'  {t:<20} （没有数据）')
            continue
        nz = sum(1 for v in vx if abs(v) > 0.005)
        print(f'  {t:<20} vx ∈ [{min(vx):+.3f}, {max(vx):+.3f}]   '
              f'非零采样 {nz}/{len(vx)}')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

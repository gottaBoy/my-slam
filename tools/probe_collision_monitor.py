#!/usr/bin/env python3
"""
`collision_monitor` 安全闸门取证工具 —— 验证「它在机器人撞上之前真的会减速/停住」。

为什么需要单独一个工具：Nav2 的全局规划器**永远不会**朝墙冲
（它会留出 `robot_radius` + 膨胀的余量），所以走正常导航**测不出安全闸门有没有用**。
只能绕开规划器，直接往 `collision_monitor` 的输入端喂指令。

本项目的链（见 `nav2_params.yaml` 的注释）：

```text
controller_server/behavior_server -> cmd_vel_nav
  -> velocity_smoother -> cmd_vel_smoothed
    -> collision_monitor -> cmd_vel        <- 机器人实际收
```

`collision_monitor` 的配置：
- `cmd_vel_in_topic: cmd_vel_smoothed`、`cmd_vel_out_topic: cmd_vel`
- `polygons: ["FootprintApproach"]`，`action_type: "approach"`
- `time_before_collision: 1.2`、`simulation_time_step: 0.1`
- `footprint_topic: /local_costmap/published_footprint`

`action_type: approach` 的语义：**按「再过 `time_before_collision` 秒就会撞」来按比例降速**，
不是一刀切急停。所以预期看到的是 **`cmd_vel` 随距离减小而渐降，最后到 0**。

⚠️ 直接往 `cmd_vel_smoothed` 喂指令会和 `velocity_smoother` 抢话题。
正确做法：起 Nav2 时用 `params_file:=` 把 `collision_monitor.cmd_vel_in_topic`
改成私有话题（例如 `my_in`），这样输入端只有本工具一个发布者：

```bash
cp .../nav2_params.yaml /tmp/tune.yaml
sed -i 's/cmd_vel_in_topic: "cmd_vel_smoothed"/cmd_vel_in_topic: "my_in"/' /tmp/tune.yaml
# 用 /tmp/tune.yaml 重启 Nav2，再设初始位姿
python3 tools/probe_collision_monitor.py --in /my_in --out /cmd_vel
```

工具自动分两阶段：
  阶段 1：原地旋转，直到正前方 ±30° 内出现 < `--search` 米的障碍 → 此时车头对着墙
  阶段 2：持续发 `--v` 前进，记录「发出的 v / 放行的 v / 前方距离 / 真值位置」

最终判定：**放行的 v 是否随距离减小而下降、并在撞上之前归零**。
"""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan


class Probe:
    def __init__(self, node, args):
        self.node = node
        self.args = args
        self.tv = 0.0
        self.tw = 0.0
        self.scan = None
        self.out = None
        self.log = []
        self.in_pub = node.create_publisher(TwistStamped, args.inp, 10)
        node.create_timer(0.05, self._pub)              # 20 Hz 独立定时器（见 F-19）
        node.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        node.create_subscription(TwistStamped, args.outp, self._on_out, 10)

    def _pub(self):
        m = TwistStamped()
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.twist.linear.x, m.twist.angular.z = self.tv, self.tw
        self.in_pub.publish(m)

    def _on_scan(self, m):
        self.scan = m

    def _on_out(self, m):
        self.out = (m.twist.linear.x, m.twist.angular.z)

    def front(self, half_deg=30.0, cap=5.0):
        s = self.scan
        if s is None or not s.ranges:
            return cap
        inc = s.angle_increment or 0.01
        i0 = max(0, int(math.ceil((math.radians(-half_deg) - s.angle_min) / inc)))
        i1 = min(len(s.ranges) - 1,
                 int(math.floor((math.radians(half_deg) - s.angle_min) / inc)))
        best = cap
        for i in range(i0, i1 + 1):
            rr = s.ranges[i]
            if rr is not None and s.range_min <= rr <= min(s.range_max, cap) and rr < best:
                best = rr
        return best

    def spin(self, dur):
        t = time.time()
        while time.time() - t < dur:
            rclpy.spin_once(self.node, timeout_sec=0.02)


def main():
    ap = argparse.ArgumentParser(description='collision_monitor 安全闸门取证')
    ap.add_argument('--in', dest='inp', default='/my_in',
                    help='collision_monitor 的输入话题（应与 cmd_vel_in_topic 一致）')
    ap.add_argument('--out', dest='outp', default='/cmd_vel',
                    help='collision_monitor 的输出话题')
    ap.add_argument('--v', type=float, default=0.25, help='阶段 2 的前进速度 (m/s)')
    ap.add_argument('--search', type=float, default=2.0,
                    help='阶段 1 转到「正前方有这个距离内的障碍」为止 (m)')
    ap.add_argument('--secs', type=float, default=25.0, help='阶段 2 最长时长 (s)')
    ap.add_argument('--period', type=float, default=0.4, help='采样周期 (s)')
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node(
        'probe_collision_monitor',
        parameter_overrides=[Parameter('use_sim_time', value=True)])
    p = Probe(node, args)
    p.spin(3.0)

    print('输入 %s   输出 %s' % (args.inp, args.outp))
    print('')
    print('=== 阶段 1：原地旋转，让车头对着墙 ===')
    t0 = time.time()
    while time.time() - t0 < 30.0:
        rclpy.spin_once(node, timeout_sec=0.02)
        d = p.front()
        if d < args.search:
            break
        p.tv, p.tw = 0.0, 0.6
    p.tv = p.tw = 0.0
    p.spin(1.0)
    print('  停转：正前方 %.2f m 处有障碍（阈值 %.2f m）' % (p.front(), args.search))

    print('')
    print('=== 阶段 2：持续发 %.2f m/s，看放行多少 ===' % args.v)
    print('')
    print('  %-7s %-11s %-11s %-11s %s'
          % ('时间(s)', '前方(m)', '发出 v', '放行 v', '比例'))
    print('  ' + '-' * 52)
    p.tv, p.tw = args.v, 0.0
    t0 = time.time()
    next_t = t0
    stopped_at = None
    while time.time() - t0 < args.secs:
        rclpy.spin_once(node, timeout_sec=0.02)
        now = time.time()
        if now >= next_t:
            next_t = now + args.period
            d = p.front()
            ov = p.out[0] if p.out else float('nan')
            ratio = (ov / args.v) if args.v else float('nan')
            print('  %-7.1f %-11.2f %-11.2f %-11.3f %.2f'
                  % (now - t0, d, args.v, ov, ratio))
            if stopped_at is None and abs(ov) < 0.005 and now - t0 > 1.0:
                stopped_at = d
        if p.front() < 0.18:
            break
    p.tv = p.tw = 0.0
    p.spin(1.0)

    print('')
    print('=== 判定 ===')
    if stopped_at is not None:
        print('  ✅ 放行速度在「前方还有 %.2f m」时已归零 → 安全闸门生效' % stopped_at)
    else:
        print('  ⚠️ 整个过程中放行速度始终等于发出速度 → 闸门没起作用（或没触发）')
        print('     查：polygons 是否 enabled、observation_sources 收到 /scan 了吗、')
        print('         base_frame_id 与 footprint_topic 是否对得上。')

    node.destroy_node()
    return 0


if __name__ == '__main__':
    sys.exit(main())

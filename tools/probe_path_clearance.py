#!/usr/bin/env python3
"""
路径净空剖面 —— 调一次规划器，把返回路径上**每个点离最近障碍的距离**算出来。

为什么需要它（工具集里的分工）：

| 工具 | 回答的问题 |
| --- | --- |
| `probe_planner.py` | 长度 / 代价积分 / 沿途最高导航代价 —— 「绕不绕」 |
| `map_reachability.py` | 两点之间**最宽路线的最窄处** —— 「过不过得去」 |
| **本工具** | **这条路径整体离墙有多远？分布长什么样？** |

验证「让路径尽量贴墙走」这类改动时，靠的就是这个剖面。

判据（本项目实测，见 `问题记录.md` E-20 改动 3；起点 (0,0) → (1.0, 3.0)，直线 3.162 m）：

| 规划器配置 | 路径长度 | 中间 60% 净空均值 |
| --- | --- | --- |
| 直线插值（`wall_clearance=0`，默认）| 3.162 m | 1.015 m |
| 贴墙 `wall_clearance=0.30` | **4.126 m** | **0.311 m** |
| 贴墙 `wall_clearance=0.10` | 3.162 m | 1.015 m（被安全兜底退回，退化成直线）|

⚠️ 净空统一用**静态地图**（`map_reachability.Map.clearance`）算，**不读 costmap** ——
「离墙多远」是纯几何量，跟膨胀参数无关；要验膨胀公式请用 `probe_map_vs_costmap.py`。

⚠️ 机器人在原地时 `compute_path_to_pose` 的 `start` 要自己给（本工具用 `--from`）；
如果 Nav2 刚起来还没设初始位姿，`planner_server` 可能不是 active，目标会被直接拒
（`Action server is inactive. Rejecting the goal.`）—— 那是 `问题记录.md` E-18。

用法（容器内，Nav2 已在跑）:
  python3 tools/probe_path_clearance.py --from 0 0 --to 1.0 3.0
  python3 tools/probe_path_clearance.py --from 0 0 --to 1.0 3.0 --dry   # 只算直线对照
"""

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from nav2_msgs.action import ComputePathToPose  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402

from map_reachability import Map  # noqa: E402

INSCRIBED = 0.2249   # 生效内切半径（见 问题记录 E-15；扫相符率的峰值其实是 0.2255）


def straight_control(m, x0, y0, x1, y1, n=200):
    """直线对照：真实长度 + 沿线的净空。不依赖规划器，规划失败时也有数据。"""
    t = np.linspace(0.0, 1.0, n)
    xs = x0 + t * (x1 - x0)
    ys = y0 + t * (y1 - y0)
    cl = np.array([m.clearance(float(x), float(y)) for x, y in zip(xs, ys)])
    return math.hypot(x1 - x0, y1 - y0), cl


def report(tag, length, cl, indent='  '):
    print(f'{indent}路径长度 : {length:.3f} m')
    print(f'{indent}净空 最小/均值/最大: '
          f'{cl.min():.3f} / {cl.mean():.3f} / {cl.max():.3f} m')
    if len(cl) <= 10:
        return
    # 只看中间 60% —— 两端被起终点钉住，不代表「贴墙效果」
    k = max(1, int(len(cl) * 0.2))
    mid = cl[k:-k]
    print(f'{indent}中间60% 最小/均值/最大: '
          f'{mid.min():.3f} / {mid.mean():.3f} / {mid.max():.3f} m')
    row = '  '.join(f'<={t:.2f}:{100.0 * float((mid <= t).mean()):5.1f}%'
                    for t in (0.25, 0.30, 0.35, 0.45))
    print(f'{indent}中间60% 落入各档占比 -> {row}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--from', dest='frm', nargs=2, type=float, required=True)
    ap.add_argument('--to', dest='to', nargs=2, type=float, required=True)
    ap.add_argument('--dry', action='store_true', help='只算直线对照，不调规划器')
    ap.add_argument('--topic', default='compute_path_to_pose')
    ap.add_argument('--timeout', type=float, default=25.0, help='等结果超时 (s)')
    args = ap.parse_args()

    m = Map()

    s_len, s_cl = straight_control(
        m, args.frm[0], args.frm[1], args.to[0], args.to[1])
    print('[直线对照]')
    report('', s_len, s_cl)
    print(f'  直线落入内切区(<{INSCRIBED} m)的采样点: '
          f'{int((s_cl < INSCRIBED).sum())}/{len(s_cl)}')

    if args.dry:
        return 0

    rclpy.init()
    n = rclpy.create_node('probe_path_clearance')
    ac = ActionClient(n, ComputePathToPose, args.topic)
    if not ac.wait_for_server(timeout_sec=10.0):
        print(f'✗ 找不到 {args.topic} 动作服务 —— Nav2 起了吗？')
        return 1

    goal = ComputePathToPose.Goal()
    for attr, xy in (('start', args.frm), ('goal', args.to)):
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.pose.position.x = float(xy[0])
        ps.pose.position.y = float(xy[1])
        ps.pose.orientation.w = 1.0
        setattr(goal, attr, ps)
    goal.use_start = True

    fut = ac.send_goal_async(goal)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=10.0)
    handle = fut.result()
    if handle is None or not handle.accepted:
        print('✗ 目标被拒绝 —— planner_server 很可能不是 active（见 问题记录 E-18）')
        n.destroy_node()
        rclpy.shutdown()
        return 1

    rf = handle.get_result_async()
    rclpy.spin_until_future_complete(n, rf, timeout_sec=args.timeout)
    res = rf.result()
    if res is None:
        print('✗ 没拿到结果（多半是规划器抛了异常，比如直线穿过致命障碍）')
        n.destroy_node()
        rclpy.shutdown()
        return 1
    path = res.result.path
    if not path.poses:
        print('✗ 空路径')
        n.destroy_node()
        rclpy.shutdown()
        return 1

    xs = np.array([p.pose.position.x for p in path.poses])
    ys = np.array([p.pose.position.y for p in path.poses])
    p_len = float(np.sum(np.hypot(np.diff(xs), np.diff(ys))))
    p_cl = np.array([m.clearance(float(x), float(y)) for x, y in zip(xs, ys)])

    print()
    print(f'[规划器返回的路径]  {len(path.poses)} 个点')
    print(f'  首/末点 : ({xs[0]:+.2f},{ys[0]:+.2f}) -> '
          f'({xs[-1]:+.2f},{ys[-1]:+.2f})')
    report('', p_len, p_cl)

    print()
    print(f'  对比：规划路径 {p_len:.3f} m vs 直线 {s_len:.3f} m'
          f'（{100.0 * (p_len / s_len - 1.0):+.1f}%）')
    print(f'        中间 60% 净空均值 {p_cl.mean():.3f} m vs 直线 {s_cl.mean():.3f} m')

    n.destroy_node()
    rclpy.shutdown()
    return 0


sys.exit(main())

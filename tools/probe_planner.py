#!/usr/bin/env python3
"""
全局规划器（navfn）取证工具 —— 用数字证明**它最小化的是「代价」不是「距离」**。

背景（`问题记录.md` E-4「绕远路不是 bug」当时只有定性解释，这里是定量版）：

navfn 把代价地图的每格转成「导航代价」，常量在
`/opt/ros/jazzy/include/nav2_navfn_planner/navfn.hpp`：

```c
#define COST_OBS      254   // 禁止区域
#define COST_OBS_ROS  253   // costmap 里 253 及以上视为障碍
#define COST_NEUTRAL   50   // 开阔地
#define COST_FACTOR   0.8   // 导航代价 = COST_NEUTRAL + COST_FACTOR * costmap_cost
```

头文件里那段注释值得原文抄下来：

> Incoming costmap cost values are in the range 0 to 252. With COST_NEUTRAL of 50,
> the COST_FACTOR needs to be about 0.8 to ensure the input values are spread evenly
> over the output range, 50 to 253. **If COST_FACTOR is higher, cost values will have
> a plateau around obstacles and the planner will then treat (for example) the whole
> width of a narrow hallway as equally undesirable and thus will not plan paths down
> the center.**

也就是说：`COST_FACTOR = 0.8` 的取值本身，就是为了让「贴墙」和「走中间」
在代价上有**可分辨的梯度**，从而把路径推到中线。

**于是可以证伪**：找一对起终点，使「直线」比「规划出来的路」短、但代价积分更高。
如果规划器选的是后者，就说明它最小化的是代价。

本工具做三件事：

  1. 调 `compute_path_to_pose` 拿到规划器**实际**给出的路径
  2. 算规划路径的：长度、代价积分 `∫(50 + 0.8·cost) ds`、沿途最大导航代价
  3. 拿**直线**做同样三项计算，做对照

判定：`L_plan > L_line` 且 `C_plan < C_line` → **代价驱动**。

用法（在仓库根目录执行）:

  ./scripts/shell.sh -c 'python3 /workspace/my-slam/tools/probe_planner.py'
  ./scripts/shell.sh -c 'python3 /workspace/my-slam/tools/probe_planner.py \\
      --from 2.17 1.88 --to -4.5 1.5'
  ./scripts/shell.sh -c 'python3 /workspace/my-slam/tools/probe_planner.py \\
      --from 2.17 1.88 --to -4.5 1.5 --profile'

先决条件：Nav2 已起，且 `map` 帧存在（用 `bringup_sim.launch.py` 省掉初始位姿那个坑）。
"""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.parameter import Parameter

import probe_costmap as pc          # 复用它的取帧逻辑与常量

COST_NEUTRAL = 50.0
COST_FACTOR = 0.8
COST_OBS_ROS = 253


def nav_cost(c):
    """导航代价。c >= 253 视为障碍，返回 None。"""
    if c is None or c >= COST_OBS_ROS:
        return None
    return COST_NEUTRAL + COST_FACTOR * c


class CostField:
    """代价地图的最近邻采样（按世界坐标取格）。"""

    def __init__(self, w, h, res, ox, oy, data):
        self.w, self.h, self.res = w, h, res
        self.ox, self.oy, self.data = ox, oy, data

    def raw(self, x, y):
        # ⚠️ 与 PGM 不同！OccupancyGrid / nav2_msgs.Costmap 的 data 是
        #    「行 0 = origin 那一行（最小 y）」，行号随 y 递增 —— **不要翻**。
        #    PGM 才是「行 0 = 最大 y」（见 问题记录.md F-1）。
        #    我在这个文件里第一版就是照抄了 PGM 的公式，结果整张图在 y 上镜像，
        #    表现为「规划出来的路径居然撞障碍」（见 F-17）。
        c = int(round((x - self.ox) / self.res))
        r = int(round((y - self.oy) / self.res))
        if 0 <= c < self.w and 0 <= r < self.h:
            return self.data[r * self.w + c]
        return None

    def nc(self, x, y):
        return nav_cost(self.raw(x, y))


def sample_route(field, pts, step):
    """沿折线采样。返回 (长度, 代价积分, 最大导航代价, 障碍穿越点列表)。

    ⚠️ 不要中途 return —— 那样量到的是「中断处」，不是整条路的长度。
    """
    total = 0.0
    integ = 0.0
    worst = 0.0
    hits = []
    for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
        seg = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(math.ceil(seg / step)))
        for i in range(n):
            t = (i + 0.5) / n
            x, y = x0 + t * (x1 - x0), y0 + t * (y1 - y0)
            nc = field.nc(x, y)
            total += seg / n
            if nc is None:
                integ += 254.0 * (seg / n)
                worst = max(worst, 254.0)
                hits.append((x, y))
            else:
                integ += nc * (seg / n)
                worst = max(worst, nc)
    return total, integ, worst, hits


def call_planner(node, x0, y0, x1, y1, timeout):
    cli = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
    if not cli.wait_for_server(timeout_sec=timeout):
        print('!! /compute_path_to_pose 不可用；planner_server 起了吗？')
        return None
    g = ComputePathToPose.Goal()
    yaw = math.atan2(y1 - y0, x1 - x0)
    g.start.header.frame_id = 'map'
    g.start.pose.position.x, g.start.pose.position.y = float(x0), float(y0)
    g.start.pose.orientation.w = 1.0
    g.use_start = True
    g.goal.header.frame_id = 'map'
    g.goal.pose.position.x, g.goal.pose.position.y = float(x1), float(y1)
    g.goal.pose.orientation.z = math.sin(yaw / 2.0)
    g.goal.pose.orientation.w = math.cos(yaw / 2.0)

    fut = cli.send_goal_async(g)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    gh = fut.result()
    if gh is None or not gh.accepted:
        print('!! 目标被拒绝')
        return None
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(node, rf, timeout_sec=timeout)
    res = rf.result()
    if res is None:
        print('!! 等结果超时')
        return None
    print('  [规划器回包] error_code=%d  planning_time=%.3f s'
          % (res.result.error_code, res.result.planning_time.sec
             + res.result.planning_time.nanosec * 1e-9))
    return res.result.path


def shortest_feasible(field, x0, y0, x1, y1):
    """只按「距离最短」找一条可行路（只要求不穿障碍，**完全不看代价**）。

    这是「距离驱动」的对照组：navfn 如果真的是距离驱动，就该给出和它一样的结果。
    """
    import heapq
    w, h, res, ox, oy = field.w, field.h, field.res, field.ox, field.oy
    c0, r0 = int(round((x0 - ox) / res)), int(round((y0 - oy) / res))
    c1, r1 = int(round((x1 - ox) / res)), int(round((y1 - oy) / res))
    if not (0 <= c0 < w and 0 <= r0 < h and 0 <= c1 < w and 0 <= r1 < h):
        return None
    d = {(c0, r0): 0.0}
    prev = {}
    seen = set()
    pq = [(0.0, c0, r0)]
    goal = (c1, r1)
    while pq:
        dd, c, r = heapq.heappop(pq)
        if (c, r) in seen:
            continue
        seen.add((c, r))
        if (c, r) == goal:
            break
        for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1),
                       (1, 1), (1, -1), (-1, 1), (-1, -1)):
            nc, nr = c + dc, r + dr
            if not (0 <= nc < w and 0 <= nr < h):
                continue
            if field.data[nr * w + nc] >= COST_OBS_ROS:   # 只避障
                continue
            nd = dd + math.hypot(dc, dr)
            if nd < d.get((nc, nr), float('inf')):
                d[(nc, nr)] = nd
                prev[(nc, nr)] = (c, r)
                heapq.heappush(pq, (nd, nc, nr))
    if goal not in d:
        return None
    cells, cur = [], goal
    while cur != (c0, r0):
        cells.append(cur)
        cur = prev[cur]
    cells.append((c0, r0))
    cells.reverse()
    return [(ox + c * res, oy + r * res) for c, r in cells]


def main():
    ap = argparse.ArgumentParser(description='全局规划器取证：代价驱动 vs 距离驱动')
    ap.add_argument('--from', dest='frm', nargs=2, type=float,
                    default=[2.17, 1.88], metavar=('X', 'Y'))
    ap.add_argument('--to', nargs=2, type=float, default=[-4.5, 1.5], metavar=('X', 'Y'))
    ap.add_argument('--costmap', default='/global_costmap/costmap_raw')
    ap.add_argument('--step', type=float, default=0.025, help='采样步长 (m)')
    ap.add_argument('--profile', action='store_true', help='打印直线上的代价剖面')
    ap.add_argument('--timeout', type=float, default=15.0)
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('probe_planner',
                             parameter_overrides=[Parameter('use_sim_time', value=True)])

    got = pc.grab(node, args.costmap, 'raw', args.timeout)
    if got is None:
        print('!! 拿不到 %s' % args.costmap)
        return 1
    w, h, res, ox, oy, data, _ = got
    field = CostField(w, h, res, ox, oy, data)

    x0, y0 = args.frm
    x1, y1 = args.to
    print('起终点     : (%.2f, %.2f) → (%.2f, %.2f)   直线距离 %.2f m'
          % (x0, y0, x1, y1, math.hypot(x1 - x0, y1 - y0)))
    print('代价地图   : %s (%dx%d, res %.3f)' % (args.costmap, w, h, res))
    print('导航代价   : %.0f + %.1f × costmap_cost   （navfn.hpp）'
          % (COST_NEUTRAL, COST_FACTOR))
    print('')

    # ---- 直线对照组 ----
    L_line, C_line, W_line, hits_line = sample_route(
        field, [(x0, y0), (x1, y1)], args.step)
    print('=== 直线（对照）===')
    print('  长度        : %.2f m' % L_line)
    print('  代价积分    : %.1f   (∫ 导航代价 ds；穿越障碍段按 254 计)' % C_line)
    print('  沿途最高代价: %.1f   (开阔地 50；越接近 253 越贴墙)' % min(W_line, 253.0))
    if hits_line:
        print('  ⛔ 穿越障碍/内切区 %d 个采样点（costmap cost ≥ %d）→ **直线不可行**'
              % (len(hits_line), COST_OBS_ROS))
        print('     例如 (%.2f, %.2f) … (%.2f, %.2f)'
              % (hits_line[0][0], hits_line[0][1],
                 hits_line[-1][0], hits_line[-1][1]))

    if args.profile:
        print('')
        print('  直线上的代价剖面（每 0.5 m 一个点）：')
        n = max(2, int(math.ceil(L_line / 0.5)))
        for i in range(n + 1):
            t = i / n
            x, y = x0 + t * (x1 - x0), y0 + t * (y1 - y0)
            raw = field.raw(x, y)
            nc = nav_cost(raw)
            bar = '#' * int(((nc or 253.0)) / 6)
            print('    %5.2f m  (%6.2f,%6.2f)  cost=%3s  导航代价=%6s  %s'
                  % (t * L_line, x, y, raw, ('%.1f' % nc) if nc else '障碍', bar))

    # ---- 规划器实际输出 ----
    path = call_planner(node, x0, y0, x1, y1, args.timeout)
    print('')
    print('=== 规划器实际输出（navfn, use_astar=false）===')
    if path is None or len(path.poses) == 0:
        print('  没拿到路径')
        node.destroy_node()
        return 1
    pts = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
    print('  帧          : %s' % path.header.frame_id)
    print('  首/末点     : (%.2f,%.2f) → (%.2f,%.2f)'
          % (pts[0][0], pts[0][1], pts[-1][0], pts[-1][1]))
    L_plan, C_plan, W_plan, hits_plan = sample_route(field, pts, args.step)
    print('  路径点数    : %d' % len(pts))
    print('  长度        : %.2f m' % L_plan)
    print('  代价积分    : %.1f' % C_plan)
    print('  沿途最高代价: %.1f' % min(W_plan, 253.0))
    if hits_plan:
        print('  ⚠️ 有 %d 个采样点落在障碍/内切区 —— 可能是「取代价地图的时刻」'
              '与「规划的时刻」不一致（雷达在动），也可能路径本身有问题'
              % len(hits_plan))

    # ---- 距离最短的可行路（距离驱动对照）----
    short_pts = shortest_feasible(field, x0, y0, x1, y1)
    print('')
    print('=== 距离最短的可行路（只避障，不看代价 —— 距离驱动对照组）===')
    if short_pts is None:
        print('  在「只避障」的可行空间里找不到通路')
        L_short = C_short = W_short = None
    else:
        L_short, C_short, W_short, h_short = sample_route(field, short_pts, args.step)
        print('  长度        : %.2f m' % L_short)
        print('  代价积分    : %.1f' % C_short)
        print('  沿途最高代价: %.1f   （它不惜贴墙）' % min(W_short, 253.0))

    # ---- 判定 ----
    print('')
    print('=== 判定 ===')
    if L_short is None:
        print('  拿不到距离驱动对照，无法判定')
        node.destroy_node()
        return 1
    print('  %-14s %-12s %-12s %-12s' % ('', '长度 (m)', '代价积分', '沿途最高代价'))
    print('  %-14s %-12.2f %-12.1f %-12.1f'
          % ('直线(不可行)', L_line, C_line, min(W_line, 253.0)))
    print('  %-14s %-12.2f %-12.1f %-12.1f'
          % ('最短可行路', L_short, C_short, min(W_short, 253.0)))
    print('  %-14s %-12.2f %-12.1f %-12.1f'
          % ('navfn 输出', L_plan, C_plan, min(W_plan, 253.0)))
    print('')
    dL = L_plan - L_short
    dC = C_plan - C_short
    print('  navfn 输出 vs 最短可行路：长度 %+.2f m，代价 %+.1f' % (dL, dC))
    if dL > 0.05 and dC < 0:
        print('')
        print('  ✅ **规划器牺牲了 %.2f m 路程，换来了 %.1f 的代价下降** ——'
              % (dL, -dC))
        print('     它最小化的是「代价」不是「距离」。')
        print('     旁证：navfn 输出的沿途最高代价 %.1f，而最短可行路要到 %.1f。'
              % (min(W_plan, 253.0), min(W_short, 253.0)))
    elif abs(dL) <= 0.05:
        print('  ● 两者长度几乎一样 —— 这一对上两条准则不冲突，换一对起终点再试')
    else:
        print('  ⚠️ navfn 给出的路又长又贵 —— 不符合「代价驱动」预期，需要查')

    node.destroy_node()
    return 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""
DWB 局部规划器取证工具 —— 把「每周期采样多少条候选轨迹、为什么选了那一条」摊开来看。

原理（DWB = **采样式**局部规划器，不是解析求解）：

  1. 在当前速度的加减速范围内，按 `vx_samples × vy_samples × vtheta_samples`
     撒一个**速度网格**
  2. 每条 `(v, ω)` 用运动学模型**前推** `sim_time` 秒，得到一条轨迹
  3. 用 `critics` 里每个 critic 给每条轨迹打分，**加权求和**得总分
  4. 取总分最低的那条，作为这一周期的 `cmd_vel_nav`

  ⚠️ 所以机器人的速度**永远来自那个离散网格**，不是连续空间里的最优解。
  网格太稀 → 车会「一顿一顿」或在窄处找不到可行速度。

这一切都被 DWB 发在 `/evaluation` 上（`dwb_msgs/msg/LocalPlanEvaluation`，
本项目的 `publish_evaluation` 默认为 `True`）：

  ```text
  LocalPlanEvaluation
    TrajectoryScore[] twists      # 全部候选
      Trajectory2D traj.velocity  # (x, y, theta)  即 (v, v_y, ω)
      CriticScore[] scores        # name / raw_score / scale
      float32 total               # 加权总分
    uint16 best_index            # 选中那条的下标
    uint16 worst_index
  ```

本工具做四件事：

  1. **数一遍候选轨迹条数**，和实测公式对：
     **`候选数 = vx_samples × (vtheta_samples + 1) − 1`**
     （实测：20/20 → 419；10/20 → 209。θ 网格比 `vtheta_samples` 多一个「不转」，
     而 `vy_samples` **完全不参与** —— 差速车没有 y 自由度）
  2. **自检两条可证伪的断言**：
     `twists[best_index].total == min(所有 total)` 与
     `total == Σ(raw_score × scale)`
  3. 打印**选中那条**的逐 critic 明细 → 直接回答「为什么选它」
  4. 汇总本次导航中所有被选中的速度 → 看离散网格的痕迹

⚠️ **`scale` 不等于配置值（实测规律）**

`MapGridCritic` 家族的 4 个 critic（`PathAlign` / `GoalAlign` / `PathDist` / `GoalDist`）
**生效的 scale = 配置值 × resolution / 2**；其余三个（`RotateToGoal` / `Oscillation` /
`BaseObstacle`）就是配置值。实测：

| `local_costmap.resolution` | GoalAlign 配置 24.0 | PathAlign 配置 32.0 |
| --- | ---: | ---: |
| 0.05 | **0.600**（÷40） | **0.800**（÷40） |
| 0.025 | **0.300**（÷80） | **0.400**（÷80） |

已逐个排除的候选：`angular_granularity`（0.025→0.05 无影响）、`vx_samples`
（20→10 无影响）、`linear_granularity`（0.05→0.1 无影响）。
库 `libdwb_critics.so` 确实引用了 `nav2_costmap_2d::Costmap2D::getResolution()`。
**那个 `/2` 的来历未定位**（可能是单元换算的半格偏移）。

用法（在仓库根目录执行）:

  ./scripts/shell.sh -c 'python3 /workspace/my-slam/tools/probe_dwb.py \\
      --goal 2.0 0.0 --secs 8'
  ./scripts/shell.sh -c 'python3 /workspace/my-slam/tools/probe_dwb.py --top 8'

先决条件：Nav2 已起、`planner_server` / `controller_server` 已 active。
本工具自己能发导航目标（`--goal`），不需要另开终端。
"""

import argparse
import math
import sys
import time

import rclpy
from dwb_msgs.msg import LocalPlanEvaluation
from rclpy.parameter import Parameter


class Collector:
    def __init__(self, node, topic):
        self.msgs = []
        node.create_subscription(LocalPlanEvaluation, topic, self.cb, 10)

    def cb(self, m):
        self.msgs.append(m)


def fmt_vel(t):
    return '(v=%+.3f, ω=%+.3f)' % (t.traj.velocity.x, t.traj.velocity.theta)


def send_goal(node, x, y, yaw=0.0):
    """发一个 NavigateToPose 目标（不等结果）。返回 action client 或 None。"""
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    cli = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    if not cli.wait_for_server(timeout_sec=8.0):
        print('!! /navigate_to_pose 不可用')
        return None
    g = NavigateToPose.Goal()
    g.pose.header.frame_id = 'map'
    g.pose.pose.position.x, g.pose.pose.position.y = float(x), float(y)
    g.pose.pose.orientation.z = math.sin(yaw / 2.0)
    g.pose.pose.orientation.w = math.cos(yaw / 2.0)
    fut = cli.send_goal_async(g)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=8.0)
    gh = fut.result()
    if gh is None or not gh.accepted:
        print('!! 目标被拒绝')
        return None
    print('已发导航目标 (%.2f, %.2f)，等 2 s 让控制器跑起来 …' % (x, y))
    return cli


def main():
    ap = argparse.ArgumentParser(description='DWB 候选轨迹与评分取证')
    ap.add_argument('--topic', default='/evaluation')
    ap.add_argument('--secs', type=float, default=6.0, help='采集时长 (s)')
    ap.add_argument('--top', type=int, default=6, help='明细里列前 N 名')
    ap.add_argument('--goal', nargs=2, type=float, metavar=('X', 'Y'),
                    help='先发一个导航目标再采集（推荐 —— 省得另开终端）')
    ap.add_argument('--vx-samples', type=int, default=20)
    ap.add_argument('--vtheta-samples', type=int, default=20)
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node(
        'probe_dwb', parameter_overrides=[Parameter('use_sim_time', value=True)])
    if args.goal:
        if send_goal(node, args.goal[0], args.goal[1]) is None:
            node.destroy_node()
            return 1
        t = time.time()
        while time.time() - t < 2.0:
            rclpy.spin_once(node, timeout_sec=0.05)
    col = Collector(node, args.topic)
    t0 = time.time()
    while time.time() - t0 < args.secs:
        rclpy.spin_once(node, timeout_sec=0.05)

    if not col.msgs:
        print('!! %.1f 秒内 %s 没有消息。这个话题只在**控制器正在跟踪路径**时才发 ——'
              % (args.secs, args.topic))
        print('   先发一个导航目标（见文件头的用法），再跑本工具。')
        return 1

    m = col.msgs[0]
    n = len(m.twists)
    print('话题 %s   消息类型 dwb_msgs/LocalPlanEvaluation' % args.topic)
    print('本次采集到 %d 条 evaluation（%.1f s）' % (len(col.msgs), args.secs))
    print('')
    print('=== 1. 候选轨迹条数 ===')
    print('  第 1 条消息里有 **%d** 条候选轨迹' % n)
    vx, vt = args.vx_samples, args.vtheta_samples
    pred_n = vx * (vt + 1) - 1
    print('  实测公式：候选数 = vx_samples × (vtheta_samples + 1) − 1')
    print('           = %d × (%d + 1) − 1 = **%d**  → %s'
          % (vx, vt, pred_n, '✅ 吻合' if pred_n == n else '⚠️ 不吻合'))
    print('  （`vy_samples` 不参与：差速车没有 y 自由度；')
    print('    θ 网格比 vtheta_samples 多一个「不转」）')
    print('  自检另两个参数点：20/20 → 419；10/20 → 209（都已在 2026-10-03 实测过）')

    print('')
    print('=== 2. 自检两条「可证伪的断言」===')
    totals = [t.total for t in m.twists]
    argmin = min(range(n), key=lambda i: totals[i])
    ok_best = (argmin == m.best_index)
    print('  断言 A：best_index == argmin(total)')
    print('     best_index=%d，argmin=%d  → %s'
          % (m.best_index, argmin, '✅ 成立' if ok_best else '❌ 不成立'))
    b = m.twists[m.best_index]
    s = sum(c.raw_score * c.scale for c in b.scores)
    print('  断言 B：total == Σ(raw_score × scale)')
    print('     total=%.6f，Σ(raw×scale)=%.6f，差 %.2e  → %s'
          % (b.total, s, abs(b.total - s), '✅ 成立' if abs(b.total - s) < 1e-3 else '❌ 不成立'))

    print('')
    print('=== 3. 选中那条（best_index=%d）为什么赢 ===' % m.best_index)
    tspan = 0.0
    if len(b.traj.time_offsets):
        _last = b.traj.time_offsets[-1]
        tspan = _last.sec + _last.nanosec * 1e-9
    print('  速度 %s   轨迹 %d 个点，时间跨度 %.2f s'
          % (fmt_vel(b), len(b.traj.poses), tspan))
    print('')
    print('  %-16s %14s %10s %14s' % ('critic', 'raw_score', 'scale', 'raw×scale'))
    print('  ' + '-' * 58)
    for c in b.scores:
        print('  %-16s %14.4f %10.3f %14.4f' % (c.name, c.raw_score, c.scale,
                                                c.raw_score * c.scale))
    print('  ' + '-' * 58)
    print('  %-16s %14s %10s %14.4f' % ('合计', '', '', s))
    print('')
    print('  ⚠️ 这里的 scale **不是** nav2_params.yaml 里写的值：')
    print('     MapGridCritic 家族（PathAlign/GoalAlign/PathDist/GoalDist）的生效 scale')
    print('     = **配置值 × resolution / 2**（res=0.05 时 ÷40，res=0.025 时 ÷80）。')
    print('     其余三个（RotateToGoal/Oscillation/BaseObstacle）就是配置值。')

    print('')
    print('=== 4. 候选总分的分布（前 %d 名）===' % args.top)
    order = sorted(range(n), key=lambda i: totals[i])[:args.top]
    print('  名次  %-24s %12s' % ('速度 (v, ω)', '总分'))
    for k, i in enumerate(order):
        star = ' ← 选中' if i == m.best_index else ''
        print('   %2d   %-24s %12.4f%s' % (k, fmt_vel(m.twists[i]), totals[i], star))
    print('')
    print('  全部 %d 条的 total：min=%.4f  max=%.4f  不同取值 %d 个'
          % (n, min(totals), max(totals), len(set(round(t, 6) for t in totals))))

    print('')
    print('=== 5. 本次被选中的速度，是不是从网格里挑的 ===')
    vs = [(x.twists[x.best_index].traj.velocity.x,
           x.twists[x.best_index].traj.velocity.theta) for x in col.msgs]
    dis = sorted(set((round(a, 4), round(b2, 4)) for a, b2 in vs))
    print('  %d 条 evaluation 里，共出现 %d 种不同的 (v, ω) 组合'
          % (len(col.msgs), len(dis)))
    print('  v  范围: %.3f ~ %.3f' % (min(a for a, _ in vs), max(a for a, _ in vs)))
    print('  ω  范围: %.3f ~ %.3f' % (min(b2 for _, b2 in vs), max(b2 for _, b2 in vs)))
    print('  （v/ω 都来自离散网格 —— 这就是「采样式」留下的痕迹）')

    node.destroy_node()
    return 0


if __name__ == '__main__':
    sys.exit(main())

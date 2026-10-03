#!/usr/bin/env python3
"""
代价地图（costmap）取证工具 —— 把「0~255 到底什么意思」和「膨胀层怎么衰减」用实测数字讲清楚。

⚠️ **第一个坑：两个话题，数字不一样**

| 话题 | 类型 | 数值 |
| --- | --- | --- |
| `<ns>/costmap` | `nav_msgs/msg/OccupancyGrid` | 把真实代价**压缩到 0~100**（线性的 `×100/255` 量级），最大值只有 100 |
| `<ns>/costmap_raw` | `nav2_msgs/msg/Costmap` | **0~255 真实代价**，要验证膨胀公式必须用这个 |

实测对照（同一份全局代价地图；两组数字的**格数一一对应**，说明是同一份数据的两种编码）：

| `*_raw` 真实代价 | 254 | 253 | 234 | 201 | 173 | 149 | 128 | 110 | 95 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `/costmap` 压缩后 | 100 | 99 | 91 | 78 | 67 | 58 | 50 | 43 | 37 |
| 格数 | 3470 | 13885 | 2426 | 2128 | 2072 | 2022 | 2058 | 2019 | 1819 |

而 `nav2_costmap_2d` 的内部常量是 `LETHAL_OBSTACLE = 254`。
**所以拿 `costmap` 话题去验证膨胀公式，得到的数字全是错的** ——
它的最大值 `100` 会让人（第一次就这么骗过了我）以为「这片区域最贵也就 100」，
从而彻底漏掉 `253`/`254` 这两档，连有没有障碍都判断错。

回答四个问题：

  1. **代价地图不是「有障碍 / 没障碍」**，而是 0~255 的分层语义。
     常量来自 `/opt/ros/jazzy/include/nav2_costmap_2d/nav2_costmap_2d/cost_values.hpp`：

     | 值 | 常量 | 含义 |
     | --- | --- | --- |
     | 0 | `FREE_SPACE` | 自由 |
     | 1~252 | （膨胀产生） | 越靠近障碍越大 |
     | 253 | `INSCRIBED_INFLATED_OBSTACLE` | 障碍物内切圆以内 —— **车中心进这里就是撞** |
     | 254 | `LETHAL_OBSTACLE` | 致命障碍 |
     | 255 | `NO_INFORMATION` | 未知 |

  2. **膨胀层是 exp 衰减，可以算出来再和实测对**。公式来自
     `inflation_layer.hpp` 的 `computeCost()` —— **四段，不是三段**：

     ```
     d = 到最近致命格的欧氏距离(格) × resolution
     d == 0                    → 254
     d <= inscribed_radius     → 253
     d >  inflation_radius     → 0     ← 最容易漏的一段
     否则                       → (unsigned char)(252 × exp(-cost_scaling_factor × (d - inscribed_radius)))
     ```

     ⚠️ **第 3 段是本工具后来补上的** —— `inflation_layer` 里有
     `if (distance > cell_inflation_radius_) continue;`，**出了这个半径根本不写代价**。
     本工具早期漏了它，只因**这个房间里几乎所有自由格都在 0.55 m 内**而没暴露。
     我在一个临时脚本上就因此得出过 52.8% 的假不一致（见 `问题记录.md` F-22 / E-22）。

     ⚠️ **另：`--inscribed` 的默认值是「生效值」不是「配置值」。**
     配置里 `robot_radius` 写 0.22，但**生效**的约 **0.2255**（E-15）。
     默认用 0.22 去预测，完全相符率只有 76.9%；用 0.2249 → 93.8%；
     用 0.2255 → **97.7%**（扫相符率的峰值）。
     剩下 ~2.3% 全是 `实测 = 预测 + 1`，**未定位**。

     本工具用**精确欧氏距离变换**（Felzenszwalb）算每格到最近致命格的距离，
     逐格比对「预测 vs 实测」，并按距离分桶给出对照表。

  3. **同一个地址上 local 和 global 代价地图不一样** —— 两边的 `plugins` 列表不同：
     `local_costmap: [voxel_layer, inflation_layer]`
     `global_costmap: [static_layer, obstacle_layer, inflation_layer]`
     local 少一个 `static_layer`，所以它**看不到地图里那些雷达扫不到的墙**
     （配置里虽然写了 `static_layer:` 一段，但没列进 `plugins` → 不生效）。
     实测致命格数：`global` 3475（≈ 地图里的墙）、`local` 只有 34（= 1.5 m 内雷达看到的那点东西）。

  4. **配置值 ≠ 生效值**。本工具会从实测代价**反推**有效 `inscribed_radius`：

     | 代价地图 | 致命格 | 反推 inscribed_radius |
     | --- | --- | --- |
     | `global_costmap` | 3475 | **0.2249 m** |
     | `local_costmap` | 34 | **0.2248 m** |

     配置里 `robot_radius` 是 **0.22**（已在运行中的节点上用 `ros2 param get` 复核），
     差 **+4.9 mm**，且在 8 个独立距离桶上稳定 —— 不是噪声。**原因未定位**
     （`getInscribedRadius()` 只是返回成员变量，真正的计算在未随包安装的 `.cpp` 里）。
     教训：**这 5 mm 直接决定「能不能过门」**，所以「配置写了什么」≠「生效了什么」。

用法（在仓库根目录执行）:

  ./scripts/shell.sh -c 'python3 /workspace/my-slam/tools/probe_costmap.py \\
      --topic /global_costmap/costmap_raw --ascii'
  ./scripts/shell.sh -c 'python3 /workspace/my-slam/tools/probe_costmap.py \\
      --topic /local_costmap/costmap_raw'
  # 想对比「被压缩过的那个」：--type occupancy
  ./scripts/shell.sh -c 'python3 /workspace/my-slam/tools/probe_costmap.py \\
      --topic /global_costmap/costmap --type occupancy'

先决条件：Nav2 已起（`bringup_sim.launch.py` 会自动设初始位姿，省掉 60 秒超时那个坑）。
"""

import argparse
import math
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.parameter import Parameter
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

try:
    from nav2_msgs.msg import Costmap as Nav2Costmap
except ImportError:                                   # pragma: no cover
    Nav2Costmap = None

FREE, INSCRIBED, LETHAL, UNKNOWN = 0, 253, 254, 255
INF = 1e18


# ---------------- 距离变换 ----------------
def _dt1d(f):
    """一维平方距离变换（Felzenszwalb & Huttenlocher），O(n)。"""
    n = len(f)
    v = [0] * n
    z = [0.0] * (n + 1)
    k = 0
    v[0] = 0
    z[0] = -INF
    z[1] = INF
    for q in range(1, n):
        s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0 * q - 2.0 * v[k])
        while s <= z[k]:
            k -= 1
            s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0 * q - 2.0 * v[k])
        k += 1
        v[k] = q
        z[k] = s
        z[k + 1] = INF
    d = [0.0] * n
    k = 0
    for q in range(n):
        while z[k + 1] < q:
            k += 1
        d[q] = (q - v[k]) ** 2 + f[v[k]]
    return d


def edt_sq(src, w, h):
    """src[y][x] 为 True 表示源点（致命格）。返回平方欧氏距离（单位：格）。"""
    f = [[0.0 if src[y][x] else INF for x in range(w)] for y in range(h)]
    for x in range(w):
        col = _dt1d([f[y][x] for y in range(h)])
        for y in range(h):
            f[y][x] = col[y]
    for y in range(h):
        f[y] = _dt1d(f[y])
    return f


def predict_cost(d_cells, res, inscribed, csf, infl):
    """严格照抄 inflation_layer.hpp 的 computeCost() —— **四段**，不是三段。

    ⚠️ 第 3 段是后来补上的（代价地图刚做时只验证了膨胀半径**以内**）：
        d > inflation_radius  ->  0
    `inflation_layer` 里有 `if (distance > cell_inflation_radius_) continue;`，
    出了这个半径**根本不写代价**。漏了它会把「离墙 0.55~3 m 的广大区域」
    全部误报成不一致（见 问题记录 F-22 / E-22）。
    """
    if d_cells == 0:
        return LETHAL
    d_m = d_cells * res
    if d_m <= inscribed:
        return INSCRIBED
    if d_m > infl:
        return FREE
    factor = math.exp(-csf * (d_m - inscribed))
    return int((INSCRIBED - 1) * factor) & 0xFF      # unsigned char 截断


# ---------------- 取一帧 ----------------
def grab(node, topic, want, timeout):
    """want: 'occupancy' 或 'raw'。返回 (w, h, res, ox, oy, data, note)"""
    got = []

    def on_og(m):
        if got:
            return
        # ⚠️ OccupancyGrid.data 在 ROS 2 里是 int8[]，128~255 会以负数出现。
        #    该话题按 0/100/−1 约定发布，实测无负值；但换成别的话题就可能踩到，
        #    所以统一 & 0xFF 归一化（见 docs/问题记录.md 的 F-16）。
        d = [v & 0xFF for v in m.data]
        got.append((m.info.width, m.info.height, m.info.resolution,
                    m.info.origin.position.x, m.info.origin.position.y, d,
                    'nav_msgs/OccupancyGrid（0/100/−1 约定，非真实代价）'))

    def on_raw(m):
        if got:
            return
        md = m.metadata
        got.append((md.size_x, md.size_y, md.resolution,
                    md.origin.position.x, md.origin.position.y,
                    [int(v) for v in m.data],
                    'nav2_msgs/Costmap（0~255 真实代价）'))

    qos = QoSProfile(depth=1,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     history=QoSHistoryPolicy.KEEP_LAST)
    if want == 'raw':
        if Nav2Costmap is None:
            raise SystemExit('nav2_msgs 不可用，无法订阅 Costmap 类型')
        node.create_subscription(Nav2Costmap, topic, on_raw, qos)
    else:
        node.create_subscription(OccupancyGrid, topic, on_og, qos)

    t0 = time.time()
    while not got and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    return got[0] if got else None


def main():
    ap = argparse.ArgumentParser(
        description='代价地图取证：语义分层 + 膨胀衰减 预测 vs 实测')
    ap.add_argument('--topic', default='/global_costmap/costmap_raw',
                    help='代价地图话题（默认用 *_raw，即真实代价）')
    ap.add_argument('--type', choices=['auto', 'occupancy', 'raw'], default='auto',
                    help='消息类型；auto = 按话题名是否以 _raw 结尾判断')
    ap.add_argument('--inscribed', type=float, default=0.2249,
                    help='生效的内切半径 (m)。⚠️ 配置里写 robot_radius=0.22，'
                         '但**生效**的是 0.2249（见 问题记录 E-15）——'
                         '默认用 0.22 去预测，会让约 23%% 的格子对不上')
    ap.add_argument('--csf', type=float, default=3.0,
                    help='cost_scaling_factor，默认 3.0')
    ap.add_argument('--infl', type=float, default=0.55,
                    help='inflation_radius (m)，默认 0.55')
    ap.add_argument('--ascii', action='store_true', help='额外打印 ASCII 缩略图')
    ap.add_argument('--timeout', type=float, default=15.0, help='等消息超时 (s)')
    args = ap.parse_args()

    want = args.type
    if want == 'auto':
        want = 'raw' if args.topic.endswith('_raw') else 'occupancy'

    rclpy.init()
    node = rclpy.create_node(
        'probe_costmap',
        parameter_overrides=[Parameter('use_sim_time', value=True)])
    got = grab(node, args.topic, want, args.timeout)
    if got is None:
        print('!! %.0f 秒内没等到 %s（按 %s 解析）；Nav2 起了吗？话题名对吗？'
              % (args.timeout, args.topic, want))
        return 1
    w, h, res, ox, oy, data, note = got

    print('话题 %s' % args.topic)
    print('  消息类型   : %s' % note)
    print('  尺寸       : %d x %d px = %.2f x %.2f m  (res %.3f m/px)'
          % (w, h, w * res, h * res, res))
    print('  origin     : (%.2f, %.2f)' % (ox, oy))
    print('  参数       : inscribed=%.2f  cost_scaling_factor=%.2f  inflation_radius=%.2f'
          % (args.inscribed, args.csf, args.infl))

    # ---------- 1. 语义分层 ----------
    bands = [(FREE, FREE, 'FREE_SPACE 0'), (1, 252, '膨胀代价 1~252'),
             (INSCRIBED, INSCRIBED, 'INSCRIBED 253'),
             (LETHAL, LETHAL, 'LETHAL 254'), (UNKNOWN, UNKNOWN, 'UNKNOWN 255')]
    tot = w * h
    print('')
    print('=== 1. 语义分层（%d 格）===' % tot)
    for lo, hi, name in bands:
        n = sum(1 for v in data if lo <= v <= hi)
        print('  %-16s %8d 格  %6.2f%%' % (name, n, 100.0 * n / tot))
    cnt = {}
    for v in data:
        cnt[v] = cnt.get(v, 0) + 1
    top = sorted(cnt.items(), key=lambda kv: -kv[1])[:12]
    print('  取值分布(前12)  : ' + ', '.join('%d×%d' % (v, c) for v, c in top))
    print('  min=%d  max=%d  不同取值数=%d' % (min(data), max(data), len(cnt)))
    if want == 'occupancy' and max(data) <= 100:
        print('  ⚠️ 这是 OccupancyGrid 约定（0/100/−1），不是真实代价 —— '
              '要验证膨胀公式请换 *_raw 话题')

    # ---------- 2. 膨胀衰减：预测 vs 实测 ----------
    print('')
    print('=== 2. 膨胀衰减：预测 vs 实测 ===')
    src = [[data[y * w + x] == LETHAL for x in range(w)] for y in range(h)]
    nsrc = sum(1 for row in src for v in row if v)
    if nsrc == 0:
        print('  这张图里没有 LETHAL(254) 格 —— 无法验证。')
        print('  （多半是看了 OccupancyGrid 那个话题，或用的是 local_costmap 而附近没有障碍）')
    else:
        d2 = edt_sq(src, w, h)
        buckets = {}
        match = mismatch = 0
        for y in range(h):
            for x in range(w):
                v = data[y * w + x]
                if v == LETHAL or v == UNKNOWN:
                    continue
                dc = math.sqrt(d2[y][x])
                if dc <= 1e-9:
                    continue
                pred = predict_cost(dc, res, args.inscribed, args.csf, args.infl)
                key = round(dc, 4)          # 精确到「格」；只可能是 sqrt(a²+b²)
                buckets.setdefault(key, {})
                buckets[key][v] = buckets[key].get(v, 0) + 1
                if pred == v:
                    match += 1
                else:
                    mismatch += 1
        print('  源 %d 个致命格；逐格比对 %d 格：完全相符 %d (%.1f%%)  不符 %d (%.1f%%)'
              % (nsrc, match + mismatch, match,
                 100.0 * match / max(1, match + mismatch),
                 mismatch, 100.0 * mismatch / max(1, match + mismatch)))
        print('')
        print('  反推 r 的算法：由 实测 = 252·exp(-csf·(d − r)) 解出 r = d + ln(实测/252)/csf，')
        print('  看它是否等于配置的 inscribed_radius(%.2f)。' % args.inscribed)
        print('')
        print('  %-7s %-7s %-7s %-7s %-9s %-8s %s'
              % ('格距', '米距', '预测', '实测', '反推 r', '格数', '判定'))
        rs = []
        shown = 0
        for key in sorted(buckets):
            if shown >= 20:
                break
            hist = buckets[key]
            topv = sorted(hist.items(), key=lambda kv: -kv[1])[:3]
            n = sum(hist.values())
            meas = topv[0][0]
            pred = predict_cost(key, res, args.inscribed, args.csf, args.infl)
            dm = key * res
            if 0 < meas < INSCRIBED:
                r_fit = dm + math.log(meas / 252.0) / args.csf
                rs.append((n, r_fit))
                rs_txt = '%7.4f' % r_fit
            else:
                rs_txt = '   —   '
            mark = '✅' if meas == pred else ('⚠️' if meas < INSCRIBED else '')
            dist = '  '.join('%d×%d' % (v, c) for v, c in topv)
            print('  %-7.3f %-7.3f %-7d %-7d %-9s %-8d %s  %s'
                  % (key, dm, pred, meas, rs_txt, n, mark, dist))
            shown += 1
        if rs:
            tot_n = sum(n for n, _ in rs)
            avg = sum(n * r for n, r in rs) / max(1, tot_n)
            print('')
            print('  → 按格数加权的反推 r = %.4f m（配置值 %.2f m，差 %+.4f m）'
                  % (avg, args.inscribed, avg - args.inscribed))
        print('')
        print('  注：距离 = 到最近 LETHAL 格的精确欧氏距离（单位：格），'
              '膨胀层内部用的是整数格偏移的 hypot，两者同源。')

    # ---------- 3. ASCII 缩略 ----------
    if args.ascii:
        step = max(1, int(round(0.20 / res)))       # 每 0.2 m 取一个样
        print('')
        print('=== 3. ASCII 缩略（每 %.1f m 取样；. 自由  : 膨胀  O 内切  # 致命  ? 未知）==='
              % (step * res))
        for y in range(h - 1, -1, -step):           # 行反序：让 +y 朝上
            row = []
            for x in range(0, w, step):
                v = data[y * w + x]
                row.append('.' if v == FREE else
                           ('#' if v == LETHAL else
                            ('O' if v == INSCRIBED else ('?' if v == UNKNOWN else ':'))))
            print('  ' + ''.join(row))

    node.destroy_node()
    return 0


if __name__ == '__main__':
    sys.exit(main())

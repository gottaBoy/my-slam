#!/usr/bin/env python3
"""
静态地图(PGM) 推出的净空  vs  实时 costmap 的实际代价 —— 逐格比对 + **反推参数**。

回答两个问题：

  1. **膨胀公式到底对不对？**
     用 PGM 算出每个点到最近障碍的距离，按公式预测代价，
     再和 `/global_costmap/costmap_raw` 的实测值逐格比。

  2. **改完参数，怎么知道它真的生效了？**
     拿同一份实测去对**多个参数假设**，看哪个吻合。
     最小值会锐利地落在真实值上 —— 比 `ros2 param get`（容易写错参数路径）
     更硬。实测：改 `inflation_radius` 到 1.2 后，各假设的不一致率是
     `0.55→35% / 0.8→21% / 1.0→10% / 1.2→2% / 1.5→12%`。

⚠️ **公式必须完整**，漏一条就会得到大面积的假不一致：

```
clear <= 0                     -> 254   致命
clear <= inscribed             -> 253   内切：硬阻断
clear >  inflation_radius      -> 0     出了影响范围，不写代价  ← 最容易漏
否则                            -> 252 * exp(-csf * (clear - inscribed))
```

我自己第一版就漏了第 3 条，得出 **52.8% 的「不一致」**，全是假的
（见 `问题记录.md` F-22 / E-22）。所以本工具默认会把这条算进去。

和 `tools/probe_costmap.py` 的分工：
  * `probe_costmap.py`     —— 按**距离分档**看语义，讲「0~255 是什么」
  * `probe_map_vs_costmap.py`（本工具）—— 按**格子**做预测 vs 实测，并反推参数

用法（容器内，Nav2 已在跑）:
  python3 tools/probe_map_vs_costmap.py
  python3 tools/probe_map_vs_costmap.py --inscribed 0.2249 --csf 3.0
  python3 tools/probe_map_vs_costmap.py --hypotheses 0.4 0.55 0.8 1.2
  python3 tools/probe_map_vs_costmap.py --topic /local_costmap/costmap_raw
"""

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from nav2_msgs.msg import Costmap  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402

from map_reachability import Map  # noqa: E402

INSCRIBED = 0.2249   # 生效内切半径：(配置 robot_radius 是 0.22，见 E-15；
                     #   扫逐格相符率的峰值其实是 0.2255，差 0.6 mm，未定位)
CSF = 3.0            # cost_scaling_factor
INFL = 0.55          # inflation_radius


def predict(clear_m, inscribed=INSCRIBED, csf=CSF, infl=INFL):
    """由「到最近障碍的净空(米)」预测 costmap 的代价值。"""
    if clear_m <= 0.0:
        return 254
    if clear_m <= inscribed:
        return 253
    if clear_m > infl:
        return 0
    return int(round(252.0 * math.exp(-csf * (clear_m - inscribed))))


class Grab(Node):
    def __init__(self, topic):
        super().__init__('probe_map_vs_costmap')
        q = QoSProfile(depth=1)
        q.durability = DurabilityPolicy.TRANSIENT_LOCAL
        q.reliability = ReliabilityPolicy.RELIABLE
        self.msg = None
        self.create_subscription(Costmap, topic, self.cb, q)

    def cb(self, m):
        self.msg = m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/global_costmap/costmap_raw')
    ap.add_argument('--inscribed', type=float, default=INSCRIBED)
    ap.add_argument('--csf', type=float, default=CSF)
    ap.add_argument('--infl', type=float, default=INFL)
    ap.add_argument('--hypotheses', nargs='+', type=float,
                    default=[0.4, 0.55, 0.7, 0.8, 1.0, 1.2, 1.5],
                    help='要试的 inflation_radius 假设（反推用）')
    ap.add_argument('--grid', type=float, default=0.25, help='采样网格间距 (m)')
    ap.add_argument('--timeout', type=float, default=20.0)
    args = ap.parse_args()

    m = Map()

    rclpy.init()
    n = Grab(args.topic)
    deadline = args.timeout / 0.1
    for _ in range(int(deadline)):
        rclpy.spin_once(n, timeout_sec=0.1)
        if n.msg is not None:
            break
    if n.msg is None:
        print(f'✗ 没收到 {args.topic}（Nav2 起了吗？话题名对吗？）')
        return 1

    cm = n.msg
    res = cm.metadata.resolution
    ox = cm.metadata.origin.position.x
    oy = cm.metadata.origin.position.y
    W, H = cm.metadata.size_x, cm.metadata.size_y
    data = np.array(cm.data, dtype=np.int16).reshape(H, W)

    print(f'costmap   {W}x{H}  res={res:.4f}  origin=({ox:.2f},{oy:.2f})')
    print(f'静态图    {m.w}x{m.h}  res={m.res:.4f}  origin=({m.ox:.2f},{m.oy:.2f})')
    print()

    # ---- 采样：在整个 costmap 范围内打网格 ----
    samples = []
    for y in np.arange(oy + 0.1, oy + H * res - 0.1, args.grid):
        for x in np.arange(ox + 0.1, ox + W * res - 0.1, args.grid):
            r = int(round((y - oy) / res))
            c = int(round((x - ox) / res))
            if not (0 <= r < H and 0 <= c < W):
                continue
            rr, cc = m.rc(float(x), float(y))
            if not (0 <= rr < m.h and 0 <= cc < m.w):
                continue
            samples.append((float(x), float(y),
                            m.clearance(float(x), float(y)),
                            int(data[r, c])))

    if not samples:
        print('✗ 没有可比的采样点')
        return 1

    print(f'=== 用当前参数预测 vs 实测（共 {len(samples)} 点，网格 {args.grid} m）===')
    print(f'    inscribed={args.inscribed}  csf={args.csf}  infl={args.infl}')
    bad = [s for s in samples
           if abs(s[3] - predict(s[2], args.inscribed, args.csf, args.infl)) > 2]
    print(f'    不一致(|Δ|>2): {len(bad)} / {len(samples)}'
          f' = {100.0 * len(bad) / len(samples):.1f}%')
    print(f'    实测非零代价格数: {int((data > 0).sum())} / {data.size}'
          f' = {100.0 * (data > 0).sum() / data.size:.1f}%')

    if bad:
        worst = sorted(bad, key=lambda s: -abs(
            s[3] - predict(s[2], args.inscribed, args.csf, args.infl)))[:6]
        print('    最严重的几处：')
        print(f"    {'x':>7}{'y':>7}{'净空m':>9}{'预测':>7}{'实测':>7}{'Δ':>7}")
        for (x, y, cl, a) in worst:
            p = predict(cl, args.inscribed, args.csf, args.infl)
            print(f'{x:7.2f}{y:7.2f}{cl:9.3f}{p:7d}{a:7d}{a-p:7d}')

    print()
    print('=== 反推 inflation_radius（同一份实测，换假设）===')
    best = None
    for hyp in args.hypotheses:
        k = sum(1 for (_x, _y, cl, a) in samples
                if abs(a - predict(cl, args.inscribed, args.csf, hyp)) > 2)
        pct = 100.0 * k / len(samples)
        mark = ''
        if best is None or k < best[1]:
            best = (hyp, k, pct)
        print(f'    假设 inflation_radius={hyp:<5} → 不一致 {k:5d}/{len(samples)}'
              f' = {pct:5.1f}%')
    if best:
        print(f'    ⇒ 最吻合的是 **{best[0]}**（不一致 {best[2]:.1f}%）')
        print('      注：最小值若不够锐利，可能是 inscribed/csf 也写错了，'
              '或代价地图没更新。')

    print()
    print('提示：如果「当前参数」那一段不一致率很高，先别怀疑 costmap ——')
    print('      检查自己的公式是不是漏了 `clear > inflation_radius -> 0` 这一条。')

    n.destroy_node()
    rclpy.shutdown()
    return 0


sys.exit(main())

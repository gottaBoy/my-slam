#!/usr/bin/env python3
"""
分析栅格地图上一条线段两侧的余量，用于回答：
「直线明明没墙，为什么 Nav2 规划出 2~3 倍的绕行？」

原理：navfn 默认 use_astar=false，是 **按代价最小** 找路（不是按距离最短）。
膨胀层（inflation_radius）会给墙附近一大片格子加代价，代价按
exp(-cost_scaling_factor * d) 衰减。如果直线全程都贴着墙（离墙很近），
它的累计代价可能远高于一条更长但更空旷的路，规划器就会绕。

用法:
  python3 tools/map_clearance.py --from 2.17 1.88 --to -4.5 1.5
  python3 tools/map_clearance.py --from 2.17 1.88 --to -4.5 1.5 --samples 25 --inflation 0.55
"""

import argparse
import os
import sys

import numpy as np

def _find_in_src(rel_path):
    """在 src/ 下定位文件，允许一级分组目录（src/<组>/<包>/...）。"""
    root = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src')
    if os.path.exists(os.path.join(root, rel_path)):
        return os.path.join(root, rel_path)
    for group in sorted(os.listdir(root)):
        cand = os.path.join(root, group, rel_path)
        if os.path.exists(cand):
            return cand
    return os.path.join(root, rel_path)


DEFAULT_MAP = _find_in_src('mybot_navigation2/maps/room.pgm')
DEFAULT_YAML = _find_in_src('mybot_navigation2/maps/room.yaml')


def read_yaml(path):
    res, origin = None, None
    with open(path) as f:
        for line in f:
            s = line.strip()
            if s.startswith('resolution:'):
                res = float(s.split(':')[1])
            elif s.startswith('origin:'):
                origin = [float(v) for v in
                          s.split(':', 1)[1].strip().strip('[]').split(',')]
    return res, origin


def read_pgm(path):
    raw = open(path, 'rb').read()
    tokens, i = [], 2
    while len(tokens) < 3:
        while i < len(raw) and raw[i:i + 1].isspace():
            i += 1
        if raw[i:i + 1] == b'#':
            while i < len(raw) and raw[i:i + 1] != b'\n':
                i += 1
            continue
        j = i
        while j < len(raw) and not raw[j:j + 1].isspace():
            j += 1
        tokens.append(int(raw[i:j]))
        i = j
    w, h, _ = tokens
    return w, h, np.frombuffer(raw[i + 1:i + 1 + w * h],
                               dtype=np.uint8).reshape(h, w)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--from', dest='p0', nargs=2, type=float, required=True,
                    metavar=('X', 'Y'))
    ap.add_argument('--to', dest='p1', nargs=2, type=float, required=True,
                    metavar=('X', 'Y'))
    ap.add_argument('--samples', type=int, default=15)
    ap.add_argument('--inflation', type=float, default=0.55,
                    help='inflation_radius，默认 0.55（nav2 默认值）')
    ap.add_argument('--map', default=DEFAULT_MAP)
    ap.add_argument('--yaml', default=DEFAULT_YAML)
    args = ap.parse_args()

    res, origin = read_yaml(args.yaml)
    ox, oy = origin[0], origin[1]
    w, h, img = read_pgm(args.map)
    occ_r, occ_c = np.where(img <= 100)

    p0, p1 = tuple(args.p0), tuple(args.p1)
    straight = float(np.hypot(p1[0] - p0[0], p1[1] - p0[1]))

    print(f'地图 {w}x{h} px @ {res} m/px，障碍像素 {len(occ_r)}')
    print(f'直线距离 = {straight:.2f} m    膨胀半径 = {args.inflation} m')
    print()
    print(f'{"采样点 (x, y)":>22} | {"占用值":>6} | {"到最近墙(m)":>11} | 在膨胀区内')
    print('-' * 62)
    worst, worst_at, in_infl = 1e9, None, 0
    for t in np.linspace(0.0, 1.0, args.samples):
        x = p0[0] + (p1[0] - p0[0]) * t
        y = p0[1] + (p1[1] - p0[1]) * t
        c = int((x - ox) / res)
        r = h - 1 - int((y - oy) / res)
        if not (0 <= c < w and 0 <= r < h):
            print(f'  ({x:>8.2f},{y:>7.2f}) |   越界 |          - | -')
            continue
        val = int(img[r, c])
        d = float(np.sqrt(((occ_r - r) * res) ** 2 +
                          ((occ_c - c) * res) ** 2).min())
        if d < worst:
            worst, worst_at = d, (x, y)
        inside = d < args.inflation
        in_infl += int(inside)
        print(f'  ({x:>8.2f},{y:>7.2f}) | {val:>6} | {d:>11.2f} | '
              f'{"是" if inside else "否"}')
    print('-' * 62)
    print(f'最小离墙距离 = {worst:.2f} m  出现在 ({worst_at[0]:.2f}, {worst_at[1]:.2f})')
    print(f'{in_infl}/{args.samples} 个采样点落在膨胀区内')
    print()
    if worst < args.inflation:
        print('结论：直线大部分被膨胀层覆盖 → 代价高，navfn 绕行是「按代价最小」的正常结果。')
        print('      想让它走直路，可调小 inflation_radius / cost_scaling_factor。')
        print('      ⚠️ 不要指望换 use_astar：实测 A* 与 Dijkstra 走的是几乎同一条路')
        print('         （见 docs/问题记录.md E-16 与 tools/probe_planner.py）。')
    else:
        print('结论：直线上有足够余量，膨胀层不足以解释绕行 → 另有原因')
        print('      （检查实时障碍层是否把扫描点错误标记、或定位偏移导致幻影障碍）。')
    return 0


if __name__ == '__main__':
    sys.exit(main())

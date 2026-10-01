#!/usr/bin/env python3
"""
查询 room.pgm 地图上若干坐标点的占用情况。

用途：给 Nav2 下目标点之前，先确认那个点在栅格地图里是「可走」的，
避免把导航失败误判成 Nav2 / AMCL 的问题。

用法:
  python3 tools/check_map_point.py 1 0 1 2 -4.5 1.5
  python3 tools/check_map_point.py --default     # 内置候选点
"""

import argparse
import os
import sys

import numpy as np

DEFAULT_MAP = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', 'src', 'fishbot_navigation2', 'maps', 'room.pgm')

DEFAULT_YAML = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', 'src', 'fishbot_navigation2', 'maps', 'room.yaml')


def read_yaml(path):
    """极简 YAML 读取：只取 resolution / origin。"""
    res, origin = None, None
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line.startswith('resolution:'):
                res = float(line.split(':')[1])
            elif line.startswith('origin:'):
                origin = [float(v) for v in
                          line.split(':', 1)[1].strip().strip('[]').split(',')]
    return res, origin


def read_pgm(path):
    raw = open(path, 'rb').read()
    if not raw.startswith(b'P5'):
        raise RuntimeError('只支持 P5 (binary) PGM')
    # 逐 token 解析头（跳过注释）
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
    w, h, _maxv = tokens
    i += 1  # 单个空白字符
    data = np.frombuffer(raw[i:i + w * h], dtype=np.uint8).reshape(h, w)
    return w, h, data


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('coords', nargs='*', type=float,
                    help='成对的 x y（米，map 坐标系）')
    ap.add_argument('--map', default=DEFAULT_MAP)
    ap.add_argument('--yaml', default=DEFAULT_YAML)
    ap.add_argument('--default', action='store_true', help='使用内置候选点')
    args = ap.parse_args()

    res, origin = read_yaml(args.yaml)
    ox, oy = origin[0], origin[1]
    w, h, data = read_pgm(args.map)

    pts = [(0, 0), (1, 0), (1, 2), (-4.5, 1.5), (-8, -5), (1, -5)] \
        if args.default or not args.coords else \
        [(args.coords[i], args.coords[i + 1])
         for i in range(0, len(args.coords) - 1, 2)]

    print(f'地图文件 : {os.path.normpath(args.map)}')
    print(f'尺寸     : {w} x {h} px, resolution={res} m/px')
    print(f'origin   : ({ox}, {oy})')
    print(f'覆盖范围 : x [{ox:.2f}, {ox + w * res:.2f}]  '
          f'y [{oy:.2f}, {oy + h * res:.2f}]')
    print()
    print(f'{"目标点 (x, y)":>20} | {"像素值":>6} | 判定')
    print('-' * 52)
    bad = 0
    for x, y in pts:
        c = int((x - ox) / res)
        r = h - 1 - int((y - oy) / res)
        if not (0 <= c < w and 0 <= r < h):
            v, verdict = None, '越界（不在地图内）'
        else:
            v = int(data[r, c])
            if v >= 230:
                verdict = '空闲（可走）'
            elif v <= 100:
                verdict = '障碍（墙/家具）'
            else:
                verdict = '未知区域'
        if verdict != '空闲（可走）':
            bad += 1
        print(f'  ({x:>7.2f}, {y:>6.2f}) | {str(v):>6} | {verdict}')
    print('-' * 52)
    print(f'不可走的目标点: {bad} / {len(pts)}')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())

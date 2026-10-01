#!/usr/bin/env python3
"""
判断「从 A 点能不能走到 B 点」，以及能走的话最窄处需要多大半径。

用途：Nav2 反复 "Failed to make progress" 然后 ABORT 时，先分清两件事：
  (1) 目标在**几何上就不可达**（比如落在桌子/柜子里）→ 别怪 Nav2；
  (2) 可达，但唯一的路线要挤过一个很窄的缝 → 车过不去，问题在膨胀层/机器人半径。

做法：二分「半径 r」，把离墙距离 >= r 的栅格连成一片，看起点终点是否仍连通。
最大的 r 就是「最宽路线的最窄处半径」——只要 r > 车半径，理论上就挤得过去。

⚠️ 占用判据必须和 map_server 一致，否则会凭空造出墙来：
   room.yaml 是 trinary 模式，occupied_thresh=0.65, free_thresh=0.25，
   map_server 用 occ = (255 - pixel) / 255 判断：
       occ > 0.65  -> 障碍（本图里是 0）
       occ < 0.25  -> 自由（本图里是 205 和 254）
       否则        -> 未知
   **注意 205 是「自由」，不是墙** —— 踩过坑：把它当障碍，地图上会多出
   一整片不存在的墙，"目标不可达"的结论全是假的。

用法:
  python3 tools/map_reachability.py --from 2.28 1.76 --to -4.5 1.5
  python3 tools/map_reachability.py --from 2.28 1.76 --to 2.17 1.88 -4.5 1.5
  python3 tools/map_reachability.py --histogram     # 只看像素值分布和占比
"""

import argparse
import os
import sys

import numpy as np

try:
    from scipy import ndimage
except ImportError:
    print('需要 scipy: pip install scipy（容器里一般已装）', file=sys.stderr)
    raise

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))


def _find_in_src(rel_path):
    """在 src/ 下定位文件，允许一级分组目录（src/<组>/<包>/...）。"""
    root = os.path.join(TOOLS_DIR, '..', 'src')
    if os.path.exists(os.path.join(root, rel_path)):
        return os.path.join(root, rel_path)
    for group in sorted(os.listdir(root)):
        cand = os.path.join(root, group, rel_path)
        if os.path.exists(cand):
            return cand
    return os.path.join(root, rel_path)


DEFAULT_MAP = _find_in_src('mybot_navigation2/maps/room.pgm')
DEFAULT_YAML = _find_in_src('mybot_navigation2/maps/room.yaml')

OCCUPIED_THRESH = 0.65
FREE_THRESH = 0.25


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
    """极简 P5 解析：读取 W H maxval，然后是像素数据。"""
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


class Map:
    def __init__(self, map_path=DEFAULT_MAP, yaml_path=DEFAULT_YAML):
        self.res, origin = read_yaml(yaml_path)
        self.ox, self.oy = origin[0], origin[1]
        self.w, self.h, self.img = read_pgm(map_path)
        occ = (255.0 - self.img.astype(float)) / 255.0
        self.free = occ < FREE_THRESH
        self.occ = occ > OCCUPIED_THRESH

    def rc(self, x, y):
        """世界坐标 -> (行, 列)。行号随 y 增大而减小（PGM 首行是 y 最大处）。"""
        return self.h - 1 - int(round((y - self.oy) / self.res)), \
            int(round((x - self.ox) / self.res))

    def value(self, x, y):
        r, c = self.rc(x, y)
        return int(self.img[r, c])

    def clearance(self, x, y):
        """该点到最近障碍栅格的距离（米）。"""
        d = ndimage.distance_transform_edt(self.free) * self.res
        return float(d[self.rc(x, y)])

    def bottleneck(self, p0, p1):
        """返回 (最宽路线最窄处半径, 说明)。半径单位米。"""
        d = ndimage.distance_transform_edt(self.free) * self.res
        st = np.ones((3, 3), bool)
        r0, c0 = self.rc(*p0)
        r1, c1 = self.rc(*p1)
        for name, (rr, cc) in (('起点', (r0, c0)), ('终点', (r1, c1))):
            if not (0 <= rr < self.h and 0 <= cc < self.w):
                return None, f'{name}超出地图范围'
            if not self.free[rr, cc]:
                return None, (f'{name}落在障碍格上（pixel={self.img[rr, cc]}）')
        lbl, _ = ndimage.label(self.free, structure=st)
        if lbl[r0, c0] != lbl[r1, c1]:
            return None, '在自由空间里就不连通（几何上不可达）'
        lo, hi = 0.0, 3.0
        for _ in range(25):
            mid = (lo + hi) / 2
            l2, _ = ndimage.label(d >= mid, structure=st)
            if l2[r0, c0] == l2[r1, c1] != 0:
                lo = mid
            else:
                hi = mid
        return lo, 'ok'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--from', dest='p0', nargs=2, type=float,
                    metavar=('X', 'Y'), help='起点')
    ap.add_argument('--to', dest='goals', nargs='+', type=float,
                    metavar='X Y', help='一个或多个目标，成对给出')
    ap.add_argument('--robot-radius', type=float, default=0.22,
                    help='车半径（默认 0.22，本项目 robot_radius）')
    ap.add_argument('--histogram', action='store_true',
                    help='只打印像素值分布，检查占用判据')
    ap.add_argument('--map', default=DEFAULT_MAP)
    ap.add_argument('--yaml', default=DEFAULT_YAML)
    args = ap.parse_args()

    m = Map(args.map, args.yaml)

    if args.histogram:
        print(f'地图 {m.w}x{m.h} px @ {m.res} m/px')
        vals, cnts = np.unique(m.img, return_counts=True)
        for v, c in zip(vals, cnts):
            o = (255.0 - int(v)) / 255.0
            if o > OCCUPIED_THRESH:
                kind = '障碍'
            elif o < FREE_THRESH:
                kind = '自由'
            else:
                kind = '未知'
            print(f'  pixel={v:>3}  {c:>6} px  occ={o:.3f}  -> {kind}')
        print(f'自由栅格合计 {int(m.free.sum())} / {m.free.size}')
        return 0

    if args.p0 is None or not args.goals:
        ap.error('需要 --from X Y 和 --to X Y [X Y ...]')
    if len(args.goals) % 2:
        ap.error('--to 的坐标必须成对')
    goals = list(zip(args.goals[0::2], args.goals[1::2]))

    print(f'起点 ({args.p0[0]:+.2f}, {args.p0[1]:+.2f})   '
          f'离墙 {m.clearance(*args.p0):.2f} m   车半径 {args.robot_radius} m')
    print()
    for gx, gy in goals:
        r, msg = m.bottleneck(tuple(args.p0), (gx, gy))
        head = f'  -> ({gx:+6.2f}, {gy:+6.2f})  '
        if r is None:
            print(head + f'不可达：{msg}')
            continue
        verdict = '车挤得过去' if r > args.robot_radius else '★ 车挤不过去'
        print(head + f'最窄处半径 {r:5.3f} m   {verdict}')
    print()
    print('提示：最窄处半径只要 > 车半径就说明「几何上能过」，')
    print('      此时若 Nav2 仍反复失败，问题在膨胀层参数/执行环节，不在可达性。')
    return 0


if __name__ == '__main__':
    sys.exit(main())

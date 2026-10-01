#!/usr/bin/env python3
"""
汇总 tools/probe_cmd_chain.py 录下来的速度链数据，判断零速指令出自哪一级。

判读原则：
  * 三级都非零，但车不动           -> 物理问题
  * 某一级恒为 0 而上一级非 0       -> 就是那一级吃掉了指令
  * 第一级就是 0                    -> controller_server 自己没输出

用法:
  python3 tools/analyze_cmd_chain.py [/tmp/chain.txt]
"""

import sys

NAMES = ['/cmd_vel_nav', '/cmd_vel_smoothed', '/cmd_vel', '/cmd_vel_teleop']
NZ = 0.005


def load(path):
    rows = []
    with open(path) as f:
        lines = [ln.rstrip() for ln in f]
    for ln in lines[2:]:            # 前两行是表头
        p = ln.split()
        if len(p) < 9:
            continue
        # 没有数据的话题用 '-' 占位，按 0 处理（列数不变，位置仍然对齐）
        vals = [0.0 if x == '-' else float(x) for x in p[1:9]]
        rows.append([float(p[0])] + vals)
    return rows


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/chain.txt'
    rows = load(path)
    if not rows:
        print(f'{path} 里没有数据')
        return 1

    span = rows[-1][0] - rows[0][0]
    print(f'{path}   采样 {len(rows)} 点   跨度 {span:.1f} s')
    print()
    print('各话题总览：')
    for i, n in enumerate(NAMES):
        v = [r[1 + 2 * i] for r in rows]
        nz = sum(1 for x in v if abs(x) > NZ)
        print(f'  {n:<20} vx [{min(v):+.3f}, {max(v):+.3f}]   '
              f'非零 {nz}/{len(v)} = {nz / len(v):.0%}')

    print()
    print('按 10 秒分桶（每桶内 linear.x 非零占比）：')
    print(f'{"t(s)":>6} | {"nav":>6} {"smooth":>6} {"cmd":>6} | 说明')
    t0 = rows[0][0]
    buckets = {}
    for r in rows:
        buckets.setdefault(int((r[0] - t0) // 10), []).append(r)
    for b in sorted(buckets):
        rs = buckets[b]
        frac = []
        for i in range(3):
            v = [r[1 + 2 * i] for r in rs]
            frac.append(sum(1 for x in v if abs(x) > NZ) / len(v))
        note = ''
        if frac[2] < 0.3:
            note = '接近静止'
        elif frac[0] < 0.3 < frac[2]:
            note = 'nav 停但 cmd 还在动'
        print(f'{b * 10:>6} | {frac[0]:>6.2f} {frac[1]:>6.2f} {frac[2]:>6.2f} | {note}')

    print()
    print('结论判据：某一级恒为 0 而上一级非 0 -> 那一级吃掉了指令。')
    return 0


if __name__ == '__main__':
    sys.exit(main())

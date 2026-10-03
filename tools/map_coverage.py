#!/usr/bin/env python3
"""
有避障的自动巡航建图 —— 用来建一张**不发散**的图，并定量监测「odom 漂了多少 / SLAM 漂了多少」。

为什么需要这个工具（教训详见 `docs/问题记录.md` 的 E-13 / F-14 / F-15）：

  1. **航点净空 ≠ 路径净空**。只保证「每个目标点周围空旷」是不够的 ——
     两个各自合格的航点之间的直线路径完全可能穿过家具。若像贪心那样
     「朝目标直发速度、不看障碍」，机器人会笔直撞上去并卡死。
  2. 机器人一旦卡住，控制器还在发速度指令，轮子空转打滑 → odom 继续累积
     → SLAM 拿这个错的 odom 当**运动先验** → 扫描匹配失败 → 位姿漂走。
     实测：卡死两次后，地图从 18.8×11.1 m 膨胀到 20.7×20.6 m。
  3. `open_loop` **不是**那个变量：改成 `false` 反而更差（odom 误差 16.5 → 19.97 m），
     因为打滑时轮子转速也确实非零，两种设置都会把打滑积进去。

本工具因此做了四层保护：

  ① 逐像素 BFS 算净空选航点（不用跳步抽样，否则 5~10 cm 的细障碍会漏检）
  ② 航点之间在地图上用 BFS 规划路径（障碍按「底盘半径 + 余量」膨胀）
  ③ 运行时 `/scan` 安全网：前方 ±60° 内近于 `--stop-range` 就只转不冲
  ④ 卡死检测：命令前进却 3 s 内位移 < 5 cm → 倒车 + 转向脱困

用法（在仓库根目录执行；路径都是**容器内**路径）:

  # 先起仿真和 SLAM（两个终端）
  ./scripts/sim.sh --headless --clean
  ./scripts/shell.sh -c 'ros2 launch slam_toolbox online_async_launch.py'

  # 再跑本工具
  ./scripts/shell.sh -c 'python3 /workspace/my-slam/tools/map_coverage.py'
  ./scripts/shell.sh -c 'python3 /workspace/my-slam/tools/map_coverage.py --spacing 2.0 --vmax 0.12'
  ./scripts/shell.sh -c 'python3 /workspace/my-slam/tools/map_coverage.py --save /workspace/my_map'

判读输出：

  | 列 | 含义 |
  | --- | --- |
  | `odom误差` | `odom->base_footprint` 与 Gazebo 真值之差 —— **会一直涨，正常** |
  | `SLAM误差` | `map->base_footprint` 与真值之差 —— **这个才是要看住的** |

  正常行驶时 **SLAM 误差 0.01~0.09 m，比 odom 误差还小** ——
  这正是扫描匹配在修正 odom 漂移的直接证据。
  若 SLAM 误差开始单调跟着 odom 涨、同时真值不再变化 → 机器人卡住了，这张图要废。

退出后建议用 `tools/map_clearance.py` 的同类手法核对：
**「障碍格的世界包围盒」是最灵敏的发散判据** ——
正常应约等于房间尺寸（18.7×11.0 m），发散会明显变大（19.9×17.2 m）。
"""

import argparse
import math
import os
import subprocess
import sys
import time
from collections import deque

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

ROBOT_R = 0.12          # 底盘碰撞半径（base_xacro: cylinder radius = length = 0.12）
PATH_MARGIN = 0.08      # 规划时额外留的余量
LOOKAHEAD = 0.30        # 路径跟踪前瞻距离
STUCK_T = 3.0           # 命令前进后多久没位移算卡死
STUCK_D = 0.05          # 「没位移」的阈值


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


# ---------------- 地图 I/O ----------------
def read_pgm(path):
    d = open(path, 'rb').read()
    tok, i = [], 0
    while len(tok) < 4:
        while i < len(d) and d[i:i + 1].isspace():
            i += 1
        if d[i:i + 1] == b'#':
            while i < len(d) and d[i:i + 1] != b'\n':
                i += 1
            continue
        j = i
        while j < len(d) and not d[j:j + 1].isspace():
            j += 1
        tok.append(d[i:j])
        i = j
    w, h = int(tok[1]), int(tok[2])
    while i < len(d) and d[i:i + 1].isspace():
        i += 1
    px = list(d[i:i + w * h])
    return w, h, [px[r * w:(r + 1) * w] for r in range(h)]


def read_yaml(path):
    d = {}
    for line in open(path):
        if ':' in line:
            k, v = line.split(':', 1)
            d[k.strip()] = v.strip().strip('[]').replace(',', ' ').split()
    return d


class Grid:
    """占据栅格 + 到最近障碍的**逐像素**距离场。

    占用判据与 `map_server` 的 trinary 阈值一致：`pixel < 200` 当障碍
    （见 F-2：205 是自由空间，不要当障碍）。
    """

    def __init__(self, pgm, yml):
        self.w, self.h, self.g = read_pgm(pgm)
        y = read_yaml(yml)
        self.res = float(y['resolution'][0])
        self.ox, self.oy = float(y['origin'][0]), float(y['origin'][1])
        INF = 10 ** 9
        self.dist = [[INF] * self.w for _ in range(self.h)]
        q = deque()
        for r in range(self.h):
            for c in range(self.w):
                if self.g[r][c] < 200:
                    self.dist[r][c] = 0
                    q.append((r, c))
        while q:
            r, c = q.popleft()
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nr, nc = r + dr, c + dc
                if (0 <= nr < self.h and 0 <= nc < self.w
                        and self.dist[nr][nc] > self.dist[r][c] + 1):
                    self.dist[nr][nc] = self.dist[r][c] + 1
                    q.append((nr, nc))

    def w2p(self, x, y):
        return (int(round((x - self.ox) / self.res)),
                self.h - 1 - int(round((y - self.oy) / self.res)))

    def p2w(self, c, r):
        return self.ox + c * self.res, self.oy + (self.h - 1 - r) * self.res

    def clr(self, x, y):
        """到最近障碍**边缘**的距离(m)。"""
        c, r = self.w2p(x, y)
        if not (0 <= c < self.w and 0 <= r < self.h):
            return -1.0
        return max(0.0, (self.dist[r][c] - 0.5) * self.res)

    def ok(self, c, r, infl):
        return 0 <= c < self.w and 0 <= r < self.h and self.dist[r][c] > infl

    def snap(self, c, r, infl):
        """起点落在非法格时（例如被安全网逼得太近），BFS 找最近的合法格。"""
        if self.ok(c, r, infl):
            return c, r
        seen = {(c, r)}
        q = deque([(c, r)])
        while q:
            cc, rr = q.popleft()
            if self.ok(cc, rr, infl):
                return cc, rr
            for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                n = (cc + dc, rr + dr)
                if n not in seen and 0 <= n[0] < self.w and 0 <= n[1] < self.h:
                    seen.add(n)
                    q.append(n)
        return None

    def plan(self, a, b, infl):
        """膨胀栅格上 8 邻域 BFS；返回世界坐标路径（0.15 m 抽稀）或 None。"""
        ca, ra = self.w2p(*a)
        cb, rb = self.w2p(*b)
        s = self.snap(ca, ra, infl)
        if s is None or not self.ok(cb, rb, infl):
            return None
        ca, ra = s
        prev = {(ca, ra): None}
        q = deque([(ca, ra)])
        while q:
            c, r = q.popleft()
            if (c, r) == (cb, rb):
                px = []
                cur = (c, r)
                while cur is not None:
                    px.append(cur)
                    cur = prev[cur]
                px.reverse()
                pts = [self.p2w(c2, r2) for c2, r2 in px[::3]]
                if pts[-1] != self.p2w(cb, rb):
                    pts.append(self.p2w(cb, rb))
                return pts
            for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1),
                           (1, 1), (1, -1), (-1, 1), (-1, -1)):
                nc, nr = c + dc, r + dr
                if (nc, nr) not in prev and self.ok(nc, nr, infl):
                    prev[(nc, nr)] = (c, r)
                    q.append((nc, nr))
        return None


def pick_waypoints(g, margin_m, spacing_m):
    step = max(1, int(round(spacing_m / g.res)))
    need = margin_m / g.res + 0.5
    out = []
    for r in range(0, g.h, step):
        for c in range(0, g.w, step):
            if g.dist[r][c] >= need:
                out.append(g.p2w(c, r))
    return out


def truth_xy():
    """Gazebo 真值。输出最后两行是 XYZ 和 RPY，取倒数第二行。"""
    try:
        o = subprocess.run(['gz', 'model', '-m', 'mybot', '-p'],
                           capture_output=True, text=True, timeout=5).stdout
        lines = [l for l in o.strip().splitlines() if l.strip()]
        v = [float(x) for x in lines[-2].strip().strip('[]').split()]
        return (v[0], v[1])
    except Exception:
        return None


class Driver:
    def __init__(self, node, args):
        self.node = node
        self.args = args
        self.tv = 0.0
        self.tw = 0.0
        self.scan = None
        self.pose = None
        self.odom = None
        self.pub = node.create_publisher(TwistStamped, '/cmd_vel', 10)
        # 用独立定时器发速度：绝不能在控制循环里现算现发（迭代耗时会把频率压到
        # cmd_vel_timeout 以下，机器人会一顿一顿 —— 见 E-13 的排查过程）
        node.create_timer(0.05, self._pub)
        node.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, node)
        node.create_timer(0.1, self._on_tf)

    def _pub(self):
        m = TwistStamped()
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.twist.linear.x, m.twist.angular.z = self.tv, self.tw
        self.pub.publish(m)

    def _on_scan(self, msg):
        self.scan = msg

    @staticmethod
    def _xy(t):
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        return (t.transform.translation.x, t.transform.translation.y, yaw)

    def _on_tf(self):
        try:
            self.pose = self._xy(self.buf.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()))
        except Exception:
            pass
        try:
            self.odom = self._xy(self.buf.lookup_transform(
                'odom', 'base_footprint', rclpy.time.Time()))
        except Exception:
            pass

    def fwd_min(self, half_deg, cap=1.5):
        s = self.scan
        if s is None or not s.ranges:
            return cap
        n = len(s.ranges)
        inc = s.angle_increment or 0.01
        i0 = max(0, int(math.ceil((math.radians(-half_deg) - s.angle_min) / inc)))
        i1 = min(n - 1, int(math.floor((math.radians(half_deg) - s.angle_min) / inc)))
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

    def escape(self):
        """倒车 + 转向脱困。"""
        self.tv, self.tw = -0.12, 0.0
        self.spin(1.3)
        self.tv, self.tw = 0.0, 0.9
        self.spin(1.3)
        self.tv = self.tw = 0.0

    def follow(self, pts, timeout=50.0):
        """沿路径走。返回 ok / stuck / timeout"""
        t0 = time.time()
        idx = 0
        ref = None
        while time.time() - t0 < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if self.pose is None:
                continue
            px, py, pyaw = self.pose
            gx, gy = pts[-1]
            goal_d = math.hypot(gx - px, gy - py)
            if goal_d < 0.20:
                self.tv = self.tw = 0.0
                return 'ok'
            while (idx < len(pts) - 1
                   and math.hypot(pts[idx][0] - px, pts[idx][1] - py) < LOOKAHEAD):
                idx += 1
            ax, ay = pts[idx]
            head = math.atan2(ay - py, ax - px)
            err = math.atan2(math.sin(head - pyaw), math.cos(head - pyaw))
            if self.fwd_min(self.args.scan_half_deg) < self.args.stop_range:
                self.tv = 0.0
                self.tw = 0.7 if err >= 0 else -0.7
            elif abs(err) > 0.5:
                self.tv = 0.0
                self.tw = 0.8 if err > 0 else -0.8
            else:
                self.tv = min(self.args.vmax, 0.5 * goal_d)
                self.tw = 1.3 * err
            # 卡死检测。⚠️ 基准必须排除「原地旋转」：旋转时 x,y 本来就不变，
            # 不重置基准的话，刚切到前进的那一瞬间就会误判卡死（见 F-14）。
            if self.tv <= 0.0:
                ref = (px, py, time.time())
            elif ref is None:
                ref = (px, py, time.time())
            elif math.hypot(px - ref[0], py - ref[1]) > STUCK_D:
                ref = (px, py, time.time())
            elif time.time() - ref[2] > STUCK_T:
                gt = truth_xy()
                print('      [卡死] SLAM位姿(%.2f,%.2f) 真值(%s) tv=%.3f goal_d=%.2f '
                      'err=%.2f 静置%.1fs'
                      % (px, py, ('%.2f,%.2f' % gt) if gt else 'n/a',
                         self.tv, goal_d, err, time.time() - ref[2]))
                self.tv = self.tw = 0.0
                return 'stuck'
        self.tv = self.tw = 0.0
        return 'timeout'


def main():
    ap = argparse.ArgumentParser(
        description='有避障的自动巡航建图（建一张不发散的图，并监测 odom/SLAM 漂移）')
    ap.add_argument('--map', default=DEFAULT_MAP, help='规划用的参考图 .pgm')
    ap.add_argument('--yaml', default=DEFAULT_YAML, help='规划用的参考图 .yaml')
    ap.add_argument('--spacing', type=float, default=1.6, help='航点网格间距 (m)，默认 1.6')
    ap.add_argument('--wp-margin', type=float, default=0.45,
                    help='航点到最近障碍的最小净空 (m)，默认 0.45')
    ap.add_argument('--vmax', type=float, default=0.15, help='前进速度上限 (m/s)，默认 0.15')
    ap.add_argument('--stop-range', type=float, default=0.40,
                    help='安全网停距 (m)，默认 0.40。'
                         '⚠️ 必须 >= 底盘半径 0.12 + 路径膨胀，否则会把自己逼进'
                         '「规划器认为的非法格」→ 全部航点规划失败（见 F-15）')
    ap.add_argument('--scan-half-deg', type=float, default=60.0,
                    help='安全网扫描半角 (度)，默认 60（只看正前方会漏掉贴墙转弯）')
    ap.add_argument('--save', metavar='PREFIX',
                    help='跑完后用 map_saver_cli 存图（容器内路径，如 /workspace/my_map）')
    ap.add_argument('--timeout', type=float, default=50.0, help='单个航点超时 (s)')
    args = ap.parse_args()

    g = Grid(args.map, args.yaml)
    infl = int(math.ceil((ROBOT_R + PATH_MARGIN) / g.res))
    if args.stop_range < ROBOT_R + infl * g.res:
        print('⚠️ --stop-range %.2f < 底盘半径 %.2f + 路径膨胀 %.2f = %.2f'
              % (args.stop_range, ROBOT_R, infl * g.res, ROBOT_R + infl * g.res))
        print('   安全网停得太近，机器人可能把自己逼进非法格（见 F-15）')
    wps = pick_waypoints(g, args.wp_margin, args.spacing)
    print('参考图 %dx%d px, res %.3f m；严格选点 %d 个（净空>=%.2f m，间距 %.1f m）'
          % (g.w, g.h, g.res, len(wps), args.wp_margin, args.spacing))
    print('路径膨胀 %d px = %.2f m' % (infl, infl * g.res))

    rclpy.init()
    node = rclpy.create_node(
        'map_coverage',
        parameter_overrides=[Parameter('use_sim_time', value=True)])
    d = Driver(node, args)
    d.spin(6.0)
    if d.pose is None:
        print('!! 拿不到 map->base_footprint；SLAM 起了吗？')
        return 1

    cur = (d.pose[0], d.pose[1])
    todo, order = list(wps), []
    while todo:
        todo.sort(key=lambda p: math.hypot(p[0] - cur[0], p[1] - cur[1]))
        nxt = todo.pop(0)
        if math.hypot(nxt[0] - cur[0], nxt[1] - cur[1]) < 0.8:
            continue
        order.append(nxt)
        cur = nxt
    print('规划出 %d 个航点，开始' % len(order))

    ok = skip = stuck = 0
    mon = []
    next_log = 0.0
    t0 = time.time()
    try:
        for i, wp in enumerate(order):
            if d.pose is None:
                break
            pts = g.plan((d.pose[0], d.pose[1]), wp, infl)
            if pts is None:
                skip += 1
                res = '无路径'
            else:
                res = d.follow(pts, args.timeout)
                if res == 'stuck':
                    stuck += 1
                    d.escape()
                    pts2 = g.plan((d.pose[0], d.pose[1]), wp, infl) if d.pose else None
                    res = d.follow(pts2, args.timeout) if pts2 else '脱困失败'
                    res = ('脱困:' + res) if res != 'ok' else '脱困成功'
            if res in ('ok', '脱困成功'):
                ok += 1
            gt = truth_xy()
            if gt and d.pose and d.odom:
                oe = math.hypot(d.odom[0] - gt[0], d.odom[1] - gt[1])
                se = math.hypot(d.pose[0] - gt[0], d.pose[1] - gt[1])
                mon.append((oe, se))
                if time.time() >= next_log:
                    print('  [%3d/%3d] %-10s 真值(%6.2f,%6.2f)  odom误差 %5.2f  '
                          'SLAM误差 %5.2f'
                          % (i + 1, len(order), res, gt[0], gt[1], oe, se))
                    next_log = time.time() + 3
    except KeyboardInterrupt:
        pass
    finally:
        d.tv = d.tw = 0.0
        d.spin(1.0)

    print('')
    print('======== 汇总 ========')
    print('  航点 %d：到位 %d / 无路径 %d / 卡死脱困 %d，用时 %.0f s'
          % (len(order), ok, skip, stuck, time.time() - t0))
    if mon:
        print('  odom 误差：最大 %.2f m / 末次 %.2f m'
              % (max(m[0] for m in mon), mon[-1][0]))
        print('  SLAM 误差：最大 %.2f m / 末次 %.2f m'
              % (max(m[1] for m in mon), mon[-1][1]))
        print('  判读：SLAM 误差应远小于 odom 误差且不持续单调上涨；')
        print('        若两者同步上涨且真值不变 → 机器人卡住了，图要废。')
    print('等地图更新 ...')
    time.sleep(12)

    if args.save:
        print('存图到 %s ...' % args.save)
        subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                        '-f', args.save, '--ros-args', '-p', 'use_sim_time:=true'])
    node.destroy_node()
    return 0


if __name__ == '__main__':
    sys.exit(main())

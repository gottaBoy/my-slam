#!/usr/bin/env python3
"""mybot 键盘遥控（不依赖「TTY raw 模式」的版本）。

为什么不直接用 teleop_twist_keyboard
------------------------------------
1. 它用 termios 把终端切换到 raw 模式逐个读按键，**必须有一个真正的 TTY**。
   在 `docker compose exec -T ...`、管道、非交互终端、后台进程里，它会永远阻塞
   在读键上：进程活着、publisher 也建好了，但一条指令都发不出去，而且不打任何
   日志。实测就是这个现象：
       $ ros2 topic info /cmd_vel -v     -> Publisher count: 1 (teleop_twist_keyboard)
       $ ros2 topic hz /cmd_vel          -> 完全没有数据
2. 它也默认发 `geometry_msgs/msg/Twist`，而本环境（Jazzy 的
   diff_drive_controller 4.x）只订阅 `TwistStamped`。

本脚本的差别
------------
* 用 select 做**非阻塞**读键：有 TTY 就是单键模式；不是 TTY 就自动退化成
  「按一个键 + 回车」的行模式，任何终端都能用。
* 以 10 Hz 持续重发最后一次的速度指令，避免 cmd_vel_timeout(0.5s) 触发刹车。
* 消息类型固定为 TwistStamped，并带上时间戳。

用法（在容器里）
----------------
    python3 /workspace/my-slam/src/robot/mybot_description/scripts/mybot_teleop.py
    python3 .../mybot_teleop.py --line     # 强制行模式（按完回车）
    python3 .../mybot_teleop.py --forward  # 不读键盘，直接前进，Ctrl-C 停

按键：w 前进 / s 后退 / a 左转 / d 右转 / k 或空格 停 / q 退出
"""

import argparse
import os
import select
import sys

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.parameter import Parameter

LINEAR_SPEED = 0.2     # m/s
ANGULAR_SPEED = 0.8    # rad/s
PUBLISH_HZ = 10.0      # 持续重发的频率，必须快于 cmd_vel_timeout

# 按键 -> (linear 系数, angular 系数)
BINDINGS = {
    'w': (1.0, 0.0),
    's': (-1.0, 0.0),
    'a': (0.0, 1.0),
    'd': (0.0, -1.0),
    'k': (0.0, 0.0),
    ' ': (0.0, 0.0),
}

# 这些字符只是「行结束/空白」，不代表要停车，必须忽略。
# （行模式下每行末尾都有 \n；raw 模式下 Enter 会送来 \r，都不能当成指令）
IGNORED_BYTES = {b'\n', b'\r', b'\t'}

QUIT_BYTES = {b'q', b'Q', b'\x03', b'\x04'}   # q / Ctrl-C / Ctrl-D

USAGE = """
按键（有 TTY 时不用回车，否则按完要回车）：
  w / s : 前进 / 后退
  a / d : 左转 / 右转
  k 或空格 : 停
  q : 退出
线速度 %.2f m/s，角速度 %.2f rad/s
""" % (LINEAR_SPEED, ANGULAR_SPEED)


class MybotTeleop(Node):
    def __init__(self):
        super().__init__('mybot_teleop')
        # 用仿真时间：TwistStamped 的时间戳要和控制器在同一个时钟上。
        # 注意 use_sim_time 已由 rclpy 自动声明，不能再 declare，只能 set。
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        # 关键：类型必须是 TwistStamped
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.linear = 0.0
        self.angular = 0.0

    def publish_cmd(self):
        msg = TwistStamped()
        stamp = self.get_clock().now()
        if stamp.nanoseconds != 0:      # /clock 还没来时不要写 0 时间戳
            msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x = self.linear
        msg.twist.angular.z = self.angular
        self.publisher.publish(msg)


def read_byte(fd):
    """无缓冲读一个字节；没有数据返回 None，EOF 返回 False。

    一定要用 os.read 而不是 sys.stdin.read()：sys.stdin 带缓冲区，
    read(1) 会把后续几个字节一起吞进 Python 内部缓冲区，之后 select
    在 fd 上探测不到数据，那些按键就永远读不出来（表现为“停下后再按
    前进没反应、后续按键全部错位”）。
    """
    try:
        data = os.read(fd, 1)
    except (OSError, ValueError):
        return False
    if not data:
        return False
    return data


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--line', action='store_true',
                        help='强制行模式（按一个键再回车），适合非 TTY 终端')
    parser.add_argument('--forward', action='store_true',
                        help='不读键盘，直接以固定速度前进，Ctrl-C 停止')
    args = parser.parse_args()

    # select 和 os.read 必须作用在同一个无缓冲 fd 上，否则会丢键。
    # 终端模式也在这里就切好：rclpy 初始化要 1~2 秒，如果等初始化完再切
    # termios，这段时间里敲的键会被那一次模式切换冲掉（表现为“刚启动时按了没反应”）。
    try:
        stdin_fd = sys.stdin.fileno()
    except (AttributeError, ValueError, OSError):
        stdin_fd = None

    raw_mode = stdin_fd is not None and sys.stdin.isatty() and not args.line
    old_term = None
    if raw_mode:
        import termios
        import tty
        old_term = termios.tcgetattr(stdin_fd)
        tty.setcbreak(stdin_fd)

    rclpy.init()
    node = MybotTeleop()

    mode_txt = '单键（无需回车）' if raw_mode else '行模式（按完回车）'
    if args.forward:
        mode_txt = '固定前进，Ctrl-C 停'
    print(USAGE, flush=True)
    print('模式：%s    发布类型：TwistStamped -> /cmd_vel' % mode_txt, flush=True)
    print('把这个终端保持在前台，用 Gazebo 窗口看效果。', flush=True)

    period = 1.0 / PUBLISH_HZ
    try:
        while rclpy.ok():
            if args.forward:
                node.linear = LINEAR_SPEED
                node.angular = 0.0
            elif stdin_fd is not None:
                # 把所有已到达的按键一次性处理完
                while select.select([stdin_fd], [], [], 0.0)[0]:
                    data = read_byte(stdin_fd)
                    if data is False:            # EOF（管道结束 / Ctrl-D）
                        rclpy.shutdown()
                        break
                    if data in IGNORED_BYTES:    # 行尾的 \n、\r、\t：忽略
                        continue
                    if data in QUIT_BYTES:
                        rclpy.shutdown()
                        break
                    factor = BINDINGS.get(
                        data.decode('utf-8', errors='ignore'), (0.0, 0.0))
                    node.linear = factor[0] * LINEAR_SPEED
                    node.angular = factor[1] * ANGULAR_SPEED
                    print('  -> linear.x=%.2f  angular.z=%.2f'
                          % (node.linear, node.angular), flush=True)

            if not rclpy.ok():
                break
            node.publish_cmd()
            rclpy.spin_once(node, timeout_sec=period)
    except KeyboardInterrupt:
        pass
    finally:
        if old_term is not None:
            import termios
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_term)
        # 退出前发一条 0 速度，避免车带着最后一条指令继续跑
        if rclpy.ok():
            node.linear = 0.0
            node.angular = 0.0
            node.publish_cmd()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print('\n已停止。')


if __name__ == '__main__':
    main()

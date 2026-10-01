#!/usr/bin/env python3
"""打印 ROS 2 的计算图（节点 ↔ 话题 的连接关系），不需要图形界面。

三条查看途径，按需要选：
  1. 图形界面最直观：./rqt.sh → Plugins → Introspection → Node Graph
  2. 命令行逐项查：ros2 node list / ros2 node info <节点> / ros2 topic info <话题> -v
  3. 本脚本：一次性把整张图打出来，headless 也能用；还能输出 Mermaid 直接粘进文档

用法（容器内，相关节点在跑时）：
    python3 /workspace/my-slam/tools/show_graph.py             # 文本列表
    python3 /workspace/my-slam/tools/show_graph.py --mermaid   # 输出 Mermaid 代码块
    python3 /workspace/my-slam/tools/show_graph.py --wait 5    # 多等一会儿（节点刚起来时用）

注意：ROS 2 的图是靠 DDS 发现机制「问」出来的，节点刚启动那几秒可能还没发现全，
      所以 --wait 默认等了 2.5 秒；刚启动就查的话可以加 --wait 5。
"""

import argparse
import sys
import time

import rclpy

# 默认藏起来的话题：它们是框架内部用的，画在图上只会把真正的数据流盖住。
# 想看全部就加 --all。
NOISE_EXACT = {'/parameter_events', '/rosout', '/diagnostics'}
NOISE_PREFIXES = ('/controller_manager/',)


def is_noise(topic):
    return topic in NOISE_EXACT or topic.startswith(NOISE_PREFIXES) \
        or topic.endswith('/transition_event')


def endpoint_names(node, topic, kind):
    """返回某个话题上所有发布者/订阅者的 '名字空间/节点名'。

    注意 rclpy 的 API 名：订阅侧是 get_subscriptions_info_by_topic
    （不是 get_subscribers_info_by_topic，写错会 AttributeError）。
    """
    infos = (node.get_publishers_info_by_topic(topic) if kind == 'pub'
             else node.get_subscriptions_info_by_topic(topic))
    names = set()
    for info in infos:
        ns = info.node_namespace.rstrip('/')
        names.add(f'{ns}/{info.node_name}' if ns else f'/{info.node_name}')
    return sorted(names)


def collect(node):
    nodes = sorted(
        f'{ns.rstrip("/")}/{name}' if ns.rstrip('/') else f'/{name}'
        for name, ns in node.get_node_names_and_namespaces())

    rows = []
    for topic, types in sorted(node.get_topic_names_and_types()):
        rows.append((topic,
                     types[0] if types else '?',
                     endpoint_names(node, topic, 'pub'),
                     endpoint_names(node, topic, 'sub')))
    return nodes, rows


def print_text(nodes, rows, hidden=0):
    print(f'节点（{len(nodes)} 个）')
    for name in nodes:
        print(f'  {name}')
    print()

    connected = [r for r in rows if r[2] or r[3]]
    isolated = [r for r in rows if not r[2] and not r[3]]
    print(f'话题连接（{len(connected)} 个有连接，{len(isolated)} 个孤立话题）')
    if hidden:
        print(f'   （已隐藏 {hidden} 个框架内部话题；要看全部加 --all）')
    for topic, ttype, pubs, subs in connected:
        print(f'  {topic}   [{ttype}]')
        print(f'      发布: {", ".join(pubs) if pubs else "（无）"}')
        print(f'      订阅: {", ".join(subs) if subs else "（无）"}')


def print_mermaid(rows):
    def node_id(prefix, name):
        return prefix + name.strip('/').replace('/', '_').replace(':', '_')

    print('```mermaid')
    print('graph LR')
    for topic, ttype, pubs, subs in rows:
        if not pubs and not subs:
            continue
        tid = node_id('T_', topic)
        print(f'  {tid}(["{topic}<br/>{ttype}"])')
        for p in pubs:
            print(f'  {node_id("N_", p)}["{p}"] --> {tid}')
        for s in subs:
            print(f'  {tid} --> {node_id("N_", s)}["{s}"]')
    print('```')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mermaid', action='store_true',
                        help='输出 Mermaid 图代码（可粘进 Markdown 直接看到图）')
    parser.add_argument('--all', action='store_true',
                        help='连 /rosout、/parameter_events、controller_manager 内省话题一起显示')
    parser.add_argument('--wait', type=float, default=2.5,
                        help='等待 DDS 发现的时间，秒（默认 2.5）')
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node('graph_dumper')
    deadline = time.time() + args.wait
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    nodes, rows = collect(node)
    node.destroy_node()
    rclpy.shutdown()

    if not nodes:
        print('没发现任何节点 —— 是不是仿真还没起来，或者 ROS_DOMAIN_ID 不一致？')
        return 1

    hidden = 0
    if not args.all:
        kept = []
        for row in rows:
            if is_noise(row[0]):
                hidden += 1
            else:
                kept.append(row)
        rows = kept

    if args.mermaid:
        print_mermaid(rows)
    else:
        print_text(nodes, rows, hidden)
    return 0


if __name__ == '__main__':
    sys.exit(main())

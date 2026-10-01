#!/usr/bin/env python3
"""
从 Nav2 官方默认 rviz 配置生成一份「去 TurtleBot 化」的配置。

为什么需要：nav2_bringup 的 nav2_default_view.rviz 是给 TurtleBot3 配的，里面
有两个显示项订阅的话题我们根本没有，挂着只会徒增困惑：

  * Bumper Hit  -> /mobile_base/sensors/bumper_pointcloud   （TB3 防撞条）
  * Realsense   -> /intel_realsense_r200_depth/image_raw    （TB3 的深度相机）

做法是：删掉 Bumper Hit，把 Realsense 组换成我们自己的相机（/camera/image、
/camera/points），其余显示项（Map / 代价地图 / 路径 / 粒子云 / TF / RobotModel /
LaserScan / Global Planner / Controller / MarkerArray）原样保留。

用法（容器内，需要 ROS 环境）：
  python3 tools/gen_nav2_rviz.py
生成结果：src/fishbot_navigation2/rviz/fishbot_nav2.rviz
"""

import os
import sys

import yaml

DEFAULT_SRC = '/opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz'
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   '..', 'src', 'fishbot_navigation2', 'rviz',
                   'fishbot_nav2.rviz')

DROP = {'Bumper Hit', 'Realsense'}


def make_camera_group():
    """照抄官方 Realsense 组的字段结构，只把话题换成我们的。"""
    return {
        'Class': 'rviz_common/Group',
        'Displays': [
            {
                'Class': 'rviz_default_plugins/Image',
                'Enabled': True,
                'Max Value': 1,
                'Median window': 5,
                'Min Value': 0,
                'Name': 'Camera Image',
                'Normalize Range': True,
                'Topic': {
                    'Depth': 5,
                    'Durability Policy': 'Volatile',
                    'History Policy': 'Keep Last',
                    'Reliability Policy': 'Reliable',
                    'Value': '/camera/image',
                },
                'Value': True,
            },
            {
                'Alpha': 1,
                'Autocompute Intensity Bounds': True,
                'Autocompute Value Bounds': {
                    'Max Value': 10, 'Min Value': -10, 'Value': True},
                'Axis': 'Z',
                'Channel Name': 'intensity',
                'Class': 'rviz_default_plugins/PointCloud2',
                'Color': '255; 255; 255',
                'Color Transformer': 'RGB8',
                'Decay Time': 0,
                'Enabled': True,
                'Invert Rainbow': False,
                'Max Color': '255; 255; 255',
                'Max Intensity': 4096,
                'Min Color': '0; 0; 0',
                'Min Intensity': 0,
                'Name': 'Camera Points',
                'Position Transformer': 'XYZ',
                'Selectable': True,
                'Size (Pixels)': 3,
                'Size (m)': 0.01,
                'Style': 'Flat Squares',
                'Topic': {
                    'Depth': 5,
                    'Durability Policy': 'Volatile',
                    'History Policy': 'Keep Last',
                    'Reliability Policy': 'Reliable',
                    'Value': '/camera/points',
                },
                'Use Fixed Frame': True,
                'Use rainbow': True,
                'Value': True,
            },
        ],
        'Enabled': False,   # 默认折叠，需要时自己勾上
        'Name': 'fishbot Camera',
    }


def main():
    if not os.path.exists(DEFAULT_SRC):
        print(f'找不到 {DEFAULT_SRC}，确认 ros-jazzy-nav2-bringup 已安装')
        return 1
    cfg = yaml.safe_load(open(DEFAULT_SRC))
    vm = cfg.get('Visualization Manager')
    if vm is None or 'Displays' not in vm:
        print('官方配置结构变了，Displays 节点找不到')
        return 1

    new_displays, dropped, camera_inserted = [], [], False
    for d in vm['Displays']:
        name = d.get('Name')
        if name not in DROP:
            new_displays.append(d)
            continue
        dropped.append(name)
        if name == 'Realsense':
            # 在原地插入我们自己的相机组，保持位置观感一致
            new_displays.append(make_camera_group())
            camera_inserted = True
    vm['Displays'] = new_displays
    if not camera_inserted:
        vm['Displays'].append(make_camera_group())

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, 'w') as f:
        f.write('# 由 tools/gen_nav2_rviz.py 从 nav2_bringup 的 '
                'nav2_default_view.rviz 生成，请勿手工大改。\n')
        yaml.safe_dump(cfg, f, allow_unicode=True, sort_keys=False,
                       default_flow_style=False, width=1000)

    names = []
    for d in vm['Displays']:
        names.append(d.get('Name', '?'))
        for sub in d.get('Displays', []) or []:
            names.append('  └ ' + sub.get('Name', '?'))
    print(f'已生成 {os.path.normpath(OUT)}')
    print(f'  移除: {", ".join(dropped) or "（无）"}')
    print(f'  加入: fishbot Camera -> /camera/image, /camera/points')
    print(f'  固定坐标系: '
          f'{vm.get("Global Options", {}).get("Fixed Frame", "?")}')
    print('  其余显示项:')
    for n in names:
        print(f'    {n}')
    return 0


if __name__ == '__main__':
    sys.exit(main())

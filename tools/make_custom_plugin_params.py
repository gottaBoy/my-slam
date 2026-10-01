#!/usr/bin/env python3
"""
从本仓库的 nav2_params.yaml 生成一份「启用自研插件」的临时参数文件。

为什么不直接改仓库里的配置：nav2_custom_planner 是在起终点之间做**直线插值、
不绕障**的示例规划器，nav2_custom_controller 只是朝目标直行且限速 0.1 m/s。
把它们设成默认会让已验证的巡逻/导航直接降级。所以默认保持 DWB + navfn，
需要体验时用这个脚本生成一份临时参数，通过 launch 的 params_file 覆盖即可，
跑完即弃、不影响仓库状态。

用法（容器内）：
  python3 tools/make_custom_plugin_params.py /tmp/nav2_custom_test.yaml
  ros2 launch mybot_navigation2 navigation2.launch.py \
      rviz:=false params_file:=/tmp/nav2_custom_test.yaml
"""

import os
import sys

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


DEFAULT_SRC = _find_in_src('mybot_navigation2/config/nav2_params.yaml')

CONTROLLER_FROM = '      plugin: "dwb_core::DWBLocalPlanner"'
CONTROLLER_TO = (
    '      plugin: "nav2_custom_controller::CustomController"\n'
    '      max_linear_speed: 0.1\n'
    '      max_angular_speed: 1.0')

PLANNER_FROM = '      plugin: "nav2_navfn_planner::NavfnPlanner"'
PLANNER_TO = (
    '      plugin: "nav2_custom_planner::CustomPlanner"\n'
    '      interpolation_resolution: 0.1')


def main():
    if len(sys.argv) not in (2, 3):
        print(__doc__)
        return 2
    out = sys.argv[1]
    src = sys.argv[2] if len(sys.argv) == 3 else DEFAULT_SRC

    text = open(src).read()
    for name, old, new in (('控制器', CONTROLLER_FROM, CONTROLLER_TO),
                           ('规划器', PLANNER_FROM, PLANNER_TO)):
        if text.count(old) != 1:
            print(f'[FAIL] {name}: 在 {src} 里找到 {text.count(old)} 处 '
                  f'"{old.strip()}"，期望 1 处。'
                  '（配置结构变了？请同步更新本脚本）')
            return 1
        text = text.replace(old, new)

    open(out, 'w').write(text)
    print(f'已生成 {out}')
    print('  控制器: dwb_core::DWBLocalPlanner -> '
          'nav2_custom_controller::CustomController'
          '（max_linear_speed 0.1 / max_angular_speed 1.0）')
    print('  规划器: nav2_navfn_planner::NavfnPlanner -> '
          'nav2_custom_planner::CustomPlanner'
          '（interpolation_resolution 0.1）')
    print('提示：这两个插件都是示例实现——规划器直线插值不绕障，控制器直行限速 0.1 m/s，')
    print('      请在空旷区域用小目标点测试。')
    return 0


if __name__ == '__main__':
    sys.exit(main())

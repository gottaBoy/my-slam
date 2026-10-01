# slam-ros2-dev

独立的 ROS 2 Jazzy 开发环境，用于 `/home/my/workspace/slam` 下的多个项目。
本目录只存放这个环境新增的 Docker 配置和启动脚本。

## 特性

- 全新构建，基础镜像为官方 `ros:jazzy-ros-base`，桌面和可视化组件在本镜像中单独安装
- 包含 `rqt`、`rviz2`、Gazebo Sim、SLAM Toolbox、Nav2、Cartographer、ROS 2 Control 和常用编译工具
- 主机目录 `/home/my/workspace/slam` 挂载到容器 `/workspace`，因此容器内可以访问该目录下的多个项目
- 使用独立 Docker bridge network：`slam-ros2-dev-net`
- 默认 `ROS_DOMAIN_ID=49`、`GZ_PARTITION=slam-dev-49`
- 容器用户为 `nvidia`，家目录为 `/home/nvidia`，默认密码为 `nvidia`
- 默认不使用 host network、`privileged`、设备映射或 Docker socket
- 容器内用户 UID/GID 与启动用户一致，避免生成 root 文件

容器密码只用于容器内部的登录或 `su`，不代表主机密码，也不适合生产环境。

## SLAM 开发库

2026-09-28 对容器内的开发包、头文件及编译配置检查结果：

| 库 | 当前状态 | 版本 |
| --- | --- | --- |
| Eigen | 已安装 `libeigen3-dev` | 3.4.0 |
| OpenCV | 已安装 `libopencv-dev` 及模块开发包 | 4.6.0 |
| Ceres | 已安装 `libceres-dev` | 2.2.0 |
| Pangolin | 未安装 | - |
| g2o | 未安装 | - |

Eigen、OpenCV、Ceres 目前由 ROS 相关依赖引入，并非 Dockerfile 中显式指定的包；
Pangolin、g2o 未自动补装，也未修改其他项目自带的依赖。

## 工作空间结构

> 每个模块是干什么的、怎么跑、有没有验证过 → [`docs/模块清单.md`](docs/模块清单.md)
> 踩过的坑与排查过程（问题 / 现象 / 原因 / 解决） → [`docs/问题记录.md`](docs/问题记录.md)

`my-slam/src/` 下 **22 个包**，按功能分 4 组：

```text
src/
├─ robot/         机器人本体：描述、仿真、启动、传感器桥
│   mybot_description        URDF/Xacro 模型、Gazebo 世界、仿真启动
│   mybot_bringup            启动：真机 bringup + 仿真版一键启动
│   gz_sensor_bridge           IMU + 相机话题的桥（可选）
├─ navigation/    Nav2 配置与自研插件
│   mybot_navigation2        Nav2 参数 + 预制地图 + rviz 配置
│   nav2_custom_planner        Nav2 自定义全局规划器插件
│   nav2_custom_controller     Nav2 自定义控制器插件
├─ app/           应用层
│   mybot_application        Python 应用示例（4 个节点）
│   mybot_application_cpp    C++ 导航示例
│   autopatrol_interfaces      巡检应用的服务接口
│   autopatrol_robot           巡检主循环 + 语音播报
└─ learning/      学习型示例（12 个，与机器人主线无耦合）
    learn_compose / learn_dds_cpp
    learn_executor_cpp / _py
    learn_lifecyclenode_cpp / _py
    learn_message_filter_cpp / _py
    learn_qos_cpp / _py
    my_tf_cpp / my_tf_pkg
```

**分组只影响目录，不影响包名** —— colcon 递归发现 `src/` 下的包，
`ros2 launch` / `ros2 run` / `get_package_share_directory` 都按包名解析，
所以调整分组**不需要**改任何 launch 或脚本（`tools/` 下的脚本也已改成自动搜索）。

### 仓库根目录

```text
my-slam/
├─ compose.yaml / compose.nvidia.yaml   Compose 项目定义（`name: slam-ros2-dev` 写死）
├─ Dockerfile / entrypoint.sh           镜像构建 与 容器入口脚本
├─ ros-env.sh                           容器内非交互 shell 的环境（compose 的 BASH_ENV 指到它）
├─ scripts/                             宿主机侧全部入口
│    container-exec.sh                  公共库，被下面这些 wrapper source
│    start.sh / stop.sh / shell.sh      容器生命周期
│    docker_into.sh / docker_run.sh / docker_stop.sh
│                                       早期 Apollo 风格命名，只是上面三个的薄别名
│    sim.sh / stop-sim.sh / stop-nav2.sh   仿真与 Nav2 启停
│    teleop.sh / rviz2.sh / rqt.sh / gazebo.sh   交互工具
│    gpu-check.sh / check-models.py     环境与模型自检
├─ tools/                               诊断与验证脚本（大多在容器内跑）
├─ docs/                                模块清单、问题记录
└─ src/                                 ROS 2 工作空间（22 个包）
```

> `scripts/` 与 `tools/` 的分工：`scripts/` 在**宿主机**跑，管容器与仿真生命周期；
> `tools/` 大多在**容器内**跑，管排查与验证。
>
> `compose.yaml` 必须留在仓库根：`docker compose` 只读当前目录的配置文件，而所有
> wrapper 都是 `cd` 到仓库根后再跑裸 `docker compose`（不传 `-f`）。原因见
> [`docs/问题记录.md`](docs/问题记录.md) G-1。

## 完整启停流程（照着敲）

这一节是日常最常用的全部命令，自成一块。**所有命令都在宿主机上执行**，脚本在 `my-slam/scripts/`
下，会自己 `cd` 到仓库根（`compose.yaml` 所在处），所以在哪个目录调用都可以（推荐就是 `my-slam/`）。

```bash
cd /home/my/workspace/slam/my-slam
```

### 一、启动

```bash
./scripts/start.sh          # 1. 启动容器（首次会先构建镜像，见「构建网络记录」）
./scripts/sim.sh            # 2. 启动仿真（带 GUI）
```

`./scripts/sim.sh` 的常用参数：

```bash
./scripts/sim.sh --headless                       # 不渲染，只起 Gazebo server（远程/低配/验证）
./scripts/sim.sh --clean                          # 启动前先清掉上一次残留的仿真进程
./scripts/sim.sh --gz-verbose 4                   # gz 日志级别 0-4
./scripts/sim.sh --world /workspace/Dataset-of-Gazebo-Worlds-Models-and-Maps/worlds/empty_room/world.sdf
./scripts/sim.sh -- headless:=false verbose:=2    # `--` 之后原样透传给 ros2 launch
```

### 二、怎么看结果

```bash
# 另开一个终端：键盘遥控（w/s/a/d/k/q，不用回车）
./scripts/teleop.sh

# 另开一个终端：图形化看
./scripts/rviz2.sh     # Fixed Frame 选 base_footprint；Add -> By topic
               #   /camera/image (Image)、/scan (LaserScan)、/imu (Imu)
./scripts/rqt.sh       # 话题列表 / 节点关系 / TF 树等：Plugins 菜单里按需勾

# 看「节点 ↔ 话题」关系图（三种方式任选）
./scripts/rqt.sh       # ① 图形界面：Plugins → Introspection → Node Graph（最直观）
./scripts/shell.sh -c 'python3 /workspace/my-slam/tools/show_graph.py'            # ② 文本
./scripts/shell.sh -c 'python3 /workspace/my-slam/tools/show_graph.py --mermaid'  # ③ Mermaid，可粘进 Markdown
#   默认隐藏 /rosout、/parameter_events、controller_manager 内省话题等噪声；
#   要看全部加 --all；节点刚起来时加 --wait 5。

# 另开一个终端：命令行核对
./scripts/shell.sh -c 'ros2 topic list'
./scripts/shell.sh -c 'ros2 topic hz /scan'
./scripts/shell.sh -c 'ros2 run tf2_ros tf2_echo base_footprint camera_optical_link'

./scripts/shell.sh -c 'ros2 node list'                    # 有哪些节点
./scripts/shell.sh -c 'ros2 node info /gz_sensor_bridge'  # 单个节点的发布/订阅/服务
./scripts/shell.sh -c 'ros2 topic info /cmd_vel -v'       # 谁在发、谁在收、QoS 是什么

# 一键自检：检查图像/点云长度是否自洽、内参是否非零、IMU 重力是否合理
# 退出码 0 = 全部通过
./scripts/shell.sh -c 'python3 /workspace/my-slam/tools/check_sensor_msgs.py'
```

启动正常时应该看到（2026-09-30 实测值，允许小幅波动）：

| 话题 | 频率 | 说明 |
| --- | --- | --- |
| `/clock` | ~2000 Hz | 仿真时钟 |
| `/scan` `/scan/points` | 5.0 Hz | 2D 激光（`parameter_bridge`） |
| `/imu` | 100 Hz | IMU（自研桥 `gz_sensor_bridge`） |
| `/odom` | 50 Hz | 里程计 |
| `/joint_states` | 100 Hz | 关节状态 |
| `/tf` | 40~70 Hz | 坐标变换 |
| `/camera/image` | 约 4 Hz | 800×600 `rgb8`；受**离屏渲染**限制，实测 2.5~7.5 之间波动 |
| `/camera/depth_image` | 约 3 Hz | 800×600 `32FC1`，同样受渲染限制 |
| `/camera/camera_info` | 10 Hz | 相机内参 |
| `/camera/points` | 1.5 Hz | RGB-D 点云 |

### 三、停止

```bash
# 停仿真：到跑 ./scripts/sim.sh 的那个终端按 Ctrl-C 即可。
# 注意：./scripts/sim.sh --clean 的语义是「清理后接着启动」，不是停止，别拿它当停止用。

# 如果那个终端已经关了、或者 Ctrl-C 后还有残留进程，用这条手动清：
#   * 方括号是故意的 —— 防止 pkill 匹配到「正在执行清理的这条命令」自身而自杀
#   * 末尾的 || true 也是必须的 —— 没有匹配到进程时 pkill 返回 1，不加会让整条命令报错退出
./scripts/shell.sh -c 'pkill -9 -f "gz si[m]" || true; pkill -9 -f "ros2 launc[h]" || true'

# 停容器
./scripts/stop.sh
```

### 四、出问题时先看这里

| 现象 | 先做什么 |
| --- | --- |
| `failed to create drawable` | `./scripts/gpu-check.sh`，见「Gazebo 报 failed to create drawable 的排查」 |
| 某个话题没数据 | `./scripts/shell.sh -c 'ros2 topic hz /话题名'`，对照上面表格 |
| 有数据但内容看着不对 | `./scripts/shell.sh -c 'python3 /workspace/my-slam/tools/check_sensor_msgs.py'` |
| 容器里残留一堆进程 | 见「三、停止」里那条 `pkill`；要顺手重开一个仿真就用 `./scripts/sim.sh --clean` |
| 改了 `gz_sensor_bridge` 的代码 | `./scripts/shell.sh -c 'cd /workspace/my-slam && colcon build --packages-select gz_sensor_bridge'` |
| 想查「这个报错当时是怎么定位的」 | 看 `docs/问题记录.md`：按问题类型整理，每条统一写清**问题 / 现象 / 原因 / 解决方法** |
| 想知道某个包是干什么的、怎么跑 | 看 `docs/模块清单.md` |

关于相机帧率：`/camera/camera_info` 稳定跑在配置的 **10 Hz**（它不需要渲染），
而 `/camera/image`、`/camera/depth_image` 只有 3~4 Hz —— 瓶颈是 Gazebo 的离屏渲染，
不是桥（否则 camera_info 也会一起掉）。这两个话题的数据本身是正确的，只是刷新慢。

### 五、两个维护注意点（改这个仿真的人必看）

**1. `/scan`、`/scan/points` 必须用单向 `[`，不能用双向 `@`。**

gz 的激光本来就发布在这两个话题上；如果桥再建一条 ROS→GZ 方向，就会自激回环：
`激光 → 桥 → ROS /scan → 桥自己订阅 → 写回 gz /scan → 桥再收 → …`。
实测后果：`/scan` 从 5 Hz 飙到 **~18000 Hz**，数据完全不可用，还白烧 CPU。

检查方法：`grep -c "Creating ROS->GZ Bridge" 启动日志` 应该是 **0**；
`ros2 topic info /scan -v` 的 Publisher count 应该是 **1**。

**2. 往 launch 里新增常驻节点时，必须把它的进程名加进 `sim.sh` 的清理列表。**

漏加的后果实测过：旧实例不会被 `--clean` 清掉，ROS 侧同一个话题出现多个发布者，
频率变成期望值的 2~3 倍（`/imu` 曾测出 300 Hz），数据也会重复。

检查方法：`./scripts/shell.sh -c 'ros2 topic info /imu -v | grep "Publisher count"'` 应该是 **1**。

## 代码移植与适配记录

`ros2bookcode/` 是**只读参考**，不参与编译；`my-slam/src/` 是唯一活代码。
目标是每搬完一块就把来源那份留在原地、功能收进本仓库，最终只有一份代码。

### 通用流程（搬任何一块都照这个走）

| 步 | 做什么 | 怎么验收 |
| --- | --- | --- |
| 0 | 清理 `src/` 杂物（挪走非包目录） | `ls src` 只剩真包 |
| 1 | `diff -rq` 新代码 vs 本仓库，把差异分成「纯新增」和「改已有」 | 差异清单列全，不靠印象 |
| 2 | **改已有的**：只挑有依据的改动，逐条给理由 + 实测验证 | 每条改动都有可证伪的测量 |
| 3 | **纯新增的**：整包复制进 `my-slam/src/`，**排除同名包** | `colcon build` 通过，包名无重复 |
| 4 | 依赖：先编译看真实报错，缺什么装什么 | 缺的依赖同时写进 `Dockerfile` |
| 5 | 适配（多数是 Humble → Jazzy 的差异） | 见下面「Jazzy 参数坑」 |
| 6 | 端到端跑通 | 用**真值**（`gz model -m mybot -p`）核对，不看日志自述 |
| 7 | 回归测试 | 见「回归清单」 |
| 8 | 写文档 | 本节 |

**原则**：
1. **绝不整包覆盖已有的包** —— `mybot_description` 是本仓库的超集（含 Gazebo Sim 版），
   被覆盖会把 gz 版全弄丢，`./scripts/sim.sh` 直接报废。
2. **本仓库的包统一用 `mybot_*` 前缀**，一眼能看出哪些是自己维护的。

### 导航与巡检应用：搬了什么

收进 `my-slam/src/` 的 5 个包（`mybot_description` **不在此列**，它是本仓库自己的）：

| 包 | 内容 |
| --- | --- |
| `mybot_navigation2` | Nav2 bringup 配置 + 预制地图 `maps/room.pgm` |
| `mybot_application` | Python 例子（设初始位姿 / 查位姿 / 去一点 / 走路点） |
| `mybot_application_cpp` | C++ 版导航客户端 |
| `autopatrol_interfaces` | 自定义服务 `SpeachText`（书上拼写如此） |
| `autopatrol_robot` | 巡逻主循环 + 语音节点 + 配置 |

### 改了哪些已有代码（每条都有实测）

这批代码顺手改过 `mybot_description` 两个文件，而本仓库原有的基线更早，
所以这两处需要单独判断。

| 改动 | 采纳? | 依据 |
| --- | --- | --- |
| `wheel_separation: 0.17 → 0.20` | ✅ 采纳 | URDF 里轮子在 `y = ±0.10`，轮距就是 0.20。实测见下表 |
| 雷达 `update_rate: 5 → 10` | ✅ 采纳 | 建图/导航扫描更密；实测 `/scan` 9.99 Hz |
| 雷达 `<remapping>~/out:=scan1</remapping>` | ❌ 不采纳 | 同批 `nav2_params.yaml` 订阅的是 `/scan`，两边对不上，像是笔误 |
| 里程计协方差全改 0 | ❌ 不采纳 | 全 0 等于「完全可信」，会让 AMCL 几乎不修正里程计漂移 |
| `amcl: base_footprint` + `costmap: base_link` | ❌ 不改 | 查过官方默认 `nav2_params.yaml` 也是这样，不是书的问题 |

轮距的 A/B 实测（`tools/verify_wheel_separation.py`，判据是 **Gazebo 真值**）：

| 配置 | 由轮速反推的实际值 | 指令转角 | `/odom` 认为转了 | **真值实际转了** |
| --- | --- | --- | --- | --- |
| `0.17` | 0.1700 | 1.2119 rad | 1.2114（−0.04%） | **1.0305（−14.96%）** ❌ |
| `0.20` | 0.2000 | 1.2116 rad | 1.2060（−0.47%） | **1.2066（−0.41%）** ✅ |

**注意**：`ros2 param set /mybot_diff_drive_controller wheel_separation ...`
**看起来成功，实际不生效**。实测把参数设成 0.10 / 0.40，由轮速反推的下发值都是
0.2000（不改）。要改必须重启控制器让它重新加载参数文件。

### Jazzy 参数坑（Humble → Jazzy，都带原始报错）

书上针对 Humble 写的 `nav2_params.yaml` 在 Jazzy 里会连环报错，按下面顺序改：

| # | 现象（原始报错） | 原因 | 改法 |
| --- | --- | --- | --- |
| 1 | `planner_server: Failed to create global planner. ... the class nav2_navfn_planner/NavfnPlanner ... does not exist` | 插件类名旧写法 `包/类` | 全部改成 `包::类`（如 `nav2_navfn_planner::NavfnPlanner`、`nav2_behaviors::Spin`） |
| 2 | `bt_navigator: Failed to create navigator id navigate_to_pose. Exception: ID [ComputePathToPose] already registered`，随后 `component_container` 段错误退出 | 书里手动列了全部内置 BT 节点，而 Jazzy 已自动注册，重复注册 | 把 `plugin_lib_names` 整块**注释掉** |
| 3 | `Expected 'value' to be one of [float, int, str, bool, bytes], but got '()' of type 'tuple'` | 上一步写成了 `plugin_lib_names: []`，launch 归一化时把空列表变成空元组 | 整行注释掉，别留空列表 |
| 4 | `collision_monitor: Error while getting parameters: parameter 'observation_sources' is not initialized`，bringup 中止 | `collision_monitor` 是 **Jazzy 新无条件启动**的节点，书里没有它的配置段 | 补上官方默认的 `collision_monitor` 段 |
| 5 | `docking_server: could not create publisher: ... existing topic name rt/cmd_vel with incompatible type ... Twist_` → `Lifecycle node docking_server does not have error state implemented` | `docking_server` 同样是 Jazzy 新启动的；它要用 `Twist` 建 `cmd_vel` 发布者，而该话题已经是 `TwistStamped` | 补 `docking_server` 段并开 `enable_stamped_cmd_vel` |
| 6 | `docking_server: Charging dock plugins not given!` | 补了上一条但没给插件 | 补 `dock_plugins` + `simple_charging_dock`（取自官方默认） |
| 7 | 控制器只收 `TwistStamped`，Nav2 默认发 `Twist` → 车不动 | Jazzy `diff_drive_controller` 只接受 stamped | `enable_stamped_cmd_vel: True` 要加在 **5 处**：`controller_server`、`behavior_server`、`velocity_smoother`、`collision_monitor`、`docking_server` |
| 8 | — | 书上是单数 `progress_checker_plugin` | 改成 Jazzy 的复数 `progress_checker_plugins` |

`enable_stamped_cmd_vel` 是否存在，用二进制字符串确认过：
`controller_server` / `velocity_smoother` / `collision_monitor` / `opennav_docking` /
`nav2_spin_behavior` / `nav2_back_up_behavior` / `nav2_drive_on_heading_behavior` /
`nav2_assisted_teleop_behavior` / `nav2_wait_behavior` 都有。

### Nav2 的速度链（改参数前必须知道）

```
controller_server ─┐
                   ├─ cmd_vel（启动时 remap 成 cmd_vel_nav）─┐
behavior_server  ──┘                                        │
                                                            ▼
                                              velocity_smoother
                                                            │
                                                   cmd_vel_smoothed
                                                            │
                                                            ▼
                                                 collision_monitor
                                                            │
                                                       cmd_vel  ← 机器人实际收
docking_server ───────────────────────────────────────────────┘（对接时才发）
```

所以 `/cmd_vel` 的**发布者不是 controller_server**，而是 `collision_monitor`
（`ros2 topic info /cmd_vel -v` 可以验证）。调试「车不动」时要从这条链的**末端**
往回查。

### 巡逻案例的适配（只动了 launch，没动教材的 .py）

| 适配 | 原因 |
| --- | --- |
| remap `/camera_sensor/image_raw` → `/camera/image` | 前者是 Gazebo classic 相机插件的话题名，我们 gz 桥出来的是后者 |
| 两个节点都注入 `use_sim_time: true` | 书上没设；仿真里不给 true，`BasicNavigator` 走系统时钟，等导航激活/超时都会异常 |
| `image_save_path` 固定到 `/workspace/my-slam/patrol_images/` 并 `mkdir -p` | 书上留空，`cv2.imwrite` 会写进进程当前目录；且**目录不存在时它只返回 False 不抛错，会静默丢图** |
| `speaker.py` 做「优雅降级」 | 本机没有 `espeak-ng` 也没有音频设备。改为：`import espeakng` 失败时只打日志，**服务名/类型/返回值都不变**。以后装了引擎自动生效，不用改代码 |

另外 `patrol_node.speach_text()` 里是 `while not wait_for_service(...)`，
**语音服务不在线会死等**，所以 `speaker` 必须跟着一起起（`autopatrol.launch.py` 已包含）。

### 怎么跑（Nav2 + 巡逻）

```bash
# 终端 1：仿真
cd /home/my/workspace/slam/my-slam && ./scripts/sim.sh --headless

# 终端 2：Nav2（无头；要看 rviz 去掉 rviz:=false）
./scripts/shell.sh -c 'ros2 launch mybot_navigation2 navigation2.launch.py rviz:=false'

# 终端 3：巡逻（拍照 + 语音服务）
./scripts/shell.sh -c 'ros2 launch autopatrol_robot autopatrol.launch.py'

# 停止（只停 Nav2 + 巡逻，保留仿真）
./scripts/stop-nav2.sh
```

> `./scripts/stop-nav2.sh` 存在的原因见「踩坑记录」：直接写 `pkill -f "navigation2.launch.py"`
> 有自杀风险。该脚本把 pkill 模式放进容器内的 `tools/stop-nav2-patrol.sh` 里规避。

**rviz 用的是本仓库裁剪过的配置**，不是 `nav2_bringup` 自带那份：

`mybot_navigation2/rviz/mybot_nav2.rviz`

官方默认那份（`nav2_default_view.rviz`）是给 TurtleBot3 配的，里面有两项订阅的是
我们根本没有的话题：

| 显示项 | 订阅的话题 | 情况 |
| --- | --- | --- |
| `Bumper Hit` | `/mobile_base/sensors/bumper_pointcloud` | TB3 防撞条，我们没有 |
| `Realsense` 组 | `/intel_realsense_r200_depth/*` | TB3 的深度相机，我们没有 |

挂着不会报错，但会让人以为是哪里没配好。裁剪版把这两项去掉，换成一个默认折叠的
`mybot Camera` 组（`/camera/image` + `/camera/points`），其余显示项
（Map / 全局与局部代价地图 / 路径 / 粒子云 / TF / RobotModel / LaserScan /
Global Planner / Controller / MarkerArray）原样保留。

`nav2_bringup` 升级后可以重新生成：

```bash
./scripts/shell.sh -c 'python3 /workspace/my-slam/tools/gen_nav2_rviz.py'
```

> **注意**：这个容器里 rviz2 **偶尔会在启动瞬间段错误退出**（`exit code -11`）。
> 实测与本配置无关 —— 同一负载下单独跑 50s 稳定、0 崩溃；官方默认配置在同一容器
> 里关闭时也会打 `terminate called without an active exception` 并 core dump。
> 遇到就重起一次，Nav2 本身不受影响。


### 实测数据（2026-10-01，aarch64 / Jazzy）

- Nav2 全部 lifecycle 节点 `active`，`ros2 action list` 有 `/navigate_to_pose`、`/follow_waypoints`
- 单点导航：目标 `map(1.0, 0.0)`，真值终点 `(0.849, −0.038)` → 误差 **0.156 m**（阈值 0.25 m），朝向误差 2.7°
- 巡逻 5 个点**全部成功**，到位误差均 ≤ 0.24 m：

| 目标点 | 真值到位 | 偏差 |
| --- | --- | --- |
| (0.0, 0.0) | (0.00, 0.00) | ~0 |
| (1.0, 2.0) | (0.89, 1.82) | 0.21 m |
| (−4.5, 1.5) | (−4.69, 1.44) | 0.20 m |
| (−8.0, −5.0) | (−7.80, −5.05) | 0.21 m |
| (1.0, −5.0) | (1.21, −4.88) | 0.24 m |

拍照落盘在 `my-slam/patrol_images/`，文件名按到位坐标生成（如 `image_-4.69_1.44.png`）。

### 示例节点：逐个实测过（不是只「编译通过」）

| 节点 | 目标 | 真值终点 | 结果 |
| --- | --- | --- | --- |
| `mybot_application init_robot_pose` | 设初始位姿 (0,0,0) | — | ✅ AMCL 随即开始发 `map→odom` |
| `mybot_application nav_to_pose` | map(1,1) | (1.24, 1.30) | ✅ SUCCEEDED |
| `mybot_application waypoint_follower` | (0,0)→(2,0)→(2,2) | (2.17, 1.88) | ✅ SUCCEEDED |
| `mybot_application get_robot_pose` | 只读 TF 位姿 | — | ✅ 正常打印（顺带验证了 `tf_transformations` 可用） |
| `mybot_application_cpp nav2pose` | map(2,2) | (2.27, 1.97) / 修orientation后 (2.22, 1.90) | ✅「处理成功」（修的是非法四元数，精度取决于容差，见下） |
| `autopatrol_robot patrol_node` + `speaker` | 5 个巡逻点 | 见上表 | ✅ 5/5 成功 |

这些例子都要 `use_sim_time`，例如：

```bash
./scripts/shell.sh -c 'ros2 run mybot_application nav_to_pose --ros-args -p use_sim_time:=true'
```

**入口注册**：`mybot_application/setup.py` 原本只注册了 `init_robot_pose`，
另外 3 个例子用 `ros2 run` 根本找不到（属于漏注册）。
本仓库把 4 个都注册上了。

**`nav2pose.cpp` 的非法四元数（已修）**：它设置目标点时**没有设
`orientation.w`**，发出去的四元数是 `(0,0,0,0)` —— 模长为 0，不是单位四元数，
属于非法旋转。本仓库补上了 `goal_msg.pose.pose.orientation.w = 1.0;`。

> 诚实说明：**修完行为没有可测量的变化**。修之前终点朝向 0.2388 rad、修之后
> 0.2450 rad，都在 `yaw_goal_tolerance = 0.25` 的边缘 —— 说明 Nav2 本来就把
> `(0,0,0,0)` 当成 yaw=0 处理了，**最终朝向是由容差决定的，不是由目标四元数
> 决定的**。所以这是一处「消除非法值、避免依赖未定义行为」的正确性修复，
> 不要期待它提升精度。真要提升到位精度，得收紧
> `xy_goal_tolerance`/`yaw_goal_tolerance`，同时提高 AMCL 收敛质量。

**关于定位精度**：上面几次的「真值终点 vs 目标点」误差在 0.16~0.38 m 之间，
和 AMCL 的收敛程度、地图栅格精度（0.05 m）都有关系，不是导航链路的问题。
要复核请用 `tools/set_initial_pose.py --from-gz` 先对齐真值再发目标点。

**为什么有时候会「绕远路」（不是 bug）**：实测发目标 `(2.17,1.88) → (-4.5,1.5)`，
直线只有 **6.68 m**，但 Nav2 报的 `distance_remaining` 是 **18.17 m**。
用 `tools/map_clearance.py` 沿直线逐点算「到最近墙的距离」后原因清楚了：

```
采样点            占用值   到最近墙(m)   在膨胀区内
(-0.69, 1.72)     254        0.47        是
(-1.64, 1.66)     254        0.35        是
(-2.12, 1.64)     205        0.05        是   <- 关键：离障碍只有 5cm
```

> ⚠️ 关于「占用值」：`room.yaml` 是 `mode: trinary`，`occupied_thresh=0.65`、
> `free_thresh=0.25`，`map_server` 按 `occ = (255 - pixel)/255` 判断：
> `pixel=0` → occ=1.0 → **障碍**；`pixel=205` → occ=0.196 → **自由**；
> `pixel=254` → occ=0.004 → **自由**。
> **205 不是「未知区域」**（本图里几乎没有未知格）—— 之前这里写成未知，已更正。
> 上表里的「到最近墙」是到 `pixel<=100` 的最近**障碍**格的距离，所以那一行说明的是
> 「这个采样点本身是自由格，但离障碍只有 5 cm」。
> 判据搞错会凭空造出墙来：把 205 当成障碍去算连通性，会得出「目标不可达」的错误结论。
> 正确的判据见 `tools/map_reachability.py` 的说明。

直线在 `x≈-2.12` 处离墙只有 **0.05 m**（那格还是未知区域 205），而机器人半径
0.22 m、`inflation_radius` 0.55 m —— 这个缝根本过不去，规划器只能绕。
再加上 navfn 默认 `use_astar: false`，它是**按代价最小**而不是距离最短找路，
所以给出 18 m 的方案是正常行为。

要让它更愿意走直线，可以（改前先确认车真能过那条缝）：
- 调小 `inflation_radius`（0.55 → 0.3 左右）或 `cost_scaling_factor`
- 或把 `planner_server` 里 `use_astar` 改成 `true`

排查手法：

```bash
python3 tools/map_clearance.py --from 2.17 1.88 --to -4.5 1.5
```


### 一次「Nav2 反复 Failed to make progress，恢复次数耗尽后 ABORT」的调查（2026-10-01）

**现象**：发目标 `(-4.5, 1.5)` 时，Nav2 反复报 `Failed to make progress` →
`[follow_path] [ActionServer] Aborting handle`，`number_of_recoveries` 涨到 13~15 后
以 `error_code: 105` ABORT，车最后停在 `(-8.88, -0.55)` 不动。

**结论：这不是确定性 bug，是「临界 + CPU 争抢」导致的概率性失败。**
同一份起点/目标，实测**既出现过 5 次恢复就成功（`error_code: 0`，距目标 0.23 m）**，
**也出现过 15 次恢复后 ABORT**。所以不要去某一行配置里找「错的那一句」。

逐项排除（每条都有实测证据，不是推断）：

| 怀疑对象 | 怎么验的 | 结论 |
| --- | --- | --- |
| 车被物理卡住 | 在它停下的位置手动发 `0.25 m/s`，车走了 **1.16 m**；激光正前方 2.02 m 无障碍 | ❌ 排除 |
| AMCL 定位飘了 | 录 `/amcl_pose` 与 `gz model -p` 真值逐点对比，全程偏差 **1~3 cm** | ❌ 排除 |
| `collision_monitor` 把速度压成 0 | 它全程只打了启动横幅，之后再无任何消息 | ❌ 排除 |
| 速度链上某级吃掉指令 | 同时录 `/cmd_vel_nav`、`/cmd_vel_smoothed`、`/cmd_vel`，三级**完全一致**（都是 47% 非零、范围 `[0, 0.26]`） | ❌ 排除 |
| 目标几何上不可达 | `tools/map_reachability.py`：最宽路线最窄处半径 **0.750 m** ≫ 车半径 0.22 m | ❌ 排除 |
| （对照）目标真不可达 | `(4.5, 1.5)` 最窄处只有 0.112 m（那格是 `cafe_table_1`），脚本正确报「不可达」 | ✅ 方法自检通过 |
| 仿真慢导致 10 秒窗口不够 | 实测 RTF ≈ **1.00**；`use_sim_time` 全为 `True`；progress checker 两个参数都没设 = 默认 10 s / 0.5 m | ❌ 排除 |

**`Failed to make progress` 本身报得没错**，它是症状不是原因。把报错时刻和真实位置对齐：

| 第几次报错 | 10 秒前位置 | 报错时位置 | 10 s 位移 |
| --- | --- | --- | --- |
| 1 | (-8.689, -0.637) | (-8.847, -0.575) | 0.16 m |
| 2 | (-8.847, -0.575) | (-8.894, -0.552) | 0.05 m |
| 3 | (-8.894, -0.552) | (-8.891, -0.553) | 0.00 m |
| 4 | (-8.891, -0.553) | (-8.883, -0.556) | 0.01 m |

确实没动，判定合理。真正要解释的是「它为什么在 `x≈-8.7` 处开始减速」。

**唯一站得住的相关因素：CPU 争抢。** 同期 Nav2 自己报了：

```
[controller_server]: Control loop missed its desired rate of 20.0000 Hz.
                     Current loop rate is 5.7471 Hz.
[planner_server]:    Planner loop missed its desired rate of 20.0000 Hz.
                     Current loop rate is 1.3605 Hz.
```

控制环只有配置值（20 Hz）的 1/3~1/4，DWB 的轨迹采样与打分质量必然下降。
这次是在 Gazebo + rviz + Nav2 同时开的 aarch64 机器上跑的。

**建议**：
- 跑 Nav2 相关验证时不要同时开 rviz / Gazebo GUI，先把 CPU 让出来；
- 要复现/观察这类问题：`bash tools/run-stuck-repro.sh`
  （瞬移回起点 + 按真值对齐 AMCL + 同时录整条速度链，一遍看完）。
- **不要**为了这个去改书上的 Nav2 参数：参数本身没问题，是资源不够。

### 还没做的（诚实清单）

| 项 | 状态 |
| --- | --- |
| 定位/导航/巡逻全链路 | ✅ 已实测 |
| 5 个包搬入 | ✅ 已实测 |
| `mybot_description` 两处修正 | ✅ 已实测（轮距带了 A/B 证据） |
| 语音发声 | ⚠️ 只打日志（本机无 `espeak-ng`、无音频设备，属于环境限制，不是遗漏） |
| rviz2 界面 | ⚠️ 全程无头验证，**没有目视检查过 GUI** |
| git 提交 | ⚠️ 改动都还没提交 |


### 一个值得知道的现象：里程计会漂移

`mybot_ros2_controller.yaml` 里 `open_loop: true`（书上的设置）。它的含义是
**里程计按「指令速度」积分，不看轮子实际转了多少**。仿真里原地转向时轮子会和
地面打滑，这部分打滑里程计完全看不到。实测巡逻一圈后：

```
/odom   (3.58, 6.16)  yaw ≈ −168°
真值    (4.57, −1.17) yaw ≈  154°
```

差了 7 m / 38°。**这不是 bug**，真机也一样 —— 这正是必须要 AMCL/建图的原因。
本项目里 AMCL 把它纠正回来了，所以导航到位误差仍只有 0.2 m。
想直观感受这一点，可以在开了 Nav2 之后对比 `tf2_echo map base_footprint`
（已纠正）和 `ros2 topic echo /odom`（未纠正）。

### 本轮新增的工具

| 工具 | 用途 |
| --- | --- |
| `tools/verify_wheel_separation.py` | 用 Gazebo 真值验证轮距配置对不对 |
| `tools/probe_wheel_speed.py` | 由轮速反推控制器**实际**下发的参数值（判定「参数是否真的生效」） |
| `tools/set_initial_pose.py` | 设 AMCL 初始位姿；`--from-gz` 直接对齐真值 |
| `tools/check_map_point.py` | 查地图上某点是否可走（下目标点前先确认，避免误判导航失败） |
| `tools/check_topic_health.sh` | 检查传感器话题健康：发布者数必须为 1 + 频率是否达标（切换 `sensor_bridge` 后必跑） |
| `tools/map_reachability.py` | 判断「A 能不能走到 B」+ 算最宽路线的最窄处半径（占用判据按 trinary 阈值，别再自己写） |
| `tools/probe-localization-vs-truth.sh` | 同时录 AMCL 位姿和 Gazebo 真值，用来区分「物理被挡」和「定位飘了」 |
| `tools/probe_cmd_chain.py` | 同时录 Nav2 速度链三级话题，定位「谁把速度清零了」 |
| `tools/analyze_cmd_chain.py` | 汇总上面那份记录（按 10 秒分桶看哪一级先变 0） |
| `tools/run-stuck-repro.sh` | 一键复现「卡住 → 恢复耗尽 → ABORT」并记录全过程 |
| `tools/stop-nav2-patrol.sh` / `stop-nav2.sh` | 安全停止 Nav2 + 巡逻 |
| `stop-nav2.sh` | 上面脚本的宿主机入口 |

### 回归清单（每次移植完都跑一遍）

| 检查 | 期望值 | 命令 |
| --- | --- | --- |
| 传感器内容自洽 | `exit=0` | `./scripts/shell.sh -c 'python3 /workspace/my-slam/tools/check_sensor_msgs.py; echo exit=$?'` |
| 无自激桥 | `0` | `grep -c "Creating ROS->GZ Bridge" 启动日志` |
| 各话题发布者唯一 | 都是 `1` | `./scripts/shell.sh -c 'ros2 topic info /scan \| grep "Publisher count"'` |
| `/scan` 频率 | ~10 Hz | `./scripts/shell.sh -c 'timeout 15 ros2 topic hz /scan'` |
| `/imu` 频率 | ~100 Hz | `./scripts/shell.sh -c 'timeout 8 ros2 topic hz /imu'` |
| **传感器话题健康（一条命令覆盖上面三项）** | `exit=0` | `./scripts/shell.sh -c 'bash /workspace/my-slam/tools/check_topic_health.sh; echo exit=$?'` |
| 遥控仍可用 | `/odom` 有变化 | `./scripts/teleop.sh --forward`，然后看 `/odom` |
| `/cmd_vel` 类型 | `TwistStamped` | `./scripts/shell.sh -c 'ros2 topic info /cmd_vel'` |

### 踩坑记录

**1. `pkill -f` 会杀掉自己（踩过两次）。**
`docker compose exec ... bash -lc 'pkill -f "navigation2.launch.p[y]"; ros2 launch ... navigation2.launch.py'`
—— 模式 `navigation2.launch.p[y]` 能匹配到**同一条命令行后半段**的字面量，于是
pkill 把自己的 shell 也杀了，表现为「命令什么都没输出就退出了」。
之前的 `pkill "ruby.*gz"` 同理。
**解法**：把 pkill 模式放进脚本文件（`tools/stop-nav2-patrol.sh`），
杀进程那条命令行里只出现脚本路径，不可能自匹配。

**2. `ros2 topic hz` 别接 `head`。**
`timeout 12 ros2 topic hz /scan | head -1` 会因为管道提前关闭而输出
`topic [/scan] does not appear to be published yet`，看起来像话题挂了，
其实 `/scan` 好好的（`ros2 topic info /scan` 显示 Publisher count: 1，实测 9.99 Hz）。
要么不接 `head`，要么用 `ros2 topic echo --once` 验证。

**3. 容器里默认用户是 `nvidia` 不是 root。**
装系统包要 `docker compose exec -u root ...`（走 docker，不用 sudo，也不会弹密码）。

**4. `packages.ros.org` 极慢。**
实测本机 ROS 索引 2.0 MB / 140 s（14 KB/s），而 `mirrors.ustc.edu.cn` 约 5.8 MB/s
（快约 400 倍）；Ubuntu ports 官方源也慢。`Dockerfile` 里新增的依赖层默认走国内
镜像，可用 `--build-arg APT_USE_CN_MIRROR=0` 切回官方源。

**5. AMCL 的 `Failed to transform initial pose in time` 是噪声，可忽略。**
设初始位姿时 AMCL 几乎必打这一行：

```
[amcl]: Failed to transform initial pose in time (Lookup would require
extrapolation into the future. Requested time 1923.414000 but the latest
data is at time 1923.405000, when looking up transform from frame
[base_footprint] to frame [odom])
```

原因是 `odom->base_footprint` 这条 TF 比仿真时钟滞后约 0.2~0.3 s（`/odom` 是
50 Hz，但 TF 的时间戳落后于 `/clock`）。**但它不影响位姿生效**，实测三种时间戳
策略后 `/amcl_pose` 与真值的差分别是：

| 时间戳策略 | 与真值偏差 |
| --- | --- |
| `now - 0.2s` | 0.016 m |
| `now - 0.3s` | 0.053 m |
| `0`（tf2 取最新） | 0.036 m |

而且设定前 AMCL 一直在打 `AMCL cannot publish a pose or update the transform.
Please set the initial pose...`（= 没有位姿），设定后立刻正常发布 `map→odom`，
证明位姿确实被采纳了。**不要为了消这行日志去调时间戳，那是白费功夫** ——
`tools/set_initial_pose.py` 已经把这个结论写在注释里了。

**6. 从 PGM 行号反推 y 时方向搞反，整张图在 y 上被镜像了（本仓库踩过）。**
PGM 的**首行对应 y 最大值**，所以 `row = h - 1 - int((y - oy)/res)`。
如果遍历时按这个公式取行、却用 `y0 + i*res` 去标注行号，标签和真实 y 就差了
一个镜像。这次因此把「障碍在 y≈0.9」当成了事实（真实位置在 y≈−5.2），
还据此写了一整段推理。
**解法**：不要手写扫描脚本，用 `tools/map_reachability.py` / `tools/map_clearance.py`
（这两个的 `rc()` / 行列换算都验证过）。真要自己写，先拿一个已知点回代验证。

**7. 占用判据必须和 `map_server` 一致，否则会凭空造出墙（本仓库踩过）。**
`room.pgm` 里只有三种像素：`0`、`205`、`254`。按 `room.yaml` 的 trinary 阈值
（`occupied_thresh=0.65`、`free_thresh=0.25`，`occ=(255−pixel)/255`）：

| pixel | occ | 判定 | 本图占比 |
| --- | --- | --- | --- |
| 0 | 1.000 | **障碍** | 3463 px |
| 205 | 0.196 | **自由** | 3157 px |
| 254 | 0.004 | **自由** | 76852 px |

**205 是自由空间**。把它当障碍，地图上会多出一大片不存在的墙，
于是「目标不可达」「这里过不去」这类结论全是假的 —— 这次就这样误判过一轮，
甚至据此怀疑是上游 Nav2 的 bug。
**解法**：判禁用 `pixel <= 100`（等价于 occ>0.65），或直接用 `tools/map_reachability.py`。

### 自定义规划器 / 控制器插件

搬入两个插件包：`nav2_custom_planner`、`nav2_custom_controller`
（这批代码里的 `mybot_description` 仍是旧值 —— 轮距 0.17、协方差全 0、雷达
5 Hz；轮距此前已按 URDF 修正过，这里又回退了，我们按 URDF 证据保持 0.20 不变）。

#### 修了 4 处 Jazzy 不兼容（都带原始报错）

| # | 报错 / 问题 | 原因 | 改法 |
| --- | --- | --- | --- |
| 1 | `fatal error: nav2_core/exceptions.hpp: No such file or directory` | Jazzy 把这个头拆成了 `planner_exceptions.hpp` / `controller_exceptions.hpp` / `smoother_exceptions.hpp` / `route_exceptions.hpp` | planner 用 `planner_exceptions.hpp`，controller 用 `controller_exceptions.hpp` |
| 2 | `invalid new-expression of abstract class type 'CustomPlanner'`：`createPlan` 未覆盖纯虚 | Jazzy 的 `GlobalPlanner::createPlan` 多了 `std::function<bool()> cancel_checker` 参数 | 补上该参数，并在生成路径的循环里用它支持中途取消 |
| 3 | 隐藏 bug（不报错，但危险） | 书里的**控制器**抛的是 `nav2_core::PlannerException` | 实测 `libcontroller_server_core.so` 里只出现 `nav2_core::ControllerException`、`libplanner_server_core.so` 里只出现 `PlannerException` → 控制器抛错了不会被接住，改成 `ControllerException` |
| 4 | 插件 XML 类名 | `custom_planner_plugin.xml` 用 Humble 的 `nav2_custom_planner/CustomPlanner`；`nav2_custom_controller.xml` 甚至没写 `name` 属性 | 统一改成 `包名::类名`（`nav2_custom_planner::CustomPlanner`、`nav2_custom_controller::CustomController`） |

#### 默认不替换插件（刻意选择）

原始实现把 `FollowPath` 的 DWB 和 `GridBased` 的 navfn **直接换成**了自研插件。
但那个自研规划器是**起终点直线插值、不绕障**，控制器也只是朝目标直行、限速 0.1 m/s——
换上去会让已验证的巡逻/导航直接降级。

所以本仓库的做法是：**默认保持 DWB + navfn**，在 `nav2_params.yaml` 里把自研插件的
配置写成带说明的注释块，想体验时改一行即可（`FollowPath` 和 `GridBased` 两处）。

#### 实测结果（用临时参数文件跑，不改仓库配置）

| 检查 | 结果 |
| --- | --- |
| 插件加载 | `Created controller : FollowPath of type nav2_custom_controller::CustomController`<br>`Created global planner plugin GridBased of type nav2_custom_planner::CustomPlanner` |
| lifecycle | 全部 `active`，0 个 ERROR/FATAL |
| 端到端 | 目标 `map(2.60, 0.90)` → 真值终点 `(2.580, 0.974)`，**误差 0.077 m**，`SUCCEEDED` |
| 回归 | 恢复 DWB/navfn 后参数文件仍合法、Nav2 正常启动 |

复现方式（不动仓库配置）：

```bash
# 造一份临时参数：把两个 plugin 行换成自研插件并补上它们的参数
./scripts/shell.sh -c 'python3 /workspace/my-slam/tools/make_custom_plugin_params.py /tmp/nav2_custom_test.yaml'
# 用它启动
./scripts/shell.sh -c 'ros2 launch mybot_navigation2 navigation2.launch.py rviz:=false params_file:=/tmp/nav2_custom_test.yaml'
```

### bringup（真机启动 + 仿真版一键启动）

#### 先说结论：这份启动文件是**真机专用**的

`mybot_bringup` 的 `launch/bringup.launch.py` 启动的是：

| 节点 | 作用 | 我们有没有 |
| --- | --- | --- |
| `ydlidar` | 实体 YDLIDAR 雷达驱动 | ❌ 没有这个包 |
| `micro_ros_agent` | 和单片机（ESP32）通信 | ❌ 没有这个包 |
| `ros_serial2wifi` | 串口转 WiFi 桥 | ❌ 没有这个包 |
| `odom2tf` | 把 `/odom` 转成 TF | ✅ 本包自带 |
| `urdf2tf` | robot_state_publisher 起 URDF | ✅ 有等价物 |

所以直接跑它必然是失败的，实测原始报错：

```
[ERROR] [launch]: Caught exception in launch ...
 - PackageNotFoundError: "package 'ydlidar' not found, searching: [...]"
```

**这份文件保持原样**（只修了下面那个 bug），真机用。

#### 修了它自身的一个真 bug

`bringup.launch.py` 里变量定义成 `ros_serial2wifi`，但在 `LaunchDescription`
列表里写成了 `ros_serail2wifi`（**字母顺序写反**）——这会让 launch 一启动就抛
`NameError`。已改正为 `ros_serial2wifi`。

#### 刻意没有搬 / 没有用的东西

| 项 | 为什么不搬 |
| --- | --- |
| 单文件版 `mybot_description` | 里面只有一个单体 `urdf/mybot.urdf`，是本仓库 xacro 版的**子集** |
| 这批里的 `maps/room.pgm` | 实测 **152×103 px**（7.6×5.15 m，origin `-3.9,-1.82`），而仿真用的那份是 **376×222 px**（18.8×11.1 m，origin `-10.4,-6.53`）——**完全不同的区域**，是作者真机所在的小场地，换上去仿真里的目标点全对不上。两份的 `nav2_params.yaml` 倒是**逐字节相同**。 |
| `odom2tf` 在仿真里启动 | 仿真里 `odom→base_footprint` 这条 TF 已经由 `mybot_diff_drive_controller`（`enable_odom_tf: true`）在发，再让 `odom2tf` 发一遍同一条变换会出现两个发布者互相打架。真机上里程计不发 TF，才需要它。 |

#### 补了一个「仿真版一键启动」

这部分的核心价值是**一条命令拉起整个机器人**。真机版在这里跑不了，所以加了
`mybot_bringup/launch/bringup_sim.launch.py`：一条命令起「Gazebo 仿真 +
Nav2 + 设置初始位姿」，把原来要开三个终端的流程收成一个。

```bash
# 容器内
ros2 launch mybot_bringup bringup_sim.launch.py
ros2 launch mybot_bringup bringup_sim.launch.py headless:=false rviz:=true
ros2 launch mybot_bringup bringup_sim.launch.py initial_pose:=false
```

参数：`headless`(默认 true)、`rviz`(false)、`initial_pose`(true)、
`nav2_delay`(12 s，等机器人生成)、`pose_delay`(30 s，等 Nav2 激活)。

实测（一条命令，headless）：**0 个 ERROR**，`ruby`(gz sim) /
`parameter_bridge` / `gz_sensor_bridge` / `robot_state_publisher` /
`component_container`(Nav2) 全部在跑，Nav2 两个 lifecycle manager 都
`Managed nodes are active`，`map→base_footprint = [0,0,0]` 与刚起仿真的机器人
真值一致。

> `initial_pose` 走的是教材的 `init_robot_pose`，固定发 `(0,0,0)`，只适合
> 「刚起仿真、机器人还在原点」。机器人已经被开走了就用
> `tools/set_initial_pose.py --from-gz`。

#### 顺带补的运维脚本

`./scripts/stop-sim.sh`（容器内 `tools/stop-sim.sh`）——安全停掉仿真。之前停仿真只能靠
跑 `sim.sh` 的那个终端 Ctrl-C，终端关了就没辙。同样把 pkill 模式放在脚本文件里，
避免自匹配。

#### 一个容易踩的坑：`--symlink-install` 新增文件也要重编

`--symlink-install` 只保证**修改**已有文件不用重编；**新增**文件（比如新写的
launch）必须再 `colcon build` 一次，否则 install 目录里没有它，运行时报：

```
file 'bringup_sim.launch.py' was not found in the share directory of package 'mybot_bringup'
```

### 学习型示例包

直接搬入（都是独立的演示程序，和前面的机器人主线没有耦合，没有同名包冲突）：

`learn_compose`（**注意它不是 docker compose**，是同进程零拷贝通信演示）、
`learn_dds_cpp`、`learn_executor_cpp/_py`、`learn_lifecyclenode_cpp/_py`、
`learn_message_filter_cpp/_py`、`learn_qos_cpp/_py`。

#### 需要补的依赖

| 包 | 原因 |
| --- | --- |
| `ros-jazzy-example-interfaces` | `learn_executor_cpp` 用了 `example_interfaces/srv/AddTwoInts`。不加会报 `Could not find a package configuration file provided by "example_interfaces"` |

已加进 `Dockerfile` 的依赖层（和 `tf-transformations` 同一层）。

#### 修了 1 处 Jazzy 弃用

`learn_executor_cpp` 里 `create_service` 传的是 rmw 层的 `rmw_qos_profile_services_default`，
这个重载在 Jazzy 已弃用：

```
warning: ... create_service(...) is deprecated:
         use rclcpp::QoS instead of rmw_qos_profile_t
```

改成 rclcpp 层的 `rclcpp::ServicesQoS()`（语义等价）。改完整个工作区
**22 个包零警告编译通过**。

#### 逐个实测（都跑过）

| 程序 | 现象 |
| --- | --- |
| `learn_compose intra_process_pubsub` | talker/listener 打印的**数据指针完全相同**（如 `0xB828CEC16E70`）→ 同进程零拷贝生效 |
| `learn_dds_cpp shm_pub` | `loaned_message_publisher` 持续发布（该包只有 publisher 源码，没有 subscriber） |
| `learn_executor_cpp/_py learn_executor` | 定时器持续发布，C++ 版打印线程 ID、Python 版还打印线程总数 |
| `learn_qos_cpp/_py reliability_test` | `odom_publisher_subscriber` 持续收到里程计消息 |
| `learn_message_filter_cpp/_py timesync_test` | 打印同步后的时间戳（C++ 用 imu、Python 用 odom） |
| `learn_lifecyclenode_cpp/_py learn_lifecyclenode` | 实测生命周期切换：`unconfigured → configure → inactive → activate → active`，回调日志对应 `on_configure()/on_activate()` |

```bash
# 例：跑生命周期演示并观察状态切换
./scripts/shell.sh -c 'ros2 run learn_lifecyclenode_py learn_lifecyclenode'   # 终端 1
./scripts/shell.sh -c 'ros2 lifecycle get /lifecyclenode'                     # 终端 2
./scripts/shell.sh -c 'ros2 lifecycle set /lifecyclenode configure'
```

> 这些 Python 演示被 `timeout`/Ctrl-C 杀掉时会打印
> `rclpy.executors.ExternalShutdownException` 的 traceback —— 这是 rclpy 收到
> SIGTERM 后的**正常**表现（节点内部没有捕获它），不是程序有 bug。





## 启动

```bash
cd /home/my/workspace/slam/my-slam
./scripts/start.sh
```

首次启动会拉取 `ros:jazzy-ros-base` 并安装桌面、Gazebo、SLAM、Nav2 等依赖，再构建
`slam-ros2-dev:jazzy`，耗时取决于网络和 Docker 缓存。

## 构建网络记录

截至 **2026-09-28**，当前 Dockerfile **没有默认使用国内镜像**，APT 使用：

- Ubuntu ARM64：`ports.ubuntu.com/ubuntu-ports`
- ROS 2：`packages.ros.org/ros2/ubuntu`

首次构建实际下载约 836 MB，在当前机器上约耗时 51 分钟；后续启动会复用本地镜像缓存，不应重复下载这层。

容器用户名和 UID/GID 参数定义在依赖安装层之后，修改用户配置不需要重新安装 ROS。
Dockerfile 前部保留首次构建的旧参数默认值，用于复用已完成的依赖层缓存。

已验证可用的国内索引：

- Ubuntu ARM64：阿里云、清华大学的 `ubuntu-ports`
- ROS 2 Jazzy：阿里云、清华大学、中科大、南京大学的 `ros2/ubuntu` `noble` 索引

ROS 2 软件源按 Ubuntu 发行版命名，Jazzy 对应 `noble`，不是
`ros2/ubuntu/dists/jazzy`。当前配置暂不自动切换源，以保持已经构建好的镜像和默认环境稳定；
如需切换，应在重新构建前明确指定并重新验证完整依赖下载。

## 常用入口

所有入口都在**宿主机**上执行，脚本内部自己 `docker compose exec` 进容器；宿主机上不需要装
任何 ROS。workspace overlay（`/workspace/my-slam/install`）由入口脚本自动 source。

| 命令 | 作用 |
| --- | --- |
| `./scripts/start.sh` / `./scripts/stop.sh` | 启动 / 停止容器 |
| `./scripts/shell.sh` | 交互式进入容器（推荐用这个） |
| `./scripts/sim.sh` | 一键启动 mybot 的 Gazebo Sim 仿真 |
| `./scripts/teleop.sh` | 键盘遥控 mybot |
| `./scripts/rviz2.sh` / `./scripts/rqt.sh` | 图形化调试 |
| `./scripts/gazebo.sh` | 只开一个空的 Gazebo GUI（手动摆模型用） |
| `./scripts/gpu-check.sh` | 排查 GPU / 渲染问题（`failed to create drawable`） |

非交互式执行单条命令（不占终端，脚本里也能用）：

```bash
./scripts/shell.sh -c 'ros2 topic list'
./scripts/shell.sh -c 'ros2 pkg prefix mybot_description'
```

`./scripts/docker_run.sh`、`./scripts/docker_into.sh`、`./scripts/docker_stop.sh` 是早期的 Apollo 风格命名，现在只是
`start.sh` / `shell.sh` / `stop.sh` 的薄别名（`docker_into.sh` 原先和 `shell.sh` 内容完全重复）。
这些脚本只操作 Compose 项目 `slam-ros2-dev`，不会碰 OOMWOO 容器。

公共逻辑集中在 `scripts/container-exec.sh`（被上面这些 wrapper source），它负责三件事：
切到仓库根（`compose.yaml` 所在处）、stdin 不是终端时自动加 `-T`、优先使用 bind mount 进来的 `entrypoint.sh`
（所以改入口脚本立即生效，不用重建镜像）。

例如启动一个 Gazebo 世界：

```bash
./scripts/gazebo.sh /workspace/Dataset-of-Gazebo-Worlds-Models-and-Maps/worlds/empty_room/world.sdf
```

### 两个高频踩坑

**1. 不要在父目录执行 `docker compose`。**
`compose.yaml` 在 `my-slam/` 根下（`scripts/` 的上一级），在 `/home/my/workspace/slam` 里执行会直接失败：

```text
no configuration file provided: not found
```

所有 wrapper 都会自己 `cd` 到 `my-slam/`（`compose.yaml` 所在处），所以从任何地方调用 `./scripts/sim.sh`、`./scripts/shell.sh` 都安全。

**2. 交互式 shell 里的 overlay（已修）。**
镜像里烘焙的入口脚本原来写死 `source /workspace/install/setup.bash`，而真正的 overlay 在
`/workspace/my-slam/install/setup.bash` —— 前者从来不存在。后果是：
非交互式 bash 因为有 `BASH_ENV=ros-env.sh` 兜底，看起来正常；但交互式 shell（`./scripts/shell.sh`）里

```bash
ros2 pkg prefix mybot_description     # -> Package not found
```

现在 `entrypoint.sh` 改成自动发现 overlay（`/workspace/install` → `/workspace/*/install`），
进容器即可直接用，新增的包 `colcon build` 完立即生效。想确认实际 source 了哪些：

```bash
./scripts/shell.sh -c 'echo "$SLAM_OVERLAY_SETUP"'
```

## Gazebo 报 `failed to create drawable` 的排查

这个报错属于 **OpenGL/GLX 渲染问题**，和模型、资源路径无关（资源路径正确时日志里会显示
类似 `/home/nvidia/.gz/models` 的本地资源目录已加载）。本质原因是容器**没有任何 GPU 直通**
（没有 `/dev/nvidia*`、没有 `/dev/dri`、没有 NVIDIA 用户态 GL 库），只剩 llvmpipe 软件渲染，
而 llvmpipe 在 X11 下创建 GLX drawable 失败。

用 `./scripts/gpu-check.sh` 可以一次性把宿主机和容器内的状态都打出来。

### 本机实测结论（2026-09-30）

```text
宿主显卡      NVIDIA GB10 / driver 580.173.02 / aarch64
宿主 DISPLAY  :1（X11），/tmp/.X11-unix/X1 存在，Xauthority 可读  -> X11 侧正常
Docker        runtimes 里没有 nvidia，但存在 CDI spec /var/run/cdi/nvidia.yaml
容器内        无 nvidia-smi、无 /dev/dri、LIBGL_ALWAYS_SOFTWARE=1
```

即：**X11 和模型路径都是好的，唯一缺的是 GPU 直通。**

### 启用 GPU 直通（推荐）

本机已装 nvidia-container-toolkit 且生成了 CDI spec，直接用 CDI，不需要重启 Docker：

```bash
./scripts/stop.sh && ./scripts/start.sh      # start.sh 检测到 CDI 后自动叠加 compose.nvidia.yaml
```

或者显式指定：

```bash
docker compose -f compose.yaml -f compose.nvidia.yaml up -d --build
```

`compose.nvidia.yaml` 做了三件事：注入 `nvidia.com/gpu=all`（CDI）、
把 `LIBGL_ALWAYS_SOFTWARE` 置为 `0`、并用 `group_add` 补上 `/dev/dri` 渲染节点所需的组。

不要用 `runtime: nvidia`：本机 `docker info` 的 Runtimes 中没有 nvidia，
写上去会直接启动失败。若确实想用 runtime，先在宿主机执行
`sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker`。

验证（不重建现有容器）：

```bash
docker compose -f compose.yaml -f compose.nvidia.yaml run --rm --no-deps \
  --entrypoint bash slam-ros2-dev -lc 'ls /dev/dri /dev/nvidia0; nvidia-smi -L'
# GPU 0: NVIDIA GB10 (UUID: GPU-...)
```

### 模型缓存的持久化

Gazebo 模型默认下载在容器内的 `/home/nvidia/.gz`，容器一旦重建就会丢。
现在 compose 已把它 bind mount 到宿主机的 `my-slam/gz-cache/`，重建容器不会丢模型。

### X11 授权

`start.sh` 已经通过挂载 `XAUTHORITY_FILE` 传递 X11 cookie，正常情况下 **不需要**
`xhost +local:docker`。如果仍然报权限错误，可在宿主机执行
`xhost +si:localuser:$(id -un)`（比 `xhost +` 安全）。

### 只验证模型加载，不启动 GUI

```bash
./scripts/gazebo.sh -s -r /workspace/.../model.sdf   # -s: 只启动 server
```

### 其它提示

* 日志里 `Can not find the XML attribute 'version' in sdf XML tag` 只是警告，不影响加载。
* 只想验证模型能否加载时，先 `./scripts/gazebo.sh -s -r <world.sdf>`（`-s` 只起 server，不起 GUI），
  可以完全绕过 drawable 问题。

## mybot 的 Gazebo Sim 仿真（my-slam 新增，不改教材原文件）

书上的 `gazebo_sim.launch.py` 基于 **Gazebo classic**（`gazebo_ros` /
`spawn_entity.py`）。ROS 2 Jazzy 已经不再提供 `gazebo_ros`（classic 2025-01 EOL），
本镜像里装的是新一代 Gazebo Sim，所以另建了一套 `*_gz` 文件，与教材内容并存。

```bash
cd /workspace/my-slam
colcon build --symlink-install
source install/setup.bash
ros2 launch mybot_description gazebo_sim_gz.launch.py
```

### 新增文件

| 文件 | 作用 |
| --- | --- |
| `launch/gazebo_sim_gz.launch.py` | 用 `ros_gz_sim` 重写：`gz_sim.launch.py` + `ros_gz_sim create` + `ros_gz_bridge` + `controller_manager spawner` |
| `urdf/mybot/mybot_gz.urdf.xacro` | 总装文件，组件复用教材的，只换最后两个插件 |
| `urdf/mybot/plugins/gz_control_plugin.xacro` | `gz_ros2_control/GazeboSimSystem` + `libgz_ros2_control-system.so` |
| `urdf/mybot/plugins/gz_sensor_plugin.xacro` | `gpu_lidar` / `imu` / `rgbd_camera` 三个 gz 传感器 |
| `world/custom_room_gz.world` | 教材 `custom_room.world` 的副本，加了 4 个系统插件 |

### 与教材的关键差异

1. **`gazebo_ros` → `ros_gz_sim`**：`gz_sim.launch.py` 起仿真，`ros_gz_sim create`
   生成模型，gz 与 ROS 之间用 `ros_gz_bridge` 桥接。
2. **ros2_control 插件换名**：`gazebo_ros2_control/GazeboSystem` →
   `gz_ros2_control/GazeboSimSystem`，`libgazebo_ros2_control.so` →
   `libgz_ros2_control-system.so`。gz_ros2_control 会从 `/robot_description`
   **话题**里读 URDF（sdformat 转换时会把 `<ros2_control>` 丢掉，所以只能走话题），
   因此 `robot_state_publisher` 必须先起来。
3. **必须加载 `Sensors` 系统**：`gz sim` 默认的 `server.config` 只带
   Physics / UserCommands / SceneBroadcaster，**不含 Sensors**，缺它相机和雷达
   完全不工作。所以 `world/custom_room_gz.world` 里显式声明了这 4 个插件。
4. **cmd_vel 重映射修正**：教材写的 `cmd_vel_unstamped:=/cmd_vel` 在 Jazzy 上无效，
   Jazzy 的 `diff_drive_controller` 只订阅 `~/cmd_vel`（`use_stamped_vel` 参数已移除）。
5. `mybot_effort_controller` 定义了却没挂进事件链（死代码）；
   `diff_drive_controller` 用的是 velocity 接口，本移植不加载 effort 控制器。

### 话题对照

| ROS 话题 | 来源 |
| --- | --- |
| `/cmd_vel`（订阅，**类型是 `geometry_msgs/msg/TwistStamped`**） | 重映射到 `/mybot_diff_drive_controller/cmd_vel` |
| `/odom`、`/tf` | `diff_drive_controller`（gz_ros2_control 直接发 ROS 话题，无需桥接） |
| `/joint_states` | `mybot_joint_state_broadcaster` |
| `/scan`、`/scan/points` | `gpu_lidar`（`laser_link`） |
| `/imu` | `imu` 传感器（`imu_link`） |
| `/camera/image`、`/camera/depth_image`、`/camera/camera_info`、`/camera/points` | `rgbd_camera`（`camera_optical_link`） |
| `/clock` | Gazebo（供 `use_sim_time`） |

### 让 mybot 动起来

镜像里**没有装 `teleop_twist_keyboard`**，而且即使装了也不能直接用（它默认发
`Twist`，而本环境需要 `TwistStamped`）。零安装的驱动方式：

```bash
# 前进
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.2}}}"

# 边走边转
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.2}, angular: {z: 0.5}}}"

# 停（Ctrl-C 也可以）
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.0}}}"
```

实测：发 `x: 0.2` 后 `/odom` 的 x 增长到 0.91 m，`/joint_states` 轮速 6.25 rad/s
（= 0.2 / 0.032，正好是线速度除以轮半径）。

原因说明：Jazzy 的 `diff_drive_controller` 4.x 把 `~/cmd_vel` 的类型从 `Twist`
换成了 `TwistStamped`，并移除了 `use_stamped_vel` 参数 —— 教材 yaml 里那行
`use_stamped_vel: false` 现在是无效果的。发错类型时 `ros2 topic pub` 会一直打印
`Waiting for at least 1 matching subscription(s)...`，车不动。

### 关于「部分传感器话题桥接失败」——⚠️ **原结论已撤回**

> **2026-10-01 更正。** 这一节原本的结论是「上游 arm64 构建缺陷」，**该结论已撤回**。
> 在同一个容器里（未重建镜像、未换容器）用 `tools/probe-bridge-types.sh` 逐个场景重试，
> 那个失败**一次都没再出现**：
>
> | 场景 | 结果 |
> | --- | --- |
> | 单条桥 `/imu`，单向 `[` | ✅ 成功 |
> | 单条桥 `/imu`，双向 `@` | ✅ 成功（建出 2 条） |
> | 当时记录在案的「失败组合」`clock,clock,scan,spts,imu` | ✅ 5/5 成功 |
> | 原始的 9 条桥 + `--ros-args` remapping（当年失败的完整配置） | ✅ 9/9 成功 |
> | 7 条桥一次性传入 | ✅ 7/7 成功 |
>
> 2026-09-30 观察到的失败（有 run 日志）应该是真的，但**原因至今未定位、而且现在
> 无法复现**；当初把它归因于「上游构建缺陷」是**推测**，没有证据支持。
> 以后要再验证：`bash tools/probe-bridge-types.sh`（自带对照组与判读说明）。

**现状：两套实现都保留，用 launch 参数二选一**（2026-10-01 实测两者等价）：

```bash
./scripts/sim.sh --clean --headless -- sensor_bridge:=parameter_bridge   # 用内置桥
./scripts/sim.sh --clean --headless                                       # 默认：用自研桥
```

只影响 `/imu` 与 `/camera/*` 这 5 个话题；`/scan`、`/scan/points`、`/clock`
一律交给 `parameter_bridge`。**两者绝不能同时开** —— 同一话题会出现两个发布者，
频率翻倍、数据重复。

两组实测对照（同为 headless）：

| 项目 | `parameter_bridge` | `gz_sensor_bridge`（默认） |
| --- | --- | --- |
| 6 个传感器话题的发布者数 | 全 = 1 | 全 = 1 |
| `/scan` | 9.94 Hz | 9.97 Hz |
| `/imu` | 99.43 Hz | 99.60 Hz |
| `/camera/camera_info` | 9.94 Hz | 9.99 Hz |
| `/imu` 的 `frame_id` | `imu_link` | `imu_link` |
| `tools/check_sensor_msgs.py` | 退出码 0 | 退出码 0 |
| 启动日志里 ROS→GZ 桥 | 0 条 | 0 条 |

自研桥的定位是**「一个可用的替代实现」，不是「绕开上游 bug 的必要手段」** ——
原始理由已不成立；保留它是因为已经逐项验证、有测试覆盖，删掉没有收益。

实测结果（headless 仿真，2026-09-30）：

| 话题 | 频率 | 数据自洽性 | 谁负责（默认） |
| --- | --- | --- | --- |
| `/imu` | 100.2 Hz | `frame_id=imu_link`，静止时重力 9.71 m/s² | 自研桥 |
| `/camera/image` | 7.5 Hz | 800×600 `rgb8`，`len(data)==step*height` | 自研桥 |
| `/camera/depth_image` | 2.6 Hz | 800×600 `32FC1`，长度自洽 | 自研桥 |
| `/camera/camera_info` | 10.0 Hz | `plumb_bob`，K/P 非零且 cx/cy 居中 | 自研桥 |
| `/camera/points` | 1.5 Hz | `point_step/row_step/data` 长度自洽 | 自研桥 |
| `/scan` `/scan/points` | 10.0 / 10.3 Hz | — | `parameter_bridge` |
| `/clock` `/odom` `/tf` `/joint_states` | ~2000 / 50 / 38 / 100 Hz | — | `parameter_bridge` + 控制器 |

两边的 QoS 都是 `RELIABLE`（与 `parameter_bridge` 默认一致），所以 rviz / slam_toolbox /
nav2 这些可靠订阅者不会有兼容问题。

自检脚本：`tools/check_sensor_msgs.py`（检查图像/点云长度自洽、内参非零、重力值等，
退出码 0 = 全部通过）。

---

下面这段是**当时的取证过程，保留存档**。注意：其中的 gdb 结论（“传进去的 key 完全
正确，是工厂查找返回了 null”）依赖于 aarch64 ABI 取参（Release 构建无调试信息，从
`x0/x1` 读），**现在已知这种取参方式有误读风险**，所以这段只能当作“当时观察到什么”
的记录，不能当作结论。

**受影响**：`/imu`、`/camera/image`、`/camera/depth_image`、`/camera/camera_info`、`/camera/points`
**不受影响**：`/clock`、`/scan`、`/scan/points`、`/odom`、`/tf`、`/joint_states`

日志里的表现是：

```text
[WARN] [ros_gz_bridge]: Failed to create a bridge for topic [/imu] with ROS2 type
  [sensor_msgs/msg.Imu] to topic [/imu] with Gazebo Transport type [gz.msgs.IMU]:
  No template specialization for the pair
```

现象很误导人：`gz topic -i -t /imu` 能看到 gz 侧确实有发布者，但 `ros2 topic list` 里
根本没有 `/imu` 这个话题。

**最硬的证据：运行时探针（gdb）**

`ros_gz_bridge` 为每条桥调用 `get_factory(ros_type, gz_type)`，查不到就抛
`No template specialization for the pair`。用 `tools/gdb-get-factory.gdb` 打断点
把每次调用的两个字符串打出来（Release 构建无调试信息，按 aarch64 ABI 从 `x0/x1` 取参）：

```text
失败： [lookup] ROS=[sensor_msgs/msg.Imu]      GZ=[gz.msgs.IMU]        → 之后立刻 WARN
成功： [lookup] ROS=[sensor_msgs/msg.LaserScan] GZ=[gz.msgs.LaserScan] → 建桥成功
```

**传进去的 key 完全正确** —— `sensor_msgs/msg.Imu` + `gz.msgs.IMU` 是标准写法，
`strings` 也能在 `libros_gz_bridge.so` 里找到这两个类型名（各出现 1 次）。
也就是说：**参数没错、名字没错，是库内部的工厂查找本身返回了 null。**

**已复现/已排除的事实**（2026-09-30，ros_gz_bridge 1.0.24 / gz sim 8.15.0）：

| 实验 | 结果 |
| --- | --- |
| 单独桥接 `/imu`（同一容器内，不同时刻） | 曾出现 **成功 3/3**，也出现 **失败 3/3** —— 会漂移 |
| **重启容器后**立刻重测 `/imu` | **失败 3/3** —— 不是"容器跑久了才坏" |
| 同时测 `/scan` | 成功（稳定） |
| 把话题名换成 `/a` `/b` `/c` | 不改变结果 —— 与名字无关 |
| 把 `@` 换成 `[`（单向） | 不能解决 |
| `/scan`+`/scan/points`+`/imu` 三条一组 | imu 必失败，换顺序也一样 |
| `/dev/shm` 用量 | 64 MB 里只用了 4% —— 排除共享内存耗尽 |
| 残留进程 | 清理干净后同样复现 —— 排除进程干扰 |
| gdb 是否影响结果 | 会（gdb 默认关闭 ASLR）—— 说明结果对内存布局/时序敏感 |

**当时的结论（❗已于 2026-10-01 撤回）**：失败发生在 `libros_gz_bridge.so` 的工厂
查找内部（key 正确却查不到），并且结果会随运行时条件漂移。**这不是本仓库的模型/
world/传感器配置问题，靠调参数无法修好。**
本文件此处先后写过三版"原因"，都被后续实验推翻 —— 所以这里只列事实，不再写推测的机理。

> ⚠️ 「库内部查表返回 null」这个判断依赖按 aarch64 ABI 从 `x0/x1` 取参，
> 而 Release 构建没有调试信息，这种取参方式本身就有误读风险，**不能当作证据**。

**⚠️ 上面这些结论在 2026-10-01 复核时全部站不住了** —— 见本节开头的更正说明。
尤其是「单独一个 `/imu` 桥同样失败」这一条：今天在**仿真正常运行中**单独跑 `/imu`
桥是**成功**的（单向、双向都成功）。所以本节只能当作「当时观察到什么」的存档，
**不要再当作结论引用**。

因此，现在**没有可报上游的材料** —— 报 issue 的前提是能稳定复现，而目前复现不出来。
想推进的话，第一步是让 `tools/probe-bridge-types.sh` 重新报出失败（比如哪天它又出现了，
把那份输出存下来），而不是拿一份过时的推断去报。

**当时为什么决定改用自研桥**（2026-09-30 的判断，⚠️ **理由已失效，仅作存档**）：

| 当时的结论 | 依据 |
| --- | --- |
| 不是配置问题 | 参数、话题名、方向符、顺序都验过；gdb 显示传进去的类型字符串完全正确 |
| 不是文件损坏 | `dpkg -V` 无任何输出（文件与包元数据一致） |
| 不是版本旧 | 与清华镜像比对，`ros-jazzy-ros-gz-bridge` 已是 apt 源里最新的 arm64 版本 |
| 不是依赖装漏 | `ldd` 无 `not found`；`/dev/shm` 只用了 4% |
| ~~可能是 arm64 构建问题~~ | ⚠️ 这台机器是 aarch64，但**这是推测，没有证据，已撤回** |

当时据此认为“输入正确、库内部却返回 null，只能绕开它”。
**这个推断现在不成立**：2026-10-01 复核时同样的桥全部建得出来，也复现不出来。
所以自研桥现在的定位是**「一个可用的替代实现」**，两套实现由 launch 参数
`sensor_bridge` 切换，见本节开头。

> 原因**尚未定位**，这里不写任何推测的机理 —— 之前写过的两版"原因"都被后续实验推翻了。
> 复现脚本：`tools/exp-bridge-repro.sh`（容器内 `bash /workspace/my-slam/tools/exp-bridge-repro.sh 3 35`）。

对激光 SLAM / 建图导航来说主链路是够用的（`/scan` + `/odom` + `/tf` + `/clock`）；
IMU 与 RGB-D 相机是后面做 IMU 融合、视觉相关功能要用的 ——
现在两套实现都可用（见本节开头），已经不是阻塞项。

### 键盘遥控（推荐用本仓库的脚本）

`teleop_twist_keyboard` 在本环境有个坑：它用 `termios` 把终端切成 raw 模式
逐个读按键，**必须有真正的 TTY**。通过 `docker compose exec -T`、管道、后台或
非交互终端启动时，它会永远阻塞在读键上 —— 进程活着、publisher 也建好了，但一条
指令都发不出去，而且不打任何日志。实测现象：

```text
$ ros2 topic info /cmd_vel -v   -> Publisher count: 1  (teleop_twist_keyboard)
$ ros2 topic hz /cmd_vel        -> 完全没有数据
```

所以本包自带了一个不依赖 TTY 的版本，并配了宿主机入口 `./scripts/teleop.sh`
（和 `gazebo.sh`/`shell.sh` 一样，在**宿主机**执行，容器里什么都不用装）：

```bash
# 交互式终端：单键模式（w/s/a/d/k/q 都不用回车）
./scripts/teleop.sh

# 强制行模式（按完回车）
./scripts/teleop.sh --line

# 不读键盘，直接前进（验证链路用，Ctrl-C 停）
./scripts/teleop.sh --forward
```

⚠️ **注意路径属于哪一侧**（这是个很容易踩的坑）：

| 命令 | 在哪执行 | 说明 |
| --- | --- | --- |
| `./scripts/teleop.sh` | **宿主机** | 推荐，内部自动 `docker compose exec` 进容器 |
| `python3 /workspace/my-slam/.../mybot_teleop.py` | **容器内** | `/workspace/...` 只在容器里存在 |

在宿主机上直接跑 `python3 /workspace/my-slam/...` 会得到
`can't open file '/workspace/my-slam/...': [Errno 2] No such file or directory`
（退出码 2），因为 `/workspace` 是容器里的挂载点，宿主机上没有。

按键：`w` 前进 / `s` 后退 / `a` 左转 / `d` 右转 / `k` 或空格 停 / `q` 退出。
脚本以 10 Hz 持续重发最后的速度指令，避免 `cmd_vel_timeout: 0.5` 触发刹车。

`teleop.sh` 退出时（包括 Ctrl-C）会清理容器内的同名进程 —— 否则残留的 teleop
会继续按最后一条指令发布 `/cmd_vel`，表现成「没按键车却还在动」。

实测：`--forward` 让 `/cmd_vel` 稳定在 9.96 Hz；行模式喂 `w` 后 `/odom` 的 x
单调增长。

### 如果还是想用 teleop_twist_keyboard

先确认跑它的那个终端是真 TTY：

```bash
tty          # 必须输出 /dev/pts/N；输出 "not a tty" 就是这个问题
```

必须是交互式进入容器（`docker compose exec` 默认带 `-t`，但脚本里加了 `-T`
或用了管道就不行），并且把该终端窗口保持在前台：

```bash
# 容器内安装（一次性；容器重建后会丢）
sudo apt update && sudo apt install -y ros-jazzy-teleop-twist-keyboard

# 必须在真 TTY 的终端里跑；注意 Jazzy 只收 TwistStamped
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

仿真 + 遥控的推荐起法（两个终端，都在宿主机）：

```bash
# 终端 1
./scripts/sim.sh

# 终端 2
./scripts/teleop.sh
```

注意 `sudo` 装在容器里，容器重建后会丢；要持久化得加进 Dockerfile 的 apt 层，
而那会让那一层（836 MB / 约 51 分钟）的构建缓存失效。

排查用命令：

```bash
gz topic -l                 # 看 gz 侧真实话题名
ros2 topic list             # 看 ROS 侧是否桥接成功
```

## 构建具体工作空间

不要在 `/workspace` 根目录直接执行 `colcon build`。进入具体 ROS 2 工作空间后再构建，例如：

```bash
./scripts/shell.sh
cd /workspace/ros2bookcode/chapt7/chapt7_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 3d 
```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.1 --y 0.0 --z 0.2 \
  --roll 0.0 --pitch 0.0 --yaw 0.0 \
  --frame-id base_link --child-frame-id base_laser

ros2 run tf2_ros static_transform_publisher \
  --x 0.3 --y 0.0 --z 0.0 \
  --roll 0.0 --pitch 0.0 --yaw 0.0 \
  --frame-id base_laser --child-frame-id wall_point

ros2 run tf2_ros tf2_echo base_link wall_point

# 查看 TF 树结构
ros2 run tf2_tools tf2_monitor

# 查看两个 frame 之间的变换
ros2 run tf2_ros tf2_echo base_link laser_frame

# 查看所有 frame
ros2 topic echo /tf_static

# pdf
ros2 run tf2_tools view_frames
ros2 topic info /tf_static
```

### `tf2_echo` 输出说明

执行：

```bash
ros2 run tf2_ros tf2_echo base_link wall_point
```

启动初期如果 `base_link` 还没有被发布，可能出现：

```text
Waiting for transform base_link -> wall_point:
Invalid frame ID "base_link" passed to canTransform argument target_frame
```

这表示当时 TF 树中还不存在 `base_link`。发布 TF 的节点启动后，若持续输出以下结果，说明变换已经可用：

```text
Translation: [0.400, 0.000, 0.200]
Rotation: in Quaternion (xyzw) [0.000, 0.000, 0.000, 1.000]
Rotation: in RPY (radian) [0.000, -0.000, 0.000]
```

即平移为 `(0.4, 0.0, 0.2)`，旋转为单位旋转。若一直等待，应检查 TF 发布节点、frame 名称以及 `ROS_DOMAIN_ID` 是否一致。

## 3d tools
```bash
sudo apt install ros-jazzy-mrpt2 -y
3d-rotation-converter

sudo apt install ros-$ROS_DISTRO-rqt-tf-tree
sudo apt install ros-jazzy-rqt-tf-tree
rm -rf ~/.config/ros.org/rqt_gui.ini

sudo apt install ros-$ROS_DISTRO-tf-transformations
from tf_transformations import quaternion_from_euler, euler_from_quaternion

# 欧拉角 → 四元数
q = quaternion_from_euler(0, 0, 1.57)  # 绕 z 轴转 90°

# 四元数 → 欧拉角
roll, pitch, yaw = euler_from_quaternion([0, 0, 0.707, 0.707])
sudo pip3 install transforms3d
import transforms3d as tfs

# 欧拉角 → 旋转矩阵
R = tfs.euler.euler2mat(0, 0, 1.57)

# 旋转矩阵 → 四元数
q = tfs.quaternions.mat2quat(R)

# 轴角 → 四元数
q = tfs.axangles.axangle2quat([0, 0, 1], 1.57)
```

## domainID

停止本环境只执行本目录 Compose project 的 `down`。

如果需要同时运行 ROS 2 仿真，保持本环境默认的 `ROS_DOMAIN_ID=49` 和 `GZ_PARTITION=slam-dev-49`

## 生成项目
```bash
ros2 pkg create --build-type ament_python \
  --dependencies rclpy geometry_msgs tf_ros tf_transformations \
  --license Apache-2.0 \
  my_tf_pkg

ros2 pkg create --build-type ament_cmake \
  --dependencies rclcpp tf2_ros geometry_msgs tf2_geometry_msgs \
  --license Apache-2.0 \
  my_tf_cpp
```

## 项目结构
```bash
my_tf_pkg/
├── my_tf_pkg/
│   └── __init__.py
├── resource/
│   └── my_tf_pkg
├── test/
├── package.xml       # 包信息（依赖写在这里）
├── setup.py          # Python 包安装配置
└── setup.cfg
```

## 安装插件
```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-tf-transformations
sudo apt install ros-jazzy-tf-transformations
sudo pip3 install tf-transformations
source /opt/ros/$ROS_DISTRO/setup.bash

sudo apt update && sudo apt install -y ros-jazzy-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

## 调试运行-1
```bash
colcon build 
source install/setup.bash
ros2 run my_tf_pkg static_tf_broadcaster
ros2 topic list 
ros2 topic echo /tf_static
```

## 调试运行-2
```bash
colcon build 
source install/setup.bash
ros2 run my_tf_pkg dynamic_tf_broadcaster
ros2 run tf2_ros tf2_echo base_link bottle_link
```

## rviz2
```bash
rviz2
rviz2 -d 
```

## turtlesim
```bash
sudo apt update
sudo apt install ros-jazzy-turtlesim
ros2 pkg executables turtlesim
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
ros2 topic list
ros2 bag record /turtle1/cmd_vel


sudo apt update
sudo apt install ros-jazzy-rqt-robot-steering
ros2 run rqt_robot_steering rqt_robot_steering
```

## urdf 
```bash
urdf_to_graphviz first_robot.urdf
```

## docker 环境
```bash
export QT_QPA_PLATFORM=xcb
export QT_DEBUG_PLUGINS=1  # 可选，用于查看插件加载错误
ros2 run rviz2 rviz2

docker run -it \
  --env="DISPLAY" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --device=/dev/input/mice \  # 可选，有时能改善输入事件传递
  your-image
```

## gazebo

### 本地模型目录是怎么被加载的

Gazebo Sim 8（`gz sim`，Harmonic）解析 `model://<name>` 时，只会去搜 `GZ_SIM_RESOURCE_PATH`
里列出的目录。规则有两点容易踩坑：

1. **`~/.gz/models` 不是自动加载的。** 那是 Gazebo classic 的 `~/.gazebo/models` 习惯，
   `gz sim` 不会自动扫它（容器内实测 `GZ_SIM_RESOURCE_PATH` 只有 `/opt/ros/jazzy/share`）。
2. **每个条目必须是「模型目录的父目录」**，不是模型本身。
   正确的树是 `/home/nvidia/.gz/models/ur10/model.config`，
   所以路径写 `/home/nvidia/.gz/models`，而不是 `.../models/ur10`。

`ros-jazzy-ros-gz` 的 hook 用 `prepend-non-duplicate` 把 `/opt/ros/jazzy/share`
加到最前面，因此我们预设的值会被保留，最终形如：

```text
/opt/ros/jazzy/share:/home/nvidia/.gz/models:/workspace/gazebo_models
```

本环境已经由 `compose.yaml` 的 `GZ_SIM_RESOURCE_PATH` 统一设置好了，
`ros-env.sh` 里还有一层幂等兜底，所以**不需要再往 `~/.bashrc` 里写 export**
（容器重建后 `~/.bashrc` 的改动会丢，写在 compose 里不会）。

### 验证

在容器里执行：

```bash
./scripts/shell.sh
echo "$GZ_SIM_RESOURCE_PATH"
ls "$HOME/.gz/models" | head          # 直接子目录就是各个模型
find "$HOME/.gz/models" -maxdepth 2 -name model.config | wc -l   # 262
```

真正的加载证据是 Gazebo 的启动日志：引用 `model://ur10` 的世界不会再报
`Unable to find file with URI [model://ur10]`，GUI 的 Resource Spawner →
Local resources 里也能看到 `/home/nvidia/.gz/models`。

### 常见操作

```bash
gz sim --versions                     # 8.15.0
mkdir -p ~/.gz/models
cp -r /workspace/gazebo_models/* ~/.gz/models/   # 或用 Fuel 在线下载
export QT_QPA_PLATFORM=xcb            # compose 里已设，交互式调试时保险
./scripts/gazebo.sh                           # 从宿主机启动
```

模型缓存已经 bind mount 到宿主机的 `my-slam/gz-cache/`，容器重建不会丢。

## world

两个 launch 文件并存，用途不同（教材原文件未做任何改动，仅新增 `_gz` 版本）：

| 文件 | 依赖 | 在 Jazzy 上 |
| --- | --- | --- |
| `gazebo_sim.launch.py`（教材原版） | `gazebo_ros`（Gazebo **classic**） | ❌ 跑不起来，classic 已 EOL，Jazzy 无 `gazebo_ros` |
| `gazebo_sim_gz.launch.py`（新增） | `ros_gz_sim`（Gazebo **Sim** 8.x） | ✅ 本仓库验证通过 |

```bash
# 启动仿真（推荐：宿主机一键入口）
./scripts/sim.sh

# 等价的手工写法（容器内）
ros2 launch mybot_description gazebo_sim_gz.launch.py

# 只跑 RViz 看模型，不开 Gazebo
ros2 launch mybot_description display_robot.launch.py

# 启动参数
./scripts/sim.sh --headless              # 只起 server，不渲染
./scripts/sim.sh --gz-verbose 4
./scripts/sim.sh --world /workspace/xxx/world.sdf
./scripts/sim.sh --clean                 # 启动前清掉上一次残留的仿真进程
```

**headless 模式是完整可用的**（2026-09-30 实测，无 GUI、纯 server）：

| 话题 | 实测频率 |
| --- | --- |
| `/clock` | ~2000 Hz |
| `/scan` | 4.99 Hz |
| `/imu` | 99.97 Hz |
| `/odom` | 50.02 Hz |
| `/joint_states` | 100.02 Hz |
| `/tf` | 68.17 Hz |
| `/camera/image` | 7.27 Hz |

即相机（离屏渲染）在 headless 下也正常，两个 controller 都 `active`。
适合在没接显示器、或 x11 socket 不可用时验证链路。

`./scripts/sim.sh` 每次启动都会检查容器里有没有上一次残留的仿真进程：终端被关掉 / 被 kill 时，
容器内的 `ros2 launch` 和 `gz sim` 不会跟着退出，而两者共用 `GZ_PARTITION`，
互相抢话题的现象非常难查。有残留时会警告，`--clean` 则直接清掉。

详见上文「mybot 的 Gazebo Sim 仿真」一节。宿主机上对应入口：
`./scripts/gazebo.sh` 起容器、`./scripts/shell.sh` 进容器、`./scripts/teleop.sh` 键盘遥控。

---

## 许可证

本项目以 **Apache-2.0** 授权，全文见 [`LICENSE`](LICENSE)。
`src/` 下 22 个包在各自的 `package.xml` 中统一声明 `<license>Apache-2.0</license>`。

### 为什么是 Apache-2.0（不是随手选的）

这个代码库里**只有 Apache-2.0 被声明过**，三处独立证据一致：

| 证据 | 内容 |
| --- | --- |
| 本项目原本已填的 4 个包 | `mybot_description`、`gz_sensor_bridge`、`my_tf_cpp`、`my_tf_pkg` —— 填的都是 `Apache-2.0` |
| 上游参考代码 `ros2bookcode/` | 16 个包声明 `Apache-2.0`（另外 26 个是未填的 `TODO`） |
| 本 README「生成项目」一节 | `ros2 pkg create` 示例里写的就是 `--license Apache-2.0` |

最后一条还解释了 `TODO` 的来源：`my_tf_pkg` / `my_tf_cpp` 正是 README 里用
**带 `--license Apache-2.0` 的命令**创建的，它们原本就声明了 Apache-2.0；
而其余 18 个包创建时没带 `--license`，于是留下了 `ros2 pkg create` 的脚手架默认值
`TODO: License declaration`。**所以这次统一是把原本的意图显式写全，不是改变授权。**

### `maintainer` 字段刻意没有统一

22 个包的 `maintainer` 目前有 4 类来源，**全部保留原样**：

| 来源 | 包数 | 值 |
| --- | --- | --- |
| 上游 `ros2bookcode` 的脚手架 | 17 | `fishros <87068644+fishros@users.noreply.github.com>` |
| 上游 `ros2bookcode`（另一种写法） | 1 | `fishros <fish@fishros.com>` —— `mybot_application_cpp` |
| 上游 `ros2bookcode` 的另一位作者 | 1 | `mzebra <mzebra@foxmail.com>` —— `mybot_description` |
| 容器里 `ros2 pkg create` 生成的占位值 | 2 | `nvidia <nvidia@todo.todo>` —— `my_tf_cpp`、`my_tf_pkg` |
| 本仓库自写 | 1 | `my-slam dev <dev@example.com>` —— `gz_sensor_bridge` |

前三类（19 个包）是**上游作者的归属信息**。Apache-2.0 第 4(c) 条要求再分发时保留
来源形式的版权与归属声明，所以**不应该**为了让字段看起来整齐就把它们改成自己的名字 ——
那正是这一条要防止的事。后两类（`nvidia@todo.todo`、`dev@example.com`，共 3 个包）
是纯粹的占位值，改不改都不涉及合规问题。

> 附：Python 包里的 `test/test_copyright.py` 是 `ros2 pkg create` 的脚手架产物
> （7 个包有，共 6 个 `setup.py` 声明了 `tests_require=['pytest']`）。
> 它用 `ament_copyright` 检查**源代码文件头**有没有版权与许可声明 ——
> 和 `package.xml` 的 `<license>` 是两件事。而我们的源码文件头确实没有版权声明。
>
> 不过这份脚手架里它**自己就被标了 skip**：
>
> ```python
> @pytest.mark.skip(reason='No copyright header has been placed in the generated source file.')
> def test_copyright():
>     rc = main(argv=['.', 'test'])
>     assert rc == 0, 'Found errors'
> ```
>
> 所以即使跑 `colcon test`，它也只会显示为 skipped，不会真的拦住构建。
> 本仓库没有 CI（无 `.github/workflows`），`colcon test` 也从未跑过
> （`log/` 下只有 `build_*`，没有测试结果目录）。

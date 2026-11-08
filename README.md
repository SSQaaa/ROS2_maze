# ROS2 地老鼠走迷宫系统

本项目是《机器人感知与控制综合实践》的课程项目，基于 ROS 2 Humble、Gazebo Classic 与 TurtleBot3 Burger 差分驱动机器人，实现开环运动、里程计 P 控制与航点跟踪、激光雷达分区处理、反应式避障和左墙跟随，并提供速度自适应的墙跟随版本用于对比实验。

系统通过 `/scan` 感知墙体、通过 `/odom` 获取位姿，向 `/cmd_vel` 发布运动指令。墙跟随采用雷达反馈，里程计用于航点控制、轨迹记录和计时。

| 项目 | 内容 |
| --- | --- |
| 课程代码 | S0610350 |
| 教学班代码 | `(2026-2027-1)-S0610350-04` |
| 功能包 | `maze_control`，构建类型为 `ament_python` |
| 机器人 | TurtleBot3 Burger |
| 仓库地址 | <https://github.com/SSQaaa/ROS2_maze> |

## 1. 功能与系统结构

| 功能 | 对应文件 | 实现内容 |
| --- | --- | --- |
| 仿真环境 | `worlds/*.world` | 七种自定义场景，墙体具有可视几何与碰撞几何 |
| 一键启动 | `launch/maze_sim.launch.py` | 加载 Gazebo、机器人描述、机器人实体、RViz2，并按参数启动一个控制算法 |
| 开环控制 | `open_loop_control.py` | 按时序直行、停车、原地旋转 |
| 闭环运动控制 | `point_control.py` | 基于 `/odom` 的距离和航向 P 控制，依次跟踪四个航点，发布轨迹 |
| 雷达处理 | `lidar_process.py` | 提取前、左、右分区距离，过滤无效测距并输出中位数 |
| 反应式避障 | `obstacle_avoidance.py` | 前进、转向、受阻停车状态切换，检测雷达超时 |
| 基础左墙跟随 | `wall_follow.py` | 根据前方障碍和左侧墙距执行右转、左修正、右修正或直行 |
| 优化左墙跟随 | `wall_follow_optimized.py` | 前方距离自适应调速、左墙距离低通滤波、连续角速度 P 控制 |
| 实验辅助 | `benchmark_timer.py`、根目录脚本 | 里程计计时、CSV 记录和雷达检查 |

通信关系如下：

```text
Gazebo + TurtleBot3 Burger
  ├─ /scan ──> 雷达处理 / 避障 / 墙跟随
  ├─ /odom ──> 航点控制 / 轨迹记录 / 计时
  └─ 接收 /cmd_vel <── 当前运行的控制节点

robot_state_publisher ──> 机器人描述与 TF
RViz2 <── 机器人模型、TF、/scan；可手动添加 /trajectory
```

Gazebo 负责动力学、碰撞及传感器仿真；RViz2 负责显示机器人模型、坐标变换和雷达数据。Launch 使用 TurtleBot3 官方 URDF 发布机器人描述，使用官方 SDF 在 Gazebo 中生成实体。

## 2. 环境与依赖

项目运行环境如下，后续命令在 Ubuntu Bash 终端执行。

| 组件 | 目标版本或配置 |
| --- | --- |
| 操作系统 | Ubuntu 22.04 LTS，具备桌面图形环境 |
| ROS 2 | Humble Hawksbill |
| 仿真器 | Gazebo Classic 11，使用 `gazebo_ros` 接口 |
| 可视化 | RViz2 |
| Python | Ubuntu 22.04 默认 Python 3.10 |
| 构建工具 | colcon、ament_python、setuptools |
| 模型资源 | `turtlebot3_gazebo`、`turtlebot3_description` 中的 Burger 模型 |

已安装并配置 ROS 2 Humble 软件源后，安装依赖：

```bash
sudo apt update
sudo apt install \
  python3-colcon-common-extensions python3-rosdep python3-setuptools \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-description \
  ros-humble-rviz2 ros-humble-robot-state-publisher ros-humble-xacro \
  ros-humble-rclpy ros-humble-sensor-msgs \
  ros-humble-geometry-msgs ros-humble-nav-msgs
```

Python 算法使用 ROS 2 消息与节点接口，以及 Python 标准库。其中 `nav_msgs` 用于里程计与轨迹消息。

## 3. 项目目录

```text
ROS2_maze/
├── README.md
├── LICENSE
├── src/
│   └── maze_control/
│       ├── package.xml
│       ├── setup.py
│       ├── setup.cfg
│       ├── resource/maze_control
│       ├── launch/maze_sim.launch.py
│       ├── rviz/maze.rviz
│       ├── worlds/
│       │   ├── straight_maze.world
│       │   ├── l_maze.world
│       │   ├── l2_maze.world
│       │   ├── shape7_maze.world
│       │   ├── u_maze.world
│       │   ├── z_maze.world
│       │   └── s_shape_maze.world
│       └── maze_control/
│           ├── __init__.py
│           ├── open_loop_control.py
│           ├── point_control.py
│           ├── lidar_process.py
│           ├── obstacle_avoidance.py
│           ├── wall_follow.py
│           ├── wall_follow_optimized.py
│           └── benchmark_timer.py
├── check_scan.py
├── record_odom.py
└── odom_l2.csv
```


## 4. 编译工作空间

将项目放到 `~/ROS2_maze`，确认目录结构为 `~/ROS2_maze/src/maze_control/package.xml`，然后执行：

```bash
source /opt/ros/humble/setup.bash
cd ~/ROS2_maze

# 仅首次使用 rosdep 且尚未初始化时执行一次
# sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

colcon build --symlink-install --packages-select maze_control
source install/setup.bash
export TURTLEBOT3_MODEL=burger

ros2 pkg prefix maze_control
ros2 pkg executables maze_control
```

`ros2 pkg executables` 应列出六个入口：`open_loop_control`、`point_control`、`lidar_process`、`obstacle_avoidance`、`wall_follow`、`wall_follow_optimized`。计时工具通过 Python 直接运行，见第 8 节。

每个新终端使用项目前，都要加载 ROS 2 和本工作空间环境：

```bash
source /opt/ros/humble/setup.bash
cd ~/ROS2_maze
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

修改 `setup.py`、Launch、world 或 RViz 配置后，重新构建并加载环境，确认修改已反映到安装资源中。

## 5. 一键启动迷宫仿真

### 5.1 基础左墙跟随

完成第 4 节环境加载后执行：

```bash
ros2 launch maze_control maze_sim.launch.py use_wall_follow:=true
```

该命令同时启动 Gazebo、`robot_state_publisher`、机器人生成节点、RViz2 和基础墙跟随节点。默认加载 S 型迷宫，出生位姿为 `(x, y, z, yaw) = (0.0, 1.0, 0.02, 0.0)`，机器人朝向世界坐标系的 `+X` 方向。机器人生成节点延迟约 8 秒启动，控制节点在收到雷达前保持停车。

### 5.2 优化版左墙跟随

先结束上一轮仿真，再执行：

```bash
ros2 launch maze_control maze_sim.launch.py use_wall_follow_optimized:=true
```

每轮实验选择一个控制算法，由该节点统一发布 `/cmd_vel`。Launch 对四个算法开关进行互斥检查。

### 5.3 仅启动仿真环境

```bash
ros2 launch maze_control maze_sim.launch.py
```

四个控制开关默认均为 `false`，因此此命令启动后机器人不会自主行驶，可用于雷达观察或单独测试算法。

### 5.4 切换场景与出生位置

`world` 接收 world 文件路径。例如，直线场景的入口位于两侧墙体之间，使用 `y=2.0`：

```bash
ros2 launch maze_control maze_sim.launch.py \
  world:="$(ros2 pkg prefix maze_control)/share/maze_control/worlds/straight_maze.world" \
  x:=0.0 y:=2.0 z:=0.02 yaw:=0.0 \
  use_wall_follow:=true
```

切换其他场景时，替换 world 文件名，并通过 `x`、`y`、`z`、`yaw` 设置对应入口位姿。

| Launch 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `world` | 安装目录中的 `s_shape_maze.world` | 仿真世界文件路径 |
| `x`、`y`、`z` | `0.0`、`1.0`、`0.02` | 世界坐标系下机器人初始位置，单位 m |
| `yaw` | `0.0` | 初始偏航角，单位 rad |
| `use_sim_time` | `true` | 为机器人状态发布器、RViz2 和可选算法节点启用仿真时钟 |
| `use_wall_follow` | `false` | 基础左墙跟随 |
| `use_wall_follow_optimized` | `false` | 优化版左墙跟随 |
| `use_obstacle_avoidance` | `false` | 反应式避障 |
| `use_point_control` | `false` | 里程计 P 控制与航点跟踪 |

可使用 `ros2 launch maze_control maze_sim.launch.py --show-args` 查看参数。

## 6. 分模块演示

### 6.1 开环直行与原地旋转

先仅启动环境，在另一已加载环境的终端运行：

```bash
ros2 run maze_control open_loop_control --ros-args -p use_sim_time:=true
```

默认动作依次为：以 `0.15 m/s` 直行 2 秒、停车 1 秒、以 `0.5 rad/s` 原地旋转 2 秒、停车 1 秒。时序在 `motion_sequence` 中修改，阶段持续时间采用主机单调时钟，演示时保持仿真连续运行。

### 6.2 里程计 P 控制与航点跟踪

```bash
ros2 launch maze_control maze_sim.launch.py use_point_control:=true
```

默认航点为 `(1, 0) → (1, 1) → (0, 1) → (0, 0)`，定义于 `point_control.py` 的 `self.waypoints`。航点采用 `/odom` 坐标系。该演示在无障碍区域进行；测试指定路线时，按里程计原点与场景可通行区域设置航点。

控制器计算目标距离误差和归一化航向误差：角度误差大于 `0.25 rad` 时先原地转向，否则同时调节线速度与角速度。距离误差小于 `0.01 m` 后切换到下一航点。默认距离增益为 `1.0`、角度增益为 `1.5`，限速为 `0.15 m/s` 和 `0.8 rad/s`。

节点向 `/trajectory` 发布 `nav_msgs/msg/Path`。查看轨迹时，在 RViz2 中添加 **Path** 显示并选择 `/trajectory`。

### 6.3 激光雷达分区处理

仅启动环境后，在另一已加载环境的终端运行：

```bash
ros2 run maze_control lidar_process --ros-args -p use_sim_time:=true
```

每条激光束角度按 `angle_min + i × angle_increment` 计算，再归一化到 `0°～360°`。前方取 `350°～10°`，左侧取 `80°～100°`，右侧取 `260°～280°`。过滤非有限值及超量程测距后，以各分区中位数作为距离，每 0.5 秒输出一次；无有效数据时显示“无有效回波”。

### 6.4 反应式避障

```bash
ros2 launch maze_control maze_sim.launch.py use_obstacle_avoidance:=true
```

正常以 `0.10 m/s` 前进；前方距离不大于 `0.30 m` 时停车，并选择空间较大的一侧以 `0.50 rad/s` 原地转向。至少转向 8 个控制周期且前方距离大于 `0.40 m` 后恢复前进；30 个周期内仍无法恢复时进入 `BLOCKED` 状态并保持停车，需要排查场景后重启节点。

该节点采用 Sensor Data QoS；前方取最小距离、左右取中位数，正无穷回波按最大量程处理。未收到雷达、雷达超过 1 秒未更新或前方分区数据无效时停车。

## 7. 墙跟随算法与参数

### 7.1 基础版决策逻辑

基础节点按下列优先级，每 0.1 秒计算一次速度：

| 条件 | 动作 |
| --- | --- |
| 尚未收到雷达 | 停车 |
| 前方距离 `< 0.35 m` | 停止前进，以 `-0.6 rad/s` 原地右转 |
| 左侧距离 `> 0.50 m` | 以 `0.15 m/s` 前进，角速度 `+0.4 rad/s`，向左靠墙 |
| 左侧距离 `< 0.30 m` | 以 `0.15 m/s` 前进，角速度 `-0.3 rad/s`，向右远离墙 |
| 其他情况 | 以 `0.15 m/s` 直行 |

目标左墙距离为 `0.4 m`。前方分区为 `0°～15°` 与 `345°～360°`，左侧为 `60°～90°`，均取最小有效距离。控制器根据距离条件切换行为，并持续使用雷达反馈修正运动。

### 7.2 优化版控制逻辑

优化版保持前方危险时原地右转，并加入以下处理：

- 左侧距离滤波：`filtered_left = 0.5 × left + 0.5 × 上次滤波值`。
- 连续转向：`angular_z = clamp(1.2 × (filtered_left - 0.8), -1.0, 1.0)`。
- 自适应调速：前方距离不大于 `0.5 m` 时停止前进；达到 `1.5 m` 时使用 `0.5 m/s`；中间区间按距离在 `0.08～0.5 m/s` 之间线性插值。

对比实验记录两种策略的目标墙距、速度上限和控制参数，使用相同场景、出生位姿与终点判定。研究单项改进时，保持其余参数一致。

墙距、速度与控制增益在对应 Python 类的构造函数中设置，调整后按第 4 节重新构建并加载环境。

### 7.3 运行设置

墙跟随使用 TurtleBot3 的 `0～2π` 扫描角度约定。实验前设置场景入口与控制参数，观察 RViz2 雷达数据和 Gazebo 运动过程，到达出口后手动结束本轮运行。


## 8. 实验记录与效果验证

### 8.1 检查话题与可视化

仿真启动后，在另一已加载环境的终端检查：

```bash
ros2 topic list
ros2 topic echo /odom --once
ros2 topic echo /scan --once --qos-reliability best_effort
ros2 topic info /cmd_vel --verbose
```

主要接口如下：

| 话题 | 消息类型 | 用途 |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 线速度与角速度指令 |
| `/odom` | `nav_msgs/msg/Odometry` | 位姿与速度反馈 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 激光雷达扫描 |
| `/trajectory` | `nav_msgs/msg/Path` | 航点控制节点运行时发布的历史轨迹 |
| `/clock` | `rosgraph_msgs/msg/Clock` | 仿真时钟 |
| `/tf`、`/tf_static` | `tf2_msgs/msg/TFMessage` | 坐标变换 |

RViz2 配置的 Fixed Frame 为 `odom`，默认显示 Grid、RobotModel、TF 和 LaserScan，雷达显示采用 Best Effort QoS。

### 8.2 记录里程计

根目录的记录脚本以约 0.1 秒为最小采样间隔，写入 `time,x,y,yaw,linear_x,angular_z`。`time` 是从脚本启动开始的主机单调时钟秒数，`yaw` 单位为 rad，其余位置和速度单位为 m、m/s、rad/s。

脚本固定以写入模式创建当前目录下的 `odom_l2.csv`。为保留仓库已有数据，每次实验在独立目录中运行：

```bash
cd ~/ROS2_maze
RUN_DIR="$(pwd)/results/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN_DIR"
cd "$RUN_DIR"
python3 ~/ROS2_maze/record_odom.py
```

按 `Ctrl+C` 结束记录并关闭文件。每轮记录配套保存场景名称、算法、参数与起终点设置，便于绘制轨迹和对比运动过程。仓库中的 `odom_l2.csv` 可用于查看记录格式。

### 8.3 起终点计时

在独立终端直接运行计时脚本：

```bash
cd ~/ROS2_maze
python3 src/maze_control/maze_control/benchmark_timer.py
```

在机器人运动前启动计时节点，交互输入终点 X、Y。脚本使用 `/odom` 消息时间戳计时。默认起点为 `(0, 1)`，双轴容差各 `0.3 m`；终点按 Y 轴判断，容差为 `0.3 m`。运行前按本轮里程计坐标设置脚本中的起点与终点判断轴。

默认 `end_one_way=False`，终点条件为 `abs(y - end_y) <= 0.3`。输出包括通行用时、起终点直线距离及平均直线速度，其中平均直线速度为起终点直线距离除以用时。计时工具负责观测与记录，机器人由当前控制节点驱动，完成实验后手动结束运行。



## 9. 常见问题排查

可按以下顺序检查环境、节点、话题和参数配置。

| 现象 | 检查与处理 |
| --- | --- |
| `Package 'maze_control' not found` | 确认在工作空间根目录构建成功，并在当前终端执行 `source install/setup.bash` |
| 找不到控制节点入口 | 对照 `setup.py` 的六个入口检查命令；修改入口后重新构建，计时脚本直接用 `python3` 运行 |
| 缺少 `nav_msgs` | 安装 `ros-humble-nav-msgs`，详见依赖说明 |
| 启动约 8 秒内暂未出现机器人 | Launch 设置了延迟生成；继续检查 `spawn_entity.py` 日志与 Gazebo 是否初始化完成 |
| 模型或网格资源加载失败 | 检查 TurtleBot3 相关包是否安装；Launch 会将其模型目录加入 `GAZEBO_MODEL_PATH` |
| 小车启动后不动 | 默认没有开启控制算法；检查算法开关、Gazebo 是否暂停，以及 `/scan`、`/odom` 是否有数据 |
| 雷达有发布但节点不接收 | 使用 `ros2 topic info /scan --verbose` 核对 QoS；部分节点使用默认可靠订阅，若发布端为 Best Effort，需调整订阅 QoS 后重建 |
| 小车速度或转向异常 | 检查 `/cmd_vel` 是否有多个发布者；同时检查初始位姿是否与墙体重叠 |
| RViz2 雷达或模型不显示 | 核对 Fixed Frame=`odom`、`/tf`、`/robot_description` 和 `/scan`；确认雷达显示 QoS |
| 避障后持续停车 | 查看是否进入 `BLOCKED` 状态，或发生雷达超时；排查后重新启动测试 |
| 计时无法开始或过早结束 | 检查 `/odom` 坐标、硬编码起点与终点 Y 轴容差；不要把世界坐标直接当作里程计坐标 |
| 修改 world 后场景没更新 | 重新构建和加载环境，核对 `world` 指向安装目录中的实际文件 |

雷达分区演示使用 `ros2 run maze_control lidar_process`。

## 10. 组员分工

项目按仿真与运动控制、感知与决策两个方向分配开发模块。

| 姓名 | 负责的代码模块 | 调试与验证职责 |
| --- | --- | --- |
| 孙嘉泓 | 雷达处理 `lidar_process.py`、反应式避障 `obstacle_avoidance.py`、基础与优化版墙跟随 `wall_follow.py` / `wall_follow_optimized.py` | 雷达分区与避障验证、墙跟随参数调试和策略对比，整理感知与决策部分说明 |
| 杨丰瑜 | 仿真场景 `worlds/`、一键启动 `maze_sim.launch.py`、RViz 配置、开环控制 `open_loop_control.py`、航点 P 控制 `point_control.py` | 环境与模型加载、运动控制和航点轨迹验证，整理建模与控制部分说明 |

系统联调、实验记录、演示视频与报告整理由两位成员协作开展。

## 11. 提交与开源说明

课程代码包以 **工作空间源码 `src/` 与本 README** 为核心，不打包 `build/`、`install/`、`log/`。如报告引用了辅助脚本或测量数据，应随附对应文件及实验条件。综合设计报告、演示视频按任务书单独整理，提交命名使用 `组号_组员姓名_材料类型`。

项目仓库：<https://github.com/SSQaaa/ROS2_maze>。课程报告参考文献中附上仓库链接与教学班代码 `(2026-2027-1)-S0610350-04`。项目采用 [Apache-2.0 许可证](LICENSE)，外部模型和软件来源见参考资源。

文档编写使用 AI 辅助整理结构与说明文字，课程报告按任务书记录 AI 使用范围与人工完善内容。

## 12. 参考资源

- [ROS 2 Humble 文档](https://docs.ros.org/en/humble/)：环境、通信与开发工具资料。
- [TurtleBot3 官方手册](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)：机器人平台及模型资料。
- [TurtleBot3 仿真资源源码](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)：本项目依赖的官方仿真模型来源。
- 课程提供的《任务书1：机器人感知与控制综合实践》与《ROS2大作业教程》：课程要求及学习资料。


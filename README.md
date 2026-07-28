# pb2025_sentry_ws

深圳北理莫斯科大学北极熊战队哨兵机器人 ROS2 工作空间，包含串口通信、视觉、导航、决策、仿真与裁判系统模拟工具。

## 1. 环境

- Ubuntu 22.04
- ROS2 Humble
- Ignition Fortress
- OpenVINO 2023.3

安装依赖：

```bash
cd ~/pb2025_sentry_wslzj
source /opt/ros/humble/setup.bash
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
```

编译：

```bash
cd ~/pb2025_sentry_wslzj
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

如果只改了行为树 XML、launch 或 yaml，通常不需要重新编译 C++，但需要重新 source：

```bash
source install/setup.bash
```

## 2. 关键词

`Control Panel` 的意思是“控制面板”。本工程里的 `pb_control_panel` 是一个 Qt 图形界面工具。

`Simulated Referee System` 的意思是“模拟裁判系统”。这里的“模拟”不是强行假数据写死在决策代码里，而是用一个独立 GUI 发布和真实裁判系统同名、同类型的 ROS2 话题，让行为树像收到真实裁判系统数据一样运行。

`Behavior Tree` 的意思是“行为树”。PB 决策部分使用 BehaviorTree.CPP 读取 XML 文件，XML 里的节点会对应到 C++ 插件。

`target_tree` 的意思是“要启动的行为树 ID”。UL 决策模式不是运行时发 topic 切换，而是在启动时选择不同 `target_tree`，也就是选择不同比赛基调。

## 3. 完整启动方式

### 3.1 只启动导航和 RViz

用于看地图、发布 Nav2 Goal、点击坐标，不启动视觉、串口、决策：

```bash
cd ~/pb2025_sentry_wslzj
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
  world:=9jiao \
  slam:=False \
  use_robot_state_pub:=True \
  use_rviz:=True
```

查看 RViz 里 `Publish Point` 点出来的坐标：

```bash
ros2 topic echo /clicked_point
```

### 3.2 全车启动

启动串口、视觉、导航、决策、录包节点：

```bash
cd ~/pb2025_sentry_wslzj
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch pb2025_sentry_bringup bringup.launch.py \
  world:=9jiao \
  use_rviz:=True
```

如果只在主机上测试，没有完整传感器或下位机，可以临时关掉相机并使用静态 TF：

```bash
ros2 launch pb2025_sentry_bringup bringup.launch.py \
  world:=9jiao \
  use_rviz:=True \
  use_hik_camera:=False \
  use_robot_state_pub:=True
```

### 3.3 全车启动并指定 UL 决策模式

激进进攻模式：

```bash
ros2 launch pb2025_sentry_bringup bringup.launch.py \
  world:=9jiao \
  use_rviz:=True \
  target_tree:=rmul2026_ul_aggressive
```

保守防守模式：

```bash
ros2 launch pb2025_sentry_bringup bringup.launch.py \
  world:=9jiao \
  use_rviz:=True \
  target_tree:=rmul2026_ul_conservative
```

占点优先模式：

```bash
ros2 launch pb2025_sentry_bringup bringup.launch.py \
  world:=9jiao \
  use_rviz:=True \
  target_tree:=rmul2026_ul_occupy_first
```

## 4. 决策单独启动

只启动行为树服务器和客户端，不启动导航、视觉、串口。适合配合模拟裁判系统验证条件判断是否进入正确分支。

激进进攻：

```bash
ros2 launch pb2025_sentry_behavior ul_aggressive.launch.py
```

保守防守：

```bash
ros2 launch pb2025_sentry_behavior ul_conservative.launch.py
```

占点优先：

```bash
ros2 launch pb2025_sentry_behavior ul_occupy_first.launch.py
```

观察决策输出：

```bash
ros2 topic echo /goal_pose
ros2 topic echo /cmd_spin
```

`/goal_pose` 是行为树发给导航的目标点，`/cmd_spin` 是行为树发给底盘/串口侧的自旋速度指令。

## 5. 模拟裁判系统

### 5.1 启动

单独开一个终端：

```bash
cd ~/pb2025_sentry_wslzj
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run pb_control_panel pb_control_panel
```

### 5.2 它发布的话题

`pb_control_panel` 会发布 PB 行为树服务器订阅的裁判系统话题：

- `/referee/game_status`
- `/referee/robot_status`
- `/referee/rfid_status`
- `/referee/all_robot_hp`
- `/referee/event_data`
- `/referee/buff`
- `/referee/ground_robot_position`

检查是否发布成功：

```bash
ros2 topic list | grep referee
ros2 topic echo --once /referee/game_status
ros2 topic echo --once /referee/robot_status
```

### 5.3 控制面板怎么用

打开窗口后，修改数值，然后点 `apply and publish`。窗口每秒也会自动继续发布当前状态。

`game_status` 是比赛状态：

- `game_progress = 0 NOT_START`：比赛未开始
- `game_progress = 1 PREPARATION`：准备阶段
- `game_progress = 2 SELF_CHECKING`：裁判系统自检阶段
- `game_progress = 3 COUNT_DOWN`：倒计时阶段
- `game_progress = 4 RUNNING`：比赛进行中
- `game_progress = 5 GAME_OVER`：比赛结束

`stage_remain_time` 是当前阶段剩余时间，单位是秒。勾选 `countdown` 后会每秒自动减 1。

`robot_status` 是我方机器人状态：

- `current_hp`：当前血量
- `maximum_hp`：最大血量
- `heat`：当前枪口热量
- `heat_limit`：热量上限
- `projectile_17mm`：17mm 允许发弹量
- `armor hit`：模拟被打
- `armor_id`：被打装甲板编号

`rfid_status` 是 RFID 区域状态：

- `supply non exchange`：在补给区但不可兑换
- `supply exchange`：在补给区且可兑换
- `center gain point`：在中心增益点
- `fortress gain point`：在己方堡垒/增益点

`all_robot_hp` 是双方关键建筑血量，行为树可以用它判断前哨站、基地是否危险。

### 5.4 配合行为树测试

终端 1，启动某个 UL 决策模式：

```bash
cd ~/pb2025_sentry_wslzj
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch pb2025_sentry_behavior ul_occupy_first.launch.py
```

终端 2，启动模拟裁判系统：

```bash
cd ~/pb2025_sentry_wslzj
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run pb_control_panel pb_control_panel
```

终端 3，观察输出：

```bash
ros2 topic echo /goal_pose
ros2 topic echo /cmd_spin
```

验收例子：

- 设置 `game_progress = 0 NOT_START`，行为树应发布 `cmd_spin = 0.0`。
- 设置 `game_progress = 4 RUNNING` 且血量正常，行为树进入对应 UL 策略。
- 设置低血量，行为树应回家点，同时保持自旋。
- 设置 `stage_remain_time` 进入占点窗口，行为树应发布占点目标 `1.59;-0.771;0.0`。

## 6. 常用调试命令

查看节点：

```bash
ros2 node list
```

查看话题：

```bash
ros2 topic list
```

查看话题类型：

```bash
ros2 topic info /referee/game_status
```

查看消息定义：

```bash
ros2 interface show pb_rm_interfaces/msg/GameStatus
ros2 interface show pb_rm_interfaces/msg/RobotStatus
```

检查包是否被 ROS2 识别：

```bash
ros2 pkg prefix pb2025_sentry_behavior
ros2 pkg prefix pb_control_panel
```

只编译决策和模拟裁判系统：

```bash
colcon build --packages-select pb2025_sentry_behavior pb_control_panel
source install/setup.bash
```

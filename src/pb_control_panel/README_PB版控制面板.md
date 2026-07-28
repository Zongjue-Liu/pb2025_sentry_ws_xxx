# PB 版 Control Panel 说明

`Control Panel` 是英文“控制面板”的意思。

`pb_control_panel` 是 PB 版模拟裁判系统工具。它启动后会弹出一个 Qt 窗口，用来手动设置比赛状态、机器人血量、弹量、RFID 区域状态等数据，并通过 ROS2 topic 发布出去。

`Simulated Referee System` 是英文“模拟裁判系统”的意思。这里模拟的是裁判系统“上发给上位机的数据”，不是替代真正的决策代码。真正做决策的是 `pb2025_sentry_behavior` 行为树。

## 1. 和火锅 control_panel 的关系

火锅的 control_panel 给了一个思路：用 GUI 模拟裁判系统数据，方便不用实物裁判系统时测试行为树。

PB 版没有整包照搬火锅代码，而是保留这个 GUI 思路，把发布的话题和消息类型换成 PB 工程正在使用的接口：

- `pb_rm_interfaces/msg/GameStatus`
- `pb_rm_interfaces/msg/RobotStatus`
- `pb_rm_interfaces/msg/RfidStatus`
- `pb_rm_interfaces/msg/GameRobotHP`
- `pb_rm_interfaces/msg/EventData`
- `pb_rm_interfaces/msg/Buff`
- `pb_rm_interfaces/msg/GroundRobotPosition`

## 2. 启动

```bash
cd ~/pb2025_sentry_wslzj
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run pb_control_panel pb_control_panel
```

如果包还没有编译：

```bash
cd ~/pb2025_sentry_wslzj
source /opt/ros/humble/setup.bash
colcon build --packages-select pb_rm_interfaces pb_control_panel
source install/setup.bash
```

## 3. 发布的话题

代码里写的是相对话题名，例如 `referee/game_status`。没有 namespace 时，终端里会显示成绝对话题名 `/referee/game_status`。

- `/referee/game_status`
- `/referee/robot_status`
- `/referee/rfid_status`
- `/referee/all_robot_hp`
- `/referee/event_data`
- `/referee/buff`
- `/referee/ground_robot_position`

检查命令：

```bash
ros2 topic list | grep referee
ros2 topic echo --once /referee/game_status
ros2 topic echo --once /referee/robot_status
```

## 4. 界面字段

`game_status` 表示比赛状态：

- `0 NOT_START`：比赛未开始
- `1 PREPARATION`：准备阶段
- `2 SELF_CHECKING`：裁判系统自检阶段
- `3 COUNT_DOWN`：倒计时阶段
- `4 RUNNING`：比赛进行中
- `5 GAME_OVER`：比赛结束

`stage_remain_time` 表示当前阶段剩余时间，单位是秒。`countdown` 勾选后会自动每秒减 1。

`robot_status` 表示我方机器人状态：

- `current_hp`：当前血量
- `maximum_hp`：最大血量
- `heat`：当前热量
- `heat_limit`：热量上限
- `projectile_17mm`：17mm 允许发弹量
- `armor hit`：是否模拟被打
- `armor_id`：被打装甲板编号

`rfid_status` 表示机器人当前是否处于对应功能区：

- `supply non exchange`：补给区但不可兑换
- `supply exchange`：补给区且可兑换
- `center gain point`：中心增益点
- `fortress gain point`：己方堡垒/增益点

`all_robot_hp` 表示双方前哨站、基地等血量。

修改字段后点击 `apply and publish`，会立刻发布一次当前状态。窗口运行期间也会每秒自动发布当前状态。

## 5. 配合行为树测试

终端 1：

```bash
cd ~/pb2025_sentry_wslzj
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch pb2025_sentry_behavior ul_occupy_first.launch.py
```

终端 2：

```bash
cd ~/pb2025_sentry_wslzj
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run pb_control_panel pb_control_panel
```

终端 3：

```bash
ros2 topic echo /goal_pose
ros2 topic echo /cmd_spin
```

当前 UL 决策模式不是通过运行时 topic 动态切换，而是通过不同 launch 文件选择：

- `ul_aggressive.launch.py`：激进进攻
- `ul_conservative.launch.py`：保守防守
- `ul_occupy_first.launch.py`：占点优先

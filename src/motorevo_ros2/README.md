# Motorevo ROS 2 驱动

面向泉智博（Motorevo）PA4310-20 关节电机的 ROS 2 Humble 驱动包，支持 MeowRobotics USB2FDCAN 适配器和 Linux SocketCAN。软件提供两套使用接口：

- `motorevo_node`：适合单电机调试、协议验证和轻量应用。
- `MotorevoSystem`：标准 `ros2_control` 硬件插件，适合控制器、规划器和机器人整机集成。

同一个 CAN 适配器不能同时被独立节点和 `ros2_control` 插件占用。开发时任选一种入口即可。

> [!CAUTION]
> 关节电机上电后可能瞬间输出较大力矩。第一次测试请拆除负载或架空关节，准备急停，并按本文“分阶段调试”从零增益开始。不要直接运行位置例程。

## 功能

- MIT 模式位置、速度、力矩前馈、`Kp` 和 `Kd` 混合控制
- 电机使能、失能、清故障和设置零点
- 位置、速度、估算力矩、温度和故障状态反馈
- 命令超时自动失能
- 单总线多电机配置
- ROS 2 话题、服务和诊断信息
- `ros2_control` 位置、速度和力矩命令接口
- MeowRobotics USB2FDCAN 原生 SDK 后端（配置名为 `meow_usb`）
- Linux SocketCAN / CAN FD 后端

## 适用环境

| 项目 | 支持情况 |
| --- | --- |
| 操作系统 | Ubuntu 22.04 LTS，推荐 x86_64 |
| ROS 2 | Humble Hawksbill |
| 编译标准 | C++17 |
| 默认总线 | CAN FD，仲裁域 1 Mbps，数据域 5 Mbps，BRS 开启 |
| 电机协议 | Motorevo Protocol，MIT 控制模式 |
| 已提供厂商库 | MeowRobotics USB2FDCAN，Linux x86_64 |
| 其他适配器 | 支持 SocketCAN 的 CAN/CAN FD 设备 |

ARM64 平台可以使用 SocketCAN；如果使用 Meow SDK，需要向适配器厂商索取 ARM64 版本动态库，并通过参数指定路径。

## 软件架构

```text
机器人控制器 / MoveIt / 自定义控制算法
                    │
          ros2_control 控制器
                    │
       MotorevoSystem 硬件插件
                    │
              MotorBus
        ┌───────────┴───────────┐
        │                       │
 Meow USB2FDCAN SDK       Linux SocketCAN
        │                       │
        └────────── CAN/CAN FD ┘
                    │
              PA4310-20
```

独立节点也直接调用 `MotorBus`，因此两种 ROS 接口共用同一套协议编解码、超时处理和传输后端。

## 仓库结构

```text
motorevo_ros2/
├── CMakeLists.txt                       # ament_cmake 构建、安装和测试规则
├── package.xml                          # ROS 2 依赖声明
├── motorevo_ros2_control.xml            # pluginlib 的 ros2_control 插件描述
├── config/
│   ├── motorevo_driver.yaml             # 独立节点参数
│   └── controllers.yaml                 # ros2_control 控制器参数
├── include/motorevo_ros2/
│   ├── types.hpp                        # 命令、反馈、限幅和故障类型
│   ├── protocol.hpp                     # Motorevo CAN 协议编解码
│   ├── transport.hpp                    # 传输层抽象接口
│   ├── motor_bus.hpp                    # 多电机总线和状态管理
│   └── meow_usb_abi.hpp                  # 厂商 SDK ABI 声明
├── src/
│   ├── protocol.cpp                     # 协议实现
│   ├── meow_usb_transport.cpp            # Meow SDK 动态加载后端
│   ├── socketcan_transport.cpp           # PF_CAN/SocketCAN 后端
│   ├── motor_bus.cpp                    # 命令、反馈和故障处理
│   ├── motorevo_node.cpp                # 独立 ROS 2 节点
│   └── motorevo_system.cpp              # ros2_control 插件实现
├── launch/
│   ├── driver.launch.py                 # 启动独立节点
│   └── ros2_control.launch.py            # 启动硬件插件和控制器
├── urdf/
│   └── pa43.urdf.xacro                   # 单关节 ros2_control 示例
├── scripts/
│   ├── setup_meow_usb2fdcan.sh            # USB 设备权限安装脚本
│   └── 99-meow-usb2fdcan.rules           # udev 规则
├── test/
│   └── test_protocol.cpp                 # 不连接硬件的协议单元测试
└── vendor/x86_64/
    └── libusb_fdcan.so                   # 厂商提供的 x86_64 动态库
```

## 依赖库

### ROS 2 依赖

| 依赖 | 用途 |
| --- | --- |
| `rclcpp`、`rclcpp_lifecycle` | 节点和生命周期接口 |
| `sensor_msgs` | 命令及关节状态 |
| `std_srvs` | 使能、失能、清故障和置零服务 |
| `diagnostic_msgs` | 电机和总线诊断 |
| `hardware_interface`、`pluginlib` | `ros2_control` 硬件插件 |
| `controller_manager` | 控制器管理 |
| `joint_state_broadcaster` | 发布关节状态 |
| `joint_trajectory_controller` | 轨迹控制示例 |
| `xacro`、`robot_state_publisher` | 机器人模型和 TF |

### 系统依赖

- CMake、GCC/G++ 和 `colcon`
- `libdl`：运行时加载 Meow SDK
- Linux `PF_CAN`：SocketCAN 后端
- `can-utils`：配置和观察 SocketCAN 总线
- Meow USB2FDCAN 厂商动态库：仅 `meow_usb` 后端需要

安装常用依赖：

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git \
  python3-colcon-common-extensions python3-rosdep \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-control-msgs \
  ros-humble-diagnostic-msgs \
  can-utils
```

## 安装

以下命令假设工作空间为 `~/motorevo_ws`。

### 1. 安装 ROS 2 并初始化 rosdep

先按 ROS 2 官方文档安装 Ubuntu 22.04 对应的 Humble，然后执行：

```bash
source /opt/ros/humble/setup.bash
sudo rosdep init        # 已初始化时会提示配置存在，可忽略
rosdep update
```

### 2. 放入工作空间并安装依赖

```bash
mkdir -p ~/motorevo_ws/src
cd ~/motorevo_ws/src

# 将本仓库中的 motorevo_ros2 目录复制或克隆到这里
# 最终应存在：~/motorevo_ws/src/motorevo_ros2/package.xml

cd ~/motorevo_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. 配置 Meow USB2FDCAN 权限

仅使用 Meow 后端时需要：

```bash
cd ~/motorevo_ws/src/motorevo_ros2
sudo bash scripts/setup_meow_usb2fdcan.sh
sudo udevadm control --reload-rules
sudo udevadm trigger
```

拔插 USB2FDCAN 后检查：

```bash
lsusb
ls -l /dev/bus/usb/*/*
groups
```

如果脚本把当前用户加入了新用户组，需要注销并重新登录。可用下面的命令检查厂商库架构和缺失依赖：

```bash
file vendor/x86_64/libusb_fdcan.so
ldd vendor/x86_64/libusb_fdcan.so
```

`ldd` 输出中不应出现 `not found`。ARM64 用户需把对应动态库放到自己的目录，并修改 `transport.library_path`。

### 4. 编译和测试

```bash
cd ~/motorevo_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select motorevo_ros2
colcon test --packages-select motorevo_ros2
colcon test-result --verbose
source install/setup.bash
```

每个新终端都要执行：

```bash
source /opt/ros/humble/setup.bash
source ~/motorevo_ws/install/setup.bash
```

确认安装成功：

```bash
ros2 pkg prefix motorevo_ros2
ros2 pkg executables motorevo_ros2
```

## 上电前检查

1. 固定或架空电机，移除机械负载，准备硬件急停。
2. 确认电机电源电压、极性和功率能力符合 PA4310-20 规格。
3. CANH、CANL 和 GND 正确连接；总线两端各使用一个 120 Ω 终端电阻。
4. 适配器与电机的 CAN FD、仲裁波特率、数据波特率和 BRS 设置一致。
5. 总线上每个电机使用唯一 ID，主机 ID 不和电机 ID 冲突。
6. 首次测试把软件中的 `kp`、`kd` 和力矩命令设为零。

## 电机上位机参数

先用泉智博上位机读取并保存原始配置，再核对这些参数。不同固件的参数名称可能略有差异，以电机厂商手册和上位机显示为准。

| 索引 | 常见名称 | 建议值或要求 |
| --- | --- | --- |
| 10 | Protocol Type | `Motorevo` |
| 11 | CAN Master ID | 与驱动的主机 ID 一致；上位机常以负数显示该模式 |
| 21 | Control Mode | MIT，常见枚举值为 `2` |
| 24 / 25 | Position Min / Max | 与驱动 `limits.position_*` 一致 |
| 26 / 27 | Velocity Min / Max | 与驱动 `limits.velocity_*` 一致 |
| 28 / 29 | Torque Min / Max | 与驱动 `limits.torque_*` 一致 |
| 30 / 31 | Kp Min / Max | 与驱动 `limits.kp_*` 一致 |
| 34 / 35 | Kd Min / Max | 与驱动 `limits.kd_*` 一致 |
| 36 | CAN ID | 与 `motor_ids` 一致，默认示例为 `1` |
| 37 | CAN bitrate | 与适配器仲裁域波特率一致 |
| 67 | CAN FD enable | 使用 CAN FD 时开启 |
| 69 | CAN FD data bitrate | 默认示例为 5 Mbps |

量程不一致会导致编解码后的实际位置、速度或力矩与命令不符。修改电机参数后应重新上电，再从零增益通信测试开始。

## 独立节点

### 配置

编辑 `config/motorevo_driver.yaml`。主要参数如下：

| 参数 | 含义 |
| --- | --- |
| `joint_names` | ROS 关节名称数组 |
| `motor_ids` | 与关节名称一一对应的电机 ID 数组 |
| `kp`、`kd` | 每个电机的默认 MIT 增益 |
| `update_rate` | 命令和反馈循环频率，Hz |
| `command_timeout_ms` | 最近命令超时阈值 |
| `disable_on_timeout` | 超时后是否主动失能 |
| `broadcast_mode` | 总线发送模式，按协议/适配器需求设置 |
| `inter_frame_delay_us` | 多帧发送间隔 |
| `transport.backend` | `meow_usb` 或 `socketcan` |
| `transport.device` | Meow 设备路径（默认 `/dev/USB2CAN0`）或 SocketCAN 接口名 |
| `transport.library_path` | Meow SDK 动态库路径；留空时使用包内库 |
| `transport.channel` | 双通道适配器的通道号 |
| `transport.can_fd` | 是否启用 CAN FD |
| `transport.bitrate_switch` | 是否启用 BRS |
| `transport.nominal_bitrate` | 仲裁域波特率 |
| `transport.data_bitrate` | 数据域波特率 |
| `limits.*` | 协议编解码量程，必须与电机参数一致 |

`joint_names` 和 `motor_ids` 的数组长度必须一致；`kp`、`kd` 可以只给一个公共值，也可以为每个关节各给一个值。广播模式下，当前实现要求电机 ID 位于 1～8。

### 启动

```bash
ros2 launch motorevo_ros2 driver.launch.py
```

### 话题和服务

| 名称 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/motorevo_driver/command` | `sensor_msgs/msg/JointState` | 订阅 | `position`、`velocity`、`effort` 分别作为 MIT 位置、速度和力矩前馈 |
| `/motorevo_driver/joint_states` | `sensor_msgs/msg/JointState` | 发布 | 实际位置、速度和估算力矩 |
| `/motorevo_driver/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 发布 | 总线、温度、故障和超时状态 |
| `/motorevo_driver/enable` | `std_srvs/srv/Trigger` | 服务 | 使能已配置电机 |
| `/motorevo_driver/disable` | `std_srvs/srv/Trigger` | 服务 | 失能已配置电机 |
| `/motorevo_driver/clear_fault` | `std_srvs/srv/Trigger` | 服务 | 清除电机故障 |
| `/motorevo_driver/set_zero` | `std_srvs/srv/Trigger` | 服务 | 把当前位置写为机械零点 |

命令示例只需填写实际要控制的关节；数组索引与 `name` 对应：

```bash
ros2 topic pub -r 100 /motorevo_driver/command sensor_msgs/msg/JointState \
  "{name: ['joint1'], position: [0.0], velocity: [0.0], effort: [0.0]}"
```

观察反馈：

```bash
ros2 topic echo /motorevo_driver/joint_states
ros2 topic echo /motorevo_driver/diagnostics
ros2 topic hz /motorevo_driver/joint_states
```

## 分阶段调试

以下流程每次只改变一个变量。看到故障、异常温升、抖动或转向错误时，立即失能并切断电源检查。

### 阶段 1：零增益通信验证

在 `motorevo_driver.yaml` 中设置：

```yaml
kp: [0.0]
kd: [0.0]
command_timeout_ms: 500
disable_on_timeout: true
```

启动节点后，先在另一个终端持续发布零输出命令：

```bash
ros2 topic pub -r 100 /motorevo_driver/command sensor_msgs/msg/JointState \
  "{name: ['joint1'], position: [0.0], velocity: [0.0], effort: [0.0]}"
```

保持发布器运行，再使能：

```bash
ros2 service call /motorevo_driver/enable std_srvs/srv/Trigger '{}'
```

此时 `Kp`、`Kd` 和力矩前馈均为零，电机不应主动跟踪位置。确认关节状态持续更新、ID 正确、温度正常且诊断无故障。完成后失能：

```bash
ros2 service call /motorevo_driver/disable std_srvs/srv/Trigger '{}'
```

### 阶段 2：低阻尼验证方向

把 `kp` 保持为 `0.0`，把 `kd` 改为 `0.1`～`0.2`。重新启动并重复阶段 1。缓慢手动转动输出轴，确认反馈位置方向和机械定义一致，且阻尼平顺。

### 阶段 3：小幅位置控制

先记录 `/joint_states` 的当前位置并失能。把 `kp` 设为较小值，例如 `2.0`，`kd` 设为 `0.2`。重新启动后，先持续发布记录到的当前位置，再使能。确认稳定后，每次只改变约 `0.02`～`0.05 rad`：

```bash
ros2 topic pub -r 100 /motorevo_driver/command sensor_msgs/msg/JointState \
  "{name: ['joint1'], position: [0.05], velocity: [0.0], effort: [0.0]}"
```

逐步增加增益和运动范围，并持续观察电流、温度、机械限位和诊断信息。

### 故障清除和零点设置

```bash
ros2 service call /motorevo_driver/clear_fault std_srvs/srv/Trigger '{}'
ros2 service call /motorevo_driver/set_zero std_srvs/srv/Trigger '{}'
```

设置零点会改变电机坐标基准。确保机构处于明确的机械零位并已卸载或可靠支撑后再执行。

## ros2_control 接口

### 状态接口

| 接口 | 单位 | 含义 |
| --- | --- | --- |
| `position` | rad | 电机输出轴位置 |
| `velocity` | rad/s | 电机输出轴速度 |
| `effort` | N·m | 估算输出力矩 |
| `temperature` | °C | 电机温度 |
| `fault_code` | 位掩码 | 电机故障码 |
| `enabled` | 0/1 | 电机使能状态 |

### 命令接口

| 接口 | MIT 字段 | 用途 |
| --- | --- | --- |
| `position` | `p_des` | 目标位置 |
| `velocity` | `v_des` | 目标速度 |
| `effort` | `t_ff` | 力矩前馈 |

`Kp` 和 `Kd` 在 URDF/Xacro 的关节硬件参数中配置。轨迹控制器通常占用位置接口；自定义控制器可同时声明位置、速度和力矩命令接口。

### 硬件参数

`urdf/pa43.urdf.xacro` 中的 `<ros2_control>` 参数包括：

- 总线级：`backend`、`device`、`library_path`、`channel`、`can_fd`、`bitrate_switch`、`nominal_bitrate`、`data_bitrate`、`broadcast_mode`、`feedback_timeout_ms`、`halt_on_fault`、`inter_frame_delay_us`，以及协议位置、速度、力矩和增益量程。
- 关节级：`motor_id`、`kp`、`kd`、`direction` 和 `offset`。`direction` 只能为 `1` 或 `-1`。

### 单关节示例

先按“分阶段调试”完成独立节点测试并关闭独立节点，再启动：

```bash
ros2 launch motorevo_ros2 ros2_control.launch.py
```

检查硬件和控制器：

```bash
ros2 control list_hardware_components
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 topic echo /joint_states
```

确认当前命令初值与实际位置一致后，发送一个小幅轨迹：

```bash
ros2 action send_goal \
  /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: ['joint1'], points: [
    {positions: [0.02], time_from_start: {sec: 2}},
    {positions: [0.0],  time_from_start: {sec: 4}}
  ]}}"
```

停止轨迹控制器并让硬件插件进入安全状态：

```bash
ros2 control switch_controllers \
  --deactivate joint_trajectory_controller \
  --strict
```

### 多电机配置

每个关节添加一个唯一 `motor_id`，并在控制器中列出全部关节。例如：

```xml
<joint name="joint2">
  <param name="motor_id">2</param>
  <param name="kp">2.0</param>
  <param name="kd">0.2</param>
  <command_interface name="position"/>
  <command_interface name="velocity"/>
  <command_interface name="effort"/>
  <state_interface name="position"/>
  <state_interface name="velocity"/>
  <state_interface name="effort"/>
</joint>
```

```yaml
joint_trajectory_controller:
  ros__parameters:
    joints: [joint1, joint2]
    command_interfaces: [position, velocity]
    state_interfaces: [position, velocity]
```

总线上的发送帧随电机数量增加。电机较多时应降低单电机更新率或增大 `inter_frame_delay_us`，并用诊断信息检查丢帧和超时。

## SocketCAN 后端

将独立节点配置改为：

```yaml
transport:
  backend: socketcan
  device: can0
  can_fd: true
  bitrate_switch: true
  nominal_bitrate: 1000000
  data_bitrate: 5000000
```

### CAN FD 1M/5M

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can \
  bitrate 1000000 dbitrate 5000000 fd on berr-reporting on restart-ms 100
sudo ip link set can0 up
ip -details -statistics link show can0
```

### CAN FD 1M/4M

部分适配器或电机只支持 4 Mbps 数据域：

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can \
  bitrate 1000000 dbitrate 4000000 fd on berr-reporting on restart-ms 100
sudo ip link set can0 up
```

同时把 YAML/URDF 的 `data_bitrate` 改为 `4000000`。

### 经典 CAN

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up
```

并设置 `can_fd: false`、`bitrate_switch: false`。只有电机协议和帧长度允许经典 CAN 时才能这样使用。

抓取总线数据和错误计数：

```bash
candump -tz -x can0
ip -details -statistics link show can0
```

## 故障与诊断

诊断话题会报告通信状态、温度和电机故障位。常见故障类别包括：

| 类别 | 优先检查 |
| --- | --- |
| 过压 / 欠压 | 电源电压、回生能量、线缆压降 |
| 过流 / 过载 | 机械卡滞、增益、加速度、负载 |
| 电机 / MOS 温度过高 | 散热、负载周期、持续力矩 |
| 编码器故障 | 编码器连接、零点和电机内部故障 |
| 通信超时 | ID、波特率、CAN FD/BRS、终端电阻、接地 |

清故障前先排除根因。频繁重复清故障可能使电机或机构进一步受损。

## 常见问题

| 现象 | 检查方法 |
| --- | --- |
| 找不到 `motorevo_ros2` | 确认已 `colcon build`，新终端已 source 工作空间 |
| Meow SDK 打不开设备 | 检查 USB、udev、用户组、通道号、设备是否被上位机占用 |
| 动态库加载失败 | 用 `file` 和 `ldd` 检查架构及依赖，设置绝对 `library_path` |
| SocketCAN 提示接口不存在 | 用 `ip link` 检查驱动和接口名称 |
| 只能发不能收 | 核对主机 ID、电机 ID、协议类型、CAN FD/BRS 和接线 |
| 位置或力矩比例明显错误 | 核对电机上位机量程与 `limits.*` |
| 电机使能后跳动 | 立即失能；检查目标初值、零点、方向和增益，从零增益重测 |
| 多电机丢帧 | 降低更新率、增大发帧间隔、检查总线负载和终端电阻 |
| 控制器无法激活 | 检查 URDF 接口和控制器声明是否一致，查看 controller manager 日志 |

## C++ 二次开发

不依赖 ROS 控制器时，可以直接复用传输层和 `MotorBus`：

```cpp
#include "motorevo_ros2/motor_bus.hpp"
#include "motorevo_ros2/transport.hpp"

motorevo_ros2::TransportConfig config;
config.backend = "socketcan";
config.device = "can0";
config.can_fd = true;
config.bitrate_switch = true;
config.nominal_bitrate = 1000000;
config.data_bitrate = 5000000;

auto transport = motorevo_ros2::make_transport(config.backend);
motorevo_ros2::Protocol protocol;
motorevo_ros2::MotorBus bus(
  std::move(transport), protocol, config, true, 200);
bus.open();

motorevo_ros2::MotorCommand command;
command.id = 1;
command.kp = 0.0;
command.kd = 0.0;
bus.send_commands({command});
// 收到有效反馈后再使能，并在固定周期内持续收发。
```

新增适配器后端时：

1. 实现 `Transport` 接口。
2. 在 `make_transport()` 的后端分派中注册新的 `backend` 名称。
3. 保持协议层和 ROS 层不变。
4. 增加回环或仿真测试，验证标准帧、扩展帧、CAN FD 和错误路径。

## 测试范围

当前自动化测试覆盖协议字段编码、解码、量程边界和故障位解析，不需要连接电机。硬件发布前建议补充：

- 适配器回环测试
- 单电机长时间收发和超时测试
- 多电机满负载总线测试
- 急停、掉电、断线和重新连接测试
- `ros2_control` 控制器切换测试

## 参考项目

本包的分层和使用方式参考了以下成熟开源驱动：

- [RobStride ROS2](https://github.com/s2015-turtle/robstride_ros2)：协议库、硬件接口、诊断和多电机配置的分层方式。
- [ODrive ROS 2](https://github.com/odriverobotics/ros_odrive)：独立节点与 `ros2_control` 两种接入方式，以及标准命令/状态接口。
- [ros2_control demos](https://github.com/ros-controls/ros2_control_demos)：URDF、硬件插件、控制器 YAML 和启动文件的标准组织方式。
- [mini-cheetah-tmotor-can](https://github.com/dfki-ric-underactuated-lab/mini-cheetah-tmotor-can)：关节电机 CAN 驱动和上层控制复用思路。

这些项目使用的电机协议与 PA4310-20 不同，不能直接替换本包的协议实现或参数。

## 许可证与厂商文件

项目源码的许可证见仓库中的许可证文件。`vendor/` 下的 MeowRobotics 动态库由适配器厂商提供，其授权、支持平台和再分发条件以厂商条款为准。

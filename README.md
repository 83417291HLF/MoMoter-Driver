# Motorevo PA43 ROS 2 驱动

这是面向 Ubuntu 22.04、ROS 2 Humble 和泉智博 PA4310-20 关节电机的 ROS 2 工作空间。驱动支持 MeowRobotics 双通道 USB2FDCAN 和 Linux SocketCAN，既可以作为独立 ROS 2 节点运行，也可以通过 `ros2_control` 接入机器人控制器。

完整手册见 **[motorevo_ros2 驱动说明](src/motorevo_ros2/README.md)**，其中包含：

- [仓库结构](src/motorevo_ros2/README.md#仓库结构)和各模块职责
- [依赖库](src/motorevo_ros2/README.md#依赖库)、Ubuntu 22.04 安装及编译步骤
- [上电前检查](src/motorevo_ros2/README.md#上电前检查)和电机上位机参数表
- [独立节点接口](src/motorevo_ros2/README.md#独立节点)、话题、服务和命令例程
- [零增益到小幅运动的分阶段调试](src/motorevo_ros2/README.md#分阶段调试)
- [`ros2_control` 接口与轨迹例程](src/motorevo_ros2/README.md#ros2_control-接口)
- [SocketCAN 配置](src/motorevo_ros2/README.md#socketcan-后端)、故障诊断和二次开发入口

## 工作空间结构

```text
MoMoter-ROS2/
├── README.md
└── src/
    └── motorevo_ros2/
        ├── config/     # 独立节点和控制器参数
        ├── include/    # 协议、传输及总线接口
        ├── launch/     # 独立节点和 ros2_control 启动文件
        ├── scripts/    # Meow USB 权限配置
        ├── src/        # 驱动和硬件插件实现
        ├── test/       # 协议单元测试
        ├── urdf/       # 单关节 ros2_control 示例
        └── vendor/     # Meow x86_64 厂商动态库
```

## 快速编译

```bash
source /opt/ros/humble/setup.bash
cd ~/motorevo_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select motorevo_ros2
colcon test --packages-select motorevo_ros2
source install/setup.bash
```

## 选择运行方式

独立节点适合首轮通信和单电机调试：

```bash
ros2 launch motorevo_ros2 driver.launch.py
```

`ros2_control` 适合轨迹控制和机器人整机集成：

```bash
ros2 launch motorevo_ros2 ros2_control.launch.py
```

同一个 CAN 适配器不能同时被这两个入口占用。第一次带电前，请先用上位机保存电机参数，并严格按照完整手册中的零增益流程进行通信验证。

# Hunter180 Deploy

Hunter180 人形机器人的 ROS2 Jazzy 部署工作空间。目录保留为 `ec_deploy`，
软件包命名参考 `hunter130_deploy`，统一使用 `ec_` 前缀。

## 软件包

| 软件包 | 说明 |
|--------|------|
| [ec_description](src/ec_description/) | Hunter180 URDF、网格和 ros2_control 配置 |
| [ec_joint_hardware](src/ec_joint_hardware/) | 关节电机硬件插件 |
| [ec_imu_hardware](src/ec_imu_hardware/) | IMU 硬件插件 |
| [ec_radio](src/ec_radio/) | IBUS 遥控器消息和串口节点 |
| [ec_controller](src/ec_controller/) | 阻尼、站立、强化学习行走控制器及切换节点 |

原 `phanerox_description`、`phanerox_controller`、`phanerox_radio` 分别更名为
`ec_description`、`ec_controller`、`ec_radio`。控制器插件名使用
`ec/DefaultController`、`ec/StandController`、`ec/WalkController`，C++ 命名空间为 `ec`。

## 环境和依赖

- Ubuntu 24.04、ROS2 Jazzy
- Encos `encos_driver` 和 `joint_sdk` C++ SDK
- 内置 ONNX Runtime 1.28.1（amd64 / x86_64）

以下命令在 Bash 终端中执行。安装 ROS2 Jazzy 后，安装构建和运行依赖：

```bash
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install -y \
  build-essential cmake python3-colcon-common-extensions \
  libeigen3-dev libyaml-cpp-dev nlohmann-json3-dev liburdfdom-tools \
  python3-tk wget \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-ament-index-cpp ros-jazzy-ament-index-python \
  ros-jazzy-asio-cmake-module ros-jazzy-serial-driver \
  ros-jazzy-realtime-tools ros-jazzy-tf2-ros \
  ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-ros2launch ros-jazzy-xacro \
  ros-jazzy-rosidl-default-generators ros-jazzy-rosidl-default-runtime \
  ros-jazzy-ament-cmake-gtest ros-jazzy-ament-cmake-pytest \
  ros-jazzy-ament-lint-auto ros-jazzy-ament-lint-common \
  ros-jazzy-launch-testing-ament-cmake ros-jazzy-launch-testing-ros
```

SDK 安装方式参考 `hunter130_deploy`，使用官方
[encos_driver v3.3.2](https://github.com/EncosTech/encos_driver/releases/tag/v3.3.2)
和 [joint_sdk v1.4.5](https://github.com/EncosTech/joint_sdk/releases/tag/v1.4.5) DEB 包：

```bash
sdk_arch=$(dpkg --print-architecture)
mkdir -p /tmp/hunter180-sdk
wget -P /tmp/hunter180-sdk \
  "https://github.com/EncosTech/encos_driver/releases/download/v3.3.2/libencosdriver_3.3.2_${sdk_arch}_noble.deb" \
  "https://github.com/EncosTech/joint_sdk/releases/download/v1.4.5/jointsdk_1.4.5_${sdk_arch}_noble.deb"
sudo apt install -y \
  "/tmp/hunter180-sdk/libencosdriver_3.3.2_${sdk_arch}_noble.deb" \
  "/tmp/hunter180-sdk/jointsdk_1.4.5_${sdk_arch}_noble.deb"
sudo ldconfig
```

ARM64 部署需要另外替换 `src/ec_controller/thirdparty/onnxruntime/` 中的头文件和
共享库为对应架构版本。关节滑块工具还需要提供含 `FsmState.srv` 的
`robot_interfaces` 包；常规控制器启动不需要该包。

## 编译

进入本工作空间目录后执行：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

如果已有旧包构建结果，先移走旧的 `build/`、`install/`、`log/`，并使用未加载旧
工作空间的新终端重新编译，避免原 `phanerox_*` 包残留。

## 启动

启动控制器管理器、状态发布器和控制器切换节点：

```bash
ros2 launch ec_controller real.launch.py
```

在另一个已加载工作空间环境的终端启动遥控器：

```bash
ros2 run ec_radio radio_node --ros-args \
  -p serial_port:=/dev/ttyUSB0 -p baud_rate:=115200
```

| 模式 | 控制器 | 行为 |
|------|--------|------|
| 1 | `default_controller` | 零比例增益，阻尼增益为 2.5 |
| 2 | `stand_controller` | 保持配置中的站立姿态 |
| 3 | `walk_controller` | ONNX 策略行走 |

切换顺序为阻尼 → 站立 → 行走 → 阻尼，也允许站立 → 阻尼。
CH0 控制偏航角速度、CH2 控制前后速度、CH3 控制左右速度，1450–1550 为死区。
CH4 和 CH5 分别选择站立和行走；无按键时选择阻尼。CH6、CH7 为预留模式，
当前切换节点将其映射到阻尼模式。

手动切换示例：

```bash
ros2 service call /controller_manager/switch_controller \
  controller_manager_msgs/srv/SwitchController \
  "{activate_controllers: ['stand_controller'], deactivate_controllers: ['default_controller'], strictness: 2}"
```

## 模型和硬件配置

模型来源：`../hunter130_train/legged_lab/assets/hunter180/urdf/hunter180.urdf`。
已将该 URDF 和它引用的 24 个 STL 文件复制到本工作空间，部署时无需训练仓库。
惯性、关节坐标、轴向和限位沿用源模型；网格路径转换为
`package://ec_description/meshes/...`，可由 ROS 正确解析。

```text
ec_deploy/
├── src/
│   ├── ec_description/
│   │   ├── meshes/
│   │   └── urdf/
│   │       ├── hunter180.urdf      # Hunter180 本体模型
│   │       ├── hunter180.xacro     # 部署入口：模型 + 硬件接口
│   │       └── ros2_control.xacro  # 关节和 IMU 配置
│   ├── ec_joint_hardware/
│   │   └── config/control.json    # SDK 电机、总线、标定和保护参数
│   ├── ec_imu_hardware/
│   ├── ec_radio/
│   └── ec_controller/
│       ├── config/
│       │   ├── controllers.yaml
│       │   └── policy/
│       ├── include/ec_controller/
│       ├── launch/real.launch.py
│       └── src/
└── README.md
```

模型共 23 个受控关节：左右腿各 6 个、腰部 3 个、左右臂各 4 个。
`controllers.yaml`、`ros2_control.xacro` 和 `control.json` 按上述顺序对应。

本次整理保留原 `ec_deploy` 的姿态、PD 增益、动作缩放和策略文件。
原站立/行走配置的肘关节目标为 1.0，而训练资产默认值为 -0.5；使用新导出的
Hunter180 策略时，应同时核对默认姿态和观测/动作定义。
URDF 中的速度、力矩限值用于模型描述，SDK 的 `maxSpd`、`maxTor` 保留原值，
没有自动提高硬件保护阈值。实机部署前需核对 `control.json` 中的电机型号、
电机方向、总线编号、并联踝/腰机构标定和保护参数。

`ros2_control.xacro` 中的 `adapter_type_0`、`interface_name_0` 配置关节通信，
IMU 的 `adapter_type`、`interface_name`、`bus_idx` 等参数配置传感器通信。
当前沿用 EtherCAT 和网卡名 `enp86s0`，需与目标机器一致。

## 策略和日志

策略路径在 `controllers.yaml` 中配置，默认 `config/policy/policy.onnx`，
相对路径从 `ec_controller` 的安装目录解析。
现有策略文件已保留，尚未确认其训练来源；更换模型不代表策略已重新训练或适配。

- 每帧观测 78 维，历史 10 帧，ONNX 输入共 780 维。
- 输出 23 维动作，推理频率 50 Hz，控制更新频率 1000 Hz。
- 控制器将 YAML 关节顺序转换为策略使用的左右交错顺序。

行走日志功能保留。`log_dir` 默认为空，使用 `/tmp/walk_log`，
每次激活行走控制器生成新的 `walk_log_N.csv`。
日志每个控制周期采样一次，按当前 `update_rate: 1000` 配置目标为 1000 Hz；
`time_ns` 使用控制周期传入的时间戳，实际频率受控制循环调度影响。
原有 CSV 字段和顺序保持不变：IMU、关节反馈和指令记录本周期的最新值，
`jp/jv/pa/act_raw/act` 保留最近一次策略观测/输出，仍按 50 Hz 更新
（首次推理前使用激活时的初始观测和零动作）。
日志使用单线程：在控制更新中格式化 CSV，使用 64 KiB 文件流缓冲区，
不再逐行刷新；缓冲区满时由当前线程写盘，退出 walk mode 时刷新并关闭文件。
写盘失败后停止记录，退出 walk mode 时报告错误。
格式化和写盘仍占用控制周期，因此实际能否稳定达到 1000 Hz 需上机测量。
可在 YAML 中将 `log_dir` 设置为目标机器的绝对路径。

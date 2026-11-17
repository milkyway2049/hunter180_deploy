# Hunter180 AMP Deploy

Hunter180 人形机器人的 ROS2 Jazzy 部署工作空间。硬件接口参考 `hunter130_deploy`，
`hunter180_amp` 分支部署 AMP_mjlab 的 Hunter180 AMP Flat 策略，
适配 `encos_driver >= 4.2.3` / `joint_sdk >= 1.5.7`，软件包统一使用 `ec_` 前缀。
主控制循环为 **500 Hz（2 ms）**，行走策略推理保持 **50 Hz（20 ms）**。

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
- Encos `encos_driver >= 4.2.3` 和 `joint_sdk >= 1.5.7` C++ SDK
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
[encos_driver v4.2.3](https://github.com/EncosTech/encos_driver/releases/tag/v4.2.3)
和 [joint_sdk v1.5.7](https://github.com/EncosTech/joint_sdk/releases/tag/v1.5.7) DEB 包：

```bash
sdk_arch=$(dpkg --print-architecture)
mkdir -p /tmp/hunter180-sdk
wget -P /tmp/hunter180-sdk \
  "https://github.com/EncosTech/encos_driver/releases/download/v4.2.3/libencosdriver_4.2.3_${sdk_arch}_noble.deb" \
  "https://github.com/EncosTech/joint_sdk/releases/download/v1.5.7/jointsdk_1.5.7_${sdk_arch}_noble.deb"
sudo apt install -y \
  "/tmp/hunter180-sdk/libencosdriver_4.2.3_${sdk_arch}_noble.deb" \
  "/tmp/hunter180-sdk/jointsdk_1.5.7_${sdk_arch}_noble.deb"
sudo ldconfig
```

硬件包使用新版 `device/imu/imu.h`、`device/motor/types.h` 和
`CreateAdapter(json)` 接口，CMake 会检查 SDK 最低版本；旧版 SDK 不再适用。

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

若 CMake 选中了 Anaconda Python，可显式指定系统 Python：

```bash
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
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
hunter180_deploy/
├── src/
│   ├── ec_description/
│   │   ├── config/control.json    # SDK 电机、总线、标定和保护参数
│   │   ├── meshes/
│   │   └── urdf/
│   │       ├── hunter180.urdf      # Hunter180 本体模型
│   │       ├── hunter180.xacro     # 部署入口：模型 + 硬件接口
│   │       └── ros2_control.xacro  # 关节和 IMU 配置
│   ├── ec_joint_hardware/
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

站立和行走使用 AMP 训练的默认姿态、PD 增益，肘关节默认位置为 -0.5 rad；
行走动作缩放逐关节匹配训练配置。
URDF 中的速度、力矩限值用于模型描述，SDK 的 `maxSpd`、`maxTor` 保留原值，
没有自动提高硬件保护阈值。实机部署前需核对 `control.json` 中的电机型号、
电机方向、总线编号、并联踝/腰机构标定和保护参数。

`ros2_control.xacro` 中的 `adapter_type_0`、`interface_name_0` 配置关节通信，
IMU 的 `adapter_type`、`interface_name`、`bus_idx` 等参数配置传感器通信。
当前沿用 EtherCAT 和网卡名 `enp86s0`，需与目标机器一致。

当前实机使用旧款 IMU，连接从站 `0` 的总线 `7`，`imu_idx=0`，
经典 CAN 波特率为 **500 kbit/s**。CAN7 时序由 IMU 插件配置，
不再放在关节 `control.json` 中，避免 IMU 先激活时关节配置尚未执行，
或者关节插件随后重配已经启动转发的 IMU 总线。
500 kbit/s 是通信波特率，与 500 Hz 主控制频率不同。

IMU 硬件参数位于 `ros2_control.xacro` 的 `hunter180_imu/hardware`：

| 参数 | 当前部署值 | 含义 |
| --- | --- | --- |
| `can_bitrate` | `500000` | 在创建 IMU、启动转发之前，只向目标总线传入波特率配置；`0` 表示保留网关已有时序 |
| `can_sample_point` | `0.75` | 可选采样点，范围 `(0, 1)`；仅在 `can_bitrate > 0` 时使用；保留网关时序时应删除此参数或设为 `0` |
| `initial_feedback_timeout_ms` | `5000` | 激活时等待完整有效反馈的时限，范围 `1–60000` ms；插件未配置时默认 1000 ms |

激活时先重新发送 IMU 全数据转发命令，在等待反馈期间每 200 ms 重试，
直到加速度、角速度和四元数同时有效。超过时限仍返回激活失败，状态接口不会用零值替代缺失数据。
超时日志列出接口、从站、总线、IMU 索引，以及 `acceleration`、`angular_velocity`、
`quaternion` 分别为 `valid`、`missing` 或 `invalid`，并记录启动转发的提交次数和结果。

旧网关固件曾拒绝在线时序设置。若配置阶段报出 `reading the staged timing back failed`
或 `CAN timing update was rejected by the slave`，需要先在网关配置正确的 500 kbit/s，
再将 `can_bitrate` 和 `can_sample_point` 都设为 `0`，保留网关设置。
配置异常会记录原始驱动错误并返回失败，不会静默忽略时序配置失败。

`control.json` 已从 `ec_joint_hardware/config/` 迁至 `ec_description/config/`，
由模型包安装并通过 `config_file` 传入。新版 SDK 按每个电机的 `canfd` 字段选择帧格式：
本配置的 `motorParams`、`motorParams1`、`motorParams2` 均显式设置 `"canfd": true`，
共 29 组参数（耦合关节的两轴会重复引用同一对电机）。旧 `canfd_enable` 参数不再生效。
适配器完整 JSON 传给 SDK，支持 `extend` 总线参数。

关节插件沿用 hunter130 的机械抱闸生命周期：配置时查询 SDK 抱闸能力并抱紧；
首次收到完整有效命令时先保持当前位置再松闸；停用、错误和关闭时尝试抱紧。
没有抱闸能力的关节不执行机械抱闸。具体行为见
[关节硬件说明](src/ec_joint_hardware/README.md)。
IMU 保留 Hunter180 原有安装转换：向量映射为 `(x, -z, y)`，姿态右乘绕 x 轴
-90° 的旋转并归一化，角速度输出为 rad/s。
激活后按单调时钟每秒打印一次机器人坐标系下的 IMU 数据：四元数 `quat_xyzw`、
角速度 `gyro_rad_s`（rad/s）、加速度 `accel_m_s2`（m/s²）。
`feedback=valid` 表示当前反馈有效；`feedback=holding_last` 表示本次反馈缺失或无效，打印的是保留的最后有效值。

## 策略

策略路径在 `controllers.yaml` 中配置为 `config/policy/hunter180_amp.onnx`，
相对路径从 `ec_controller` 的安装目录解析。来源为 AMP_mjlab 的
`logs/rsl_rl/hunter180_amp_baseline/2026-11-16_18-49-26_hunter180_flat_pd_v1_resume_49999`。
具体检查点编号、SHA-256、训练参数及推理对比结果记录在
[hunter180_amp.json](src/ec_controller/config/policy/hunter180_amp.json)。

- 每帧观测 78 维，历史 4 帧，按从旧到新的时间顺序拼接，ONNX 输入共 312 维。
- 输出 23 维动作，推理频率 50 Hz，控制更新频率 500 Hz，标称每 10 个控制周期推理一次。
- 观测依次为机身角速度、投影重力、速度指令、相对默认位置、关节速度、上一帧原始动作。
- 观测和动作均按 YAML 的物理关节顺序排列：左腿、右腿、腰、左臂、右臂。
- 经验归一化的均值、标准差和 epsilon=0.01 已封装在 ONNX；控制器直接传入原始观测。
- 训练未裁剪观测或动作，两个 `clip_*` 配置为 0 表示禁用。参考位置保留原值形成 PD 误差，
  SDK 按真实状态、输出方向和力矩限制执行保护。
- 配置时校验 ONNX 输入输出维度、关节顺序、历史长度、默认姿态、PD 增益和动作缩放；
  与模型元数据不一致时拒绝配置。

训练和现有硬件限扭存在差异，例如踝 pitch 训练为 200 N·m、硬件 `maxTor` 为 80 N·m，
踝 roll 训练为 100 N·m、硬件为 38 N·m；完整训练限扭见上述 JSON 的
`training_effort_limits`，硬件配置见 `ec_description/config/control.json`。
该分支保留已有硬件阈值，实机可能比训练更早饱和；离线推理一致性不代表实机步态已验证。

使用训练 Python 环境重新导出检查点（只读取训练目录）：

```bash
/home/milkyway/anaconda3/envs/mjlab/bin/python scripts/import_amp_policy.py \
  --run-dir /home/milkyway/HunterOpensource/AMP_mjlab/logs/rsl_rl/hunter180_amp_baseline/2026-11-16_18-49-26_hunter180_flat_pd_v1_resume_49999 \
  --checkpoint latest
```

也可将 `latest` 替换为指定的 `model_N.pt`。脚本从固定检查点导出 actor 和归一化参数，
独立于训练进程不断覆盖的 `policy.onnx`，并与 PyTorch 比较 256 组输入。
它更新策略、来源 JSON 和 C++ 测试参考；若训练配置改变，还需同步 `controllers.yaml`。

## 行走日志

WalkController 每次激活创建一个 `walk_log_N.csv`，停用时关闭并刷新文件。
`controllers.yaml` 中的 `walk_controller.ros__parameters.log_dir` 配置为
`/home/milkyway/log/hunter180`；空值时控制器也使用该目录。Joint SDK 的
`ros2_control.xacro` 硬件参数 `log_path` 同样配置为 `/home/milkyway/log/hunter180`。

目录不存在时会自动创建，也可提前创建：

```bash
mkdir -p /home/milkyway/log/hunter180
```

WalkController 的 CSV 和 Joint SDK 的 `*_command.csv.zstd`、`*_status.csv.zstd`
保存在该机器人目录下。

每个控制周期记录一行，当前为 500 Hz，列按以下顺序排列：

| 列 | 内容 |
|---|---|
| `time_ns` | 当前控制周期的 ROS 时间戳，单位 ns |
| `mpos0...` | 关节反馈位置，rad |
| `mvel0...` | 关节反馈速度，rad/s |
| `meff0...` | 关节反馈力矩，N·m |
| `cpos0...` | 控制器生成的位置命令，rad |
| `ckp0...` | 当前周期命令 Kp |
| `ckd0...` | 当前周期命令 Kd |

列编号对应该控制器 `joints` 的 YAML 顺序，关节数量自动适配配置。
记录的是控制器生成的命令；SDK 限幅后的电机侧数据由 `ec_joint_hardware` 的 `log_path` 独立记录。

## 离线验证

硬件单元测试使用 Fake/Mock 适配器，不连接真实电机：

```bash
colcon build --symlink-install --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=ON
colcon test --packages-select ec_joint_hardware ec_imu_hardware ec_controller \
  --ctest-args -R 'ec_joint_hardware|ec_imu_hardware|walk_controller_amp_test' --output-on-failure
colcon test-result --verbose
```

500 Hz 是控制器管理器的目标调度频率；实机运行频率仍取决于实时调度、通信和日志开销。
AMP 控制器测试加载实际插件和 ONNX，只提供内存中的状态/命令接口；检查六个连续策略帧、
重新激活后的历史复位，以及错误历史长度、关节顺序和默认姿态的拒绝配置。

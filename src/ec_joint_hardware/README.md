# ec_joint_hardware

`ec_joint_hardware/EcJointHardware` 是面向 `joint_sdk` 的 `ros2_control`
`SystemInterface`。一个硬件实例只加载一个既有格式的 `config_file` JSON；主臂、从臂和
灵巧手等需要独立控制器生命周期时，应配置为独立实例。实例内所有已选择关节的命令在同一
次 `write()` 中提交一次 `encos::DefaultJointManager::Commit()`。

## 硬件声明与配置

```xml
<hardware>
  <plugin>ec_joint_hardware/EcJointHardware</plugin>
  <param name="config_file">$(find robot_description)/config/slave_arm_config.json</param>
  <param name="adapter_type_1">$(arg adapter_type)</param>
  <param name="interface_name_1">$(arg interface_name)</param>
  <param name="status_max_life_cycle">3</param>
  <param name="canfd_enable">true</param>
</hardware>
```

`config_file` 和至少一组完整的 `adapter_type_N` /
`interface_name_N` 为必填项。`N` 是 adapter ID；如果一个硬件实例使用多个 adapter，
继续传入对应的 `_2`、`_3` 等参数。参数值必须非空，类型和接口名必须使用相同的 ID。

JSON 中的 adapter 模板通过 ID 解析：

```text
plugin-replace-me-1 -> adapter_type_1
enx-replace-me-1    -> interface_name_1
plugin-replace-me-2 -> adapter_type_2
enx-replace-me-2    -> interface_name_2
```

替换会递归处理 `adapters` 和关节 JSON 中完整匹配的字符串值；`busId`、`motorId` 等数值
字段保持原样。JSON 引用了未传入的 adapter ID，或替换后仍有完整占位符时，硬件会在创建
SDK adapter 和 joint 前返回配置错误。`status_max_life_cycle` 为可选参数，表示 adapter 反馈缓存
在没有新反馈时允许持续的最大控制周期数；默认值为 `3`。插件使用 `atoi` 读取该值，
转换结果必须大于零，否则 `on_init()` 失败。

JSON 根对象必须包含 `adapters` 和 `joints` 数组。适配器仅以 `type`、`name` DTO 读取；
每个关节 JSON 原样传给 `joint_sdk` 的 `AddJoint(json)`。硬件只校验 JSON 与
`ros2_control` 的关节名集合相同，不重复实现 SDK 的电机地址、限位或关节类型校验。

## CANFD 配置

`canfd_enable` 是可选的 hardware 参数，缺省为 `true`。接受 `true`、`True`、
`TRUE`、`1` 表示开启，`false`、`False`、`FALSE`、`0` 表示关闭；空值、其他拼写
或带前后空白的值会使 `on_init()` 返回 `ERROR`。

每次 `on_configure()` 重建 manager 后、创建关节前，插件调用 JointSDK 的
`EnableJointMotorCanfd()` 或 `DisableJointMotorCanfd()`，控制后续电机报文的 CANFD
帧标志。重新配置会再次应用该设置，配置成功日志会输出 `canfd_enable`。这是硬件配置
参数，修改后需要重新加载硬件描述并配置硬件。

`robot.xacro`、`master_robot.xacro`、`slave_robot.xacro` 均提供默认 `true` 的
`canfd_enable` 参数。主臂、从臂跟随它；左右灵巧手的 hardware 参数固定为 `false`。
例如直接展开从机器人描述并关闭臂部 CANFD：

```bash
xacro "$(ros2 pkg prefix --share robot_description)/xacro/slave_robot.xacro" \
  slave_arm_slave:=0 canfd_enable:=false
```

如果有hardware不是跟随全局模式应显式添加 `<param name="canfd_enable">false</param>`。
如果没有给hardware添加canfd_enable参数hardware会默认启动canfd.

## 控制器对接协议

控制器的控制模式由它**认领的命令接口集合**决定：认领哪个集合就是哪种模式（见下表），
在控制器激活时绑定，激活期间固定。中途不能切换模式——换模式就是换控制器，由 ros2_control
的控制器切换触发硬件的 `prepare_command_mode_switch()` 校验：任一关节的认领集合不完整、
跨模式混合或与其他控制器冲突时，整个切换被拒绝。

对接一个关节只需三步，但有一条铁律：**命令是单周期有效的**。

1. **认领接口**：按你的控制模式认领对应的固定接口集合（不多不少），以及需要观察的
   状态接口。
2. **每周期写完整命令**：每次 `write()` 结束后，硬件把**所有**命令接口重置为 `NaN`。
   因此每个需要控制的周期都必须重新写入全部必填参数。
3. **读状态**：从状态接口读取最新反馈（见状态接口表）。

行为与后果：

| 行为 | 后果 |
| --- | --- |
| 本周期必填接口全部不写（保持 `NaN`） | 控制器已绑定却未写命令即视为非法命令：该关节被跳过并记录错误日志；持续未写则每周期记录一次错误日志（提示控制器已绑定但未写入命令） |
| 必填接口缺失/非法（`NaN`、无穷或非法值） | 该关节本周期命令被跳过并记录一次错误日志（含关节名、模式与缺失/非法接口名），其余关节照常派发；`write()` 返回 `OK`（仅 SDK 异常时返回 `ERROR`） |
| 控制器停用后不再写命令 | 命令槽已复位为 `NaN`，不会遗留旧目标被重复执行 |
| 认领集合不完整/跨模式混合/多控制器冲突 | `prepare_command_mode_switch()` 拒绝，控制器切换整体失败 |

逐关节语义意味着：单个关节命令非法只影响该关节本周期不派发，其他关节的合法命令照常派发并 `Commit()`；`write()` 仅在 SDK 调用或 `Commit()` 抛出异常时才返回 `ERROR`。

## 控制模式

| 模式 | 认领的命令接口集合 | 下发的 SDK 调用 |
| --- | --- | --- |
| PVT（力位混控） | `position`、`velocity`、`effort`、`kp`、`kd`、`torque_mode` | `PVTControl(kp, kd, position, velocity, effort, torque_mode)` |
| 位置 | `position`、`velocity`、`current` | `PosControl(position, velocity, current, 1)` |
| 速度 | `velocity`、`current` | `SpdControl(velocity, current, 1)` |
| 电流 | `current` | `CurControl(current, 1)` |
| 扭矩 | `effort` | `TorControl(effort, 1)` |
| 停止 | `stop_current` | `Stop(FullBrake, stop_current, 1)` |

所有模式都请求完整电机反馈（SDK 调用的反馈参数固定为 `1`），控制器不需要选择反馈
类型。停止固定为全刹，电机固件不提供其他制动方式。

## 命令接口语义

| 接口 | 含义 | 单位 / 取值 |
| --- | --- | --- |
| `position` | 目标位置 | rad（关节坐标系） |
| `velocity` | 目标/最大速度 | rad/s |
| `effort` | 目标扭矩 | N·m |
| `current` | 最大/目标电流 | A |
| `kp` | PVT 比例增益 | 无量纲，SDK 按关节配置限幅 |
| `kd` | PVT 微分增益 | 无量纲，SDK 按关节配置限幅 |
| `torque_mode` | PVT 解算模式 | `0` 电机 PVT；`1` 上位机完整 PD；`2` 上位机 Kp、电机 Kd |
| `stop_current` | 停止时的制动电流 | A |

`torque_mode` 命令接口仍使用 ros2_control 的 `double` 数据槽，但仅接受精确的 `0`、`1`、`2`，
校验通过后转换为 SDK 的 `encos::PVTControlMode`，不再转为布尔值：

- `0` → `MotorPVT`：SDK 映射后直接下发电机 PVT。
- `1` → `HostTorque`：SDK 计算 `Kp * 位置误差 + Kd * 速度误差 + 前馈扭矩`，限扭并映射后下发扭矩控制。
- `2` → `HostKpMotorKd`：SDK 计算 Kp 位置反馈与前馈扭矩，换算后的 Kd 由电机本地闭环执行。

上述上位机解算语义适用于耦合关节；普通旋转关节 `RJoint` 忽略合法模式选择，
仍执行自身 PVT；`ContinuousJoint` 仍不支持 PVT。
耦合关节同批次的两轴需要选择相同的上位机模式，否则 SDK 回退普通 PVT。
非整数、越界、NaN 或无穷值按原有逐关节校验规则跳过，不阻断其他关节。
需要配套使用提供该枚举接口的新版 `joint_sdk`；旧 SDK 的布尔接口不再兼容。

## 状态接口

| 接口 | 含义 | 单位 |
| --- | --- | --- |
| `position` | 关节位置 | rad |
| `velocity` | 关节速度 | rad/s |
| `effort` | 关节扭矩 | N·m |
| `mos_temperature` | MOS 管温度 | °C |
| `motor_temperature` | 电机温度 | °C |
| `motor_error` | 电机错误码 | 数值枚举，见下表 |

`motor_error` 取值（`encos::MotorError`）：

| 值 | 含义 |
| --- | --- |
| `0` | 无错误 |
| `1` | 过温 |
| `2` | 过流 |
| `3` | 电压过高 |
| `4` | 电压过低 |
| `5` | 编码器错误 |
| `6` | 制动电压过高 |
| `7` | 驱动器错误 |
| `8` | 过温警告 |
| `255` | 无响应 |

错误码变化时硬件记录一次错误日志，恢复 `0` 时记录一次警告；持续故障不重复刷屏。
需要主动监控错误的控制器应订阅 `motor_error` 状态接口。

## 运行期错误语义

- **反馈看门狗**：电机反馈每帧刷新 3 个 `read()` 周期的生命；连续 3 个周期无新反馈则状态过期，`read()` 返回 `ERROR`（实际时长 = 3 / `update_rate`，如 500 Hz 时为
  6 ms）。控制器应把 `read()` 返回 `ERROR` 视为反馈链路故障。
- **SDK 异常**：控制调用或提交抛出异常时，硬件记录一次错误日志并返回 `ERROR`，本周期
  命令不会提交到总线。硬件不做重试或自动刹车等兜底动作；需要安全停车时，控制器应显式
  使用停止模式。

## 生命周期行为

- **激活准入**：激活前所有配置关节必须取得完整有效反馈（位置、速度、扭矩、两项温度
  均有限）；最多尝试 20 轮、每轮间隔 50 ms，超时或异常返回 `ERROR`。激活瞬间电机
  已带错误码时会记录错误日志，但不阻止激活。
- **停用语义**：`on_deactivate()` 只停止硬件读写，**不会发送停止命令**——停用不等于
  刹车。需要安全停车时，应先让停止控制器（或本控制器的停止逻辑）下发停止命令，再停用。

# ec_joint_hardware

`ec_joint_hardware/EcJointHardware` 是面向 `joint_sdk` 的 `ros2_control`
`SystemInterface`。当前配套版本为 Joint SDK 1.5.3 / EncosDriver 4.0.0。一个硬件实例只加载一个 `config_file` JSON；主臂、从臂和
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
</hardware>
```

`config_file` 为必填项。JSON 已使用固定适配器名称时，可省略所有编号参数；使用占位符时，
提供对应的 `adapter_type_N` / `interface_name_N`。`N` 是占位符编号，参数必须成对、
编号合法且值非空。SDK 的整数 `adapterId` 是适配器声明顺序索引，与占位符编号无关。

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
在没有新反馈时允许持续的最大控制周期数；默认值为 `10`。插件使用 `atoi` 读取该值，
转换结果必须大于零，否则 `on_init()` 失败。

JSON 根对象必须包含 `adapters` 和 `joints` 数组。适配器完整 JSON 按声明顺序交给 `CreateAdapter(json)`，保留 `extend` 和其中的 `buses`；
每个关节 JSON 原样传给 `joint_sdk` 的 `AddJoint(json)`。硬件只校验 JSON 与
`ros2_control` 的关节名集合相同，不重复实现 SDK 的电机地址、限位或关节类型校验。

## CAN 与 CAN FD 配置

帧格式由 JSON 的 `motorParams.canfd` 决定；双耦合分别使用 `motorParams1.canfd` 和
`motorParams2.canfd`。`true` 开启 CAN FD，`false` 或缺省使用普通 CAN。硬件不注入默认
字段，也不调用全局 CAN FD 开关。整数/字符串 `adapterId`、可选 `slaveId`、电机参数均
原样交给 SDK 校验。

硬件不读取、解析或告警旧参数 `canfd_enable`。迁移旧配置时，
需要 CAN FD 的电机必须在 JSON 中明确写入 `"canfd": true`；仅传旧 xacro/hardware 参数
已不能开启 CAN FD。本包不自动修改外部 JSON 或机器人描述。

适配器公共 CAN 参数与逐总线覆盖写在 `extend` 中，例如：

```json
{
  "type": "Ethercat",
  "name": "enp86s0",
  "extend": {
    "bitrate": 1000000,
    "sample_point": 0.75,
    "dbitrate": 5000000,
    "dsample_point": 0.875,
    "buses": {"0:7": {"bitrate": 500000, "sample_point": 0.75}}
  }
}
```

字段含义、支持范围、总线标识和复用行为由 SDK/驱动负责。硬件保留其异常边界，不丢弃
`extend`。当前安装驱动对仅支持旧 `MakeAdapter` 的插件（如 Fake）会告警并忽略扩展参数；
实际配置总线需使用支持 `MakeAdapterV2` 及对应字段的插件。整机格式的解析样例见
[test/fixtures/control_v2.json](test/fixtures/control_v2.json)；该文件来自参考整机配置，
测试只解析其内容或抽取关节配置后替换为 Fake，不连接实际 EtherCAT。

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

逐关节校验命令，不等待其他关节；尚未松闸的关节收到完整有效命令才松闸。

行为与后果：

| 行为 | 后果 |
| --- | --- |
| 本周期必填接口全部不写（保持 `NaN`） | 控制器已绑定却未写命令即视为非法命令：该关节被跳过并记录错误日志；持续未写则每周期记录一次错误日志（提示控制器已绑定但未写入命令） |
| 必填接口缺失/非法（`NaN`、无穷或非法值） | 该关节本周期命令被跳过并记录一次错误日志（含关节名与模式），其余关节照常派发；`write()` 返回 `OK`（SDK 异常或松闸失败时返回 `ERROR`） |
| 控制器停用后不再写命令 | 命令槽已复位为 `NaN`，不会遗留旧目标被重复执行 |
| 控制器切换后 | 切换回调保留命令缓存；进入 None 的抱闸关节在 write 中抱紧；各关节独立校验和派发有效命令，不等待全部关节首帧 |
| 认领集合不完整/跨模式混合/多控制器冲突 | `prepare_command_mode_switch()` 拒绝，控制器切换整体失败 |

逐关节语义意味着：单个关节命令非法只影响该关节本周期不派发，其他关节的合法命令照常派发并 `Commit()`；`write()` 在松闸返回失败、SDK 调用或 `Commit()` 抛出异常时返回 `ERROR`。

## 控制模式

| 模式 | 认领的命令接口集合 | 下发的 SDK 调用 |
| --- | --- | --- |
| None（未认领） | 无 | 无抱闸关节：`CurControl(0.0F, 1)`；有抱闸关节：调用 `EngageBrake` 抱紧，已抱紧时跳过 |
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
| `9` | 输出编码器错误 |
| `10` | FET 过温 |
| `11` | 超速 |
| `12` | 温度警告 |
| `255` | 无响应 |

错误码变化时硬件记录一次错误日志，恢复 `0` 时记录一次警告；持续故障不重复刷屏。
需要主动监控错误的控制器应订阅 `motor_error` 状态接口。

## 运行期错误语义

- **反馈看门狗**：反馈缓存按 `status_max_life_cycle`（默认 10）递减。缺失反馈时记录错误，保留该关节已有状态并继续读取其他关节，最终返回 `OK`；SDK 异常返回 `ERROR`。
- **SDK 异常**：控制调用或提交抛出异常时，硬件记录一次错误日志并返回 `ERROR`，本批次不能保证全部发送完成，提交也不代表物理原子执行。本次写调用不自动重试或调用机械抱闸；进入 `on_error()` 后按生命周期规则尝试抱紧。
- **独立派发**：不设首帧命令门。各关节沿用原有 if/else 校验，命令缺失或非法时记录错误并跳过，
  不阻塞其他关节。
- **松闸失败**：`Brake(false)` 返回 false 时回退并返回 Engaged，write 返回 `ERROR`，不派发该关节命令，后续可重试；抛异常时沿用外层错误处理。
  已成功松闸的记录与控制提交结果分开；发生控制或提交错误后由 `on_error()` 尝试抱紧。

## 生命周期行为

- **配置与能力缓存**：全部关节创建及配对完成后，在同一个循环中逐关节查询一次 `HasBrake()` 并保存，
  支持抱闸时立即调用 `Brake(true)`；失败即终止配置并返回 `ERROR`。
  硬件只处理 SDK 关节，不解析电机参数或配对键。仅用 `brake_state` 保存状态：
  0 无抱闸能力、1 已抱闸、2 已松闸、3 抱闸中、4 松闸中。运行期 0 不转换，
  其他状态仅按 `1→4→2→3→1` 转换；SDK 成功返回后才进入 1/2。
  配置时不支持关节保持 0，支持关节以 2 为初始请求起点，经 EngageBrake 初始抱闸成功后为 1；
  配置失败销毁绑定并清除状态。
  这是 SDK 调用状态记录，不是物理状态轮询结果。
- **激活**：每轮发送零电流命令请求反馈，检查完整有限反馈，最多 20 轮、轮间 2 ms。
  激活不松闸、不发送额外位置保持。
- **切换控制器**：`prepare` 仅校验接口；`perform` 记录新模式、保留命令缓存，
  切换回调不操作抱闸。仍有控制器认领的已松闸关节直接派发新命令；进入 None 的关节在下一次 write 抱紧。
- **首次松闸**：关节命令完整有效时，在 write 遍历中调用
  `ReleaseBrake(joint)`，返回 `BrakeState`；无抱闸或已松闸时静默返回当前状态；
  状态 3/4 在 write 中静默跳过，不派发控制命令；两个辅助函数在状态 3/4 均静默返回当前枚举状态。
  状态 1 进入 ReleaseBrake 后立即设为 4，再读取有效 SDK 位置、发送 `PosControl(当前位置)`，最后调用
  `Brake(false)`；该关节松闸成功后立即执行其控制命令，再处理下一个关节。保持与松闸使用直接发送，成功后恢复同步提交；后续周期不重复保持或松闸。
  位置缺失或非有限时，ReleaseBrake 回退并返回 Engaged，write 返回 `ERROR`，可在反馈恢复后重试。
  保持调用失败时返回 `ERROR`，不松闸。速度和电流上限使用 SDK 默认
  策略；读取反馈最多查询 5 次，仅在反馈缺失后的重试之间等待 1 ms，不增加保持状态。
  重试等待和 SDK 同步确认耗时会计入首帧 write。
- **无控制器**：支持抱闸的未认领关节由 write 调用 `EngageBrake` 抱紧，不发送控制命令或位置保持。
  抱紧成功后不重复调用，状态 3/4 静默等待；抱紧返回 false 则回退 Released，当次 write 返回 ERROR。
  EngageBrake 内部在实际抱闸前切换直接发送，成功后恢复同步模式，与 ReleaseBrake 一致，避免等待回包时命令滞留队列。
  无抱闸关节继续使用 `CurControl(0.0F, 1)` 零电流控制并请求反馈。
- **停用、错误和关闭**：停止普通控制，调用单关节 `EngageBrake(joint)`，状态 2 先变为 3，抱闸成功后变为 1。
  函数返回当前 `BrakeState`，退出回调将状态 0/1 视为完成，状态 3/4 视为未完成并返回错误；任一失败仍处理其他关节。
  抱闸返回 false 时回退并返回 Released，退出回调仍返回 `ERROR`，后续可重试；SDK 抛异常时仍保留过渡状态。机械抱闸与 `Stop(FullBrake)` 电气停止独立。

软件测试使用 Mock/Fake，未执行真实电机动作；实机需使用具备能力标记及固件支持的电机。
保持调用及发送顺序不等同于驱动已建立承载力矩。此流程要求关节支持 PosControl；
耦合关节仍依赖 SDK 的命令配对；逐关节保持、松闸、控制的顺序不能保证两轴保持在松闸前已下发。

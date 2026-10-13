/**
 * @file types.hpp
 * @brief 定义 ec_joint_hardware 的接口名称、控制模式和配置数据类型。
 */

#pragma once

#include <cstdint>
#include <map>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

/** @namespace encos::ec_joint_hardware
 * @brief joint_sdk 与 ros2_control 之间的硬件插件实现。
 */
namespace encos::ec_joint_hardware {

/** @brief 由 hardware 参数选择的单个 adapter。 */
struct AdapterSelection {
  std::string type;            ///< adapter 类型名称。
  std::string interface_name;  ///< adapter 使用的接口名称。
};

/** @brief 以 JSON 占位符后缀为键的 adapter 选择表。 */
using AdapterSelections = std::map<std::uint32_t, AdapterSelection>;

/**
 * @brief 关节当前绑定的控制模式。
 *
 * 控制模式由 ros2_control 控制器认领的完整命令接口集合确定。
 */
enum class ModeDefinition {
  None,      ///< 当前没有控制器认领该关节。
  Pvt,       ///< 力位混合控制模式。
  Position,  ///< 位置控制模式。
  Velocity,  ///< 速度控制模式。
  Current,   ///< 电流控制模式。
  Torque,    ///< 扭矩控制模式。
  Stop,      ///< 全制动停止模式。
};

/** @brief 每个关节必须导出的完整命令接口名称集合。 */
inline const std::vector<std::string> kCommandInterfaceNames = {
    "position", "velocity", "effort", "current", "kp", "kd", "torque_mode", "stop_current",
};

/** @brief 每个关节必须导出的完整状态接口名称集合。 */
inline const std::vector<std::string> kStateInterfaceNames = {
    "position", "velocity", "effort", "mos_temperature", "motor_temperature", "motor_error",
};

/**
 * @brief 控制模式到其完整命令接口集合的映射。
 *
 * `ModeDefinition::None` 映射为空集合，其他模式必须恰好认领对应集合。
 */
inline const std::map<ModeDefinition, std::vector<std::string>> kModeDefinitions = {
    {ModeDefinition::None, {}},
    {ModeDefinition::Pvt, {"position", "velocity", "effort", "kp", "kd", "torque_mode"}},
    {ModeDefinition::Position, {"position", "velocity", "current"}},
    {ModeDefinition::Velocity, {"velocity", "current"}},
    {ModeDefinition::Current, {"current"}},
    {ModeDefinition::Torque, {"effort"}},
    {ModeDefinition::Stop, {"stop_current"}},
};

/** @brief joint_sdk adapter 的包级配置数据。 */
struct AdapterConfig {
  std::string type;  ///< adapter 类型名称。
  std::string name;  ///< adapter 实例名称。
};

/** @brief 硬件插件从 JSON 文件读取的配置数据。 */
struct JointHardwareConfig {
  std::vector<AdapterConfig> adapters;  ///< 需要创建的 adapter 列表。
  std::vector<nlohmann::json> joints;   ///< 原样转交 joint_sdk 的关节配置列表。
};

}  // namespace encos::ec_joint_hardware

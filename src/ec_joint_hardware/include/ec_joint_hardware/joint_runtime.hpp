/**
 * @file joint_runtime.hpp
 * @brief 定义单个 ros2_control 关节的运行时状态与模式解析行为。
 */

#pragma once

#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "BaseJoint.h"
#include "ec_joint_hardware/types.hpp"

namespace encos::ec_joint_hardware {

/** @brief 硬件插件类的前向声明。 */
class EcJointHardware;

/** @brief 测试专用访问器的前向声明。 */
struct EcJointHardwareTestAccess;

/**
 * @brief 保存并维护单个关节的 SDK 句柄、控制模式、状态和单周期命令。
 *
 * 该类型是包内实现细节。命令值在每次硬件写周期结束后恢复为 NaN，模式只在
 * ros2_control 命令模式切换的 perform 阶段更新。
 */
class JointRuntime final {
 public:
  /**
   * @brief 创建未绑定控制模式的关节运行时。
   * @param[in] joint_name ros2_control 中的关节名称。
   */
  explicit JointRuntime(std::string joint_name);

  /** @brief 将全部命令槽重置为 NaN，使旧命令不能跨控制周期复用。 */
  void ResetCommands();

  /**
   * @brief 将 joint_sdk 状态写入对应的 ros2_control 状态槽。
   * @param[in] status 最新的完整关节反馈。
   */
  void StoreStatus(const encos::JointStatus& status);

  /**
   * @brief 根据完整的接口名称集合解析控制模式。
   * @param[in] claimed_interfaces 某个关节最终认领的纯接口名称集合。
   * @return 匹配的模式；非法集合返回 std::nullopt，空集合返回
   * ModeDefinition::None。
   */
  static std::optional<ModeDefinition> ResolveMode(const std::vector<std::string>& claimed_interfaces);

  /**
   * @brief 在当前模式上应用接口增量并解析切换后的控制模式。
   * @param[in] added_interface_keys 本次新增认领的全局 `joint/interface` 键。
   * @param[in] removed_interface_keys 本次移除认领的全局 `joint/interface` 键。
   * @return 合法的最终模式；非法接口组合返回 std::nullopt。
   * @throws std::out_of_range 当前模式不在 kModeDefinitions 中时抛出。
   */
  std::optional<ModeDefinition> ResolveMode(const std::vector<std::string>& added_interface_keys,
                                            const std::vector<std::string>& removed_interface_keys) const;

  /**
   * @brief 更新 perform 阶段确认的控制模式。
   * @param[in] new_mode 新的合法模式或 ModeDefinition::None。
   */
  void SetMode(ModeDefinition new_mode) noexcept;

  /**
   * @brief 获取当前绑定的控制模式。
   * @return 当前模式；ModeDefinition::None 表示未被控制器认领。
   */
  ModeDefinition GetMode() const noexcept;

 private:
  friend class EcJointHardware;             ///< 允许硬件插件直接绑定 ros2_control 数据槽。
  friend struct EcJointHardwareTestAccess;  ///< 允许硬件测试设置受控运行时状态。

  std::string name;                           ///< ros2_control 关节名称。
  encos::BaseJointPtr ptr;                    ///< on_configure 绑定的 joint_sdk 关节共享指针。
  ModeDefinition mode{ModeDefinition::None};  ///< 当前由 perform 回调确认的控制模式。

  double state_position{std::numeric_limits<double>::quiet_NaN()};           ///< 关节位置，单位 rad。
  double state_velocity{std::numeric_limits<double>::quiet_NaN()};           ///< 关节速度，单位 rad/s。
  double state_effort{std::numeric_limits<double>::quiet_NaN()};             ///< 关节扭矩，单位 N·m。
  double state_mos_temperature{std::numeric_limits<double>::quiet_NaN()};    ///< MOS 温度，单位 °C。
  double state_motor_temperature{std::numeric_limits<double>::quiet_NaN()};  ///< 电机温度，单位 °C。
  double state_motor_error{std::numeric_limits<double>::quiet_NaN()};        ///< MotorError 的数值枚举。

  double command_position{std::numeric_limits<double>::quiet_NaN()};      ///< 目标位置，单位 rad。
  double command_velocity{std::numeric_limits<double>::quiet_NaN()};      ///< 目标或最大速度，单位
                                                                          ///< rad/s。
  double command_effort{std::numeric_limits<double>::quiet_NaN()};        ///< 目标扭矩，单位 N·m。
  double command_current{std::numeric_limits<double>::quiet_NaN()};       ///< 目标或最大电流，单位 A。
  double command_kp{std::numeric_limits<double>::quiet_NaN()};            ///< PVT 比例增益。
  double command_kd{std::numeric_limits<double>::quiet_NaN()};            ///< PVT 微分增益。
  /// PVTControlMode 数值：0 电机 PVT，1 上位机 PD，2 上位机 Kp/电机 Kd。
  double command_torque_mode{std::numeric_limits<double>::quiet_NaN()};
  double command_stop_current{std::numeric_limits<double>::quiet_NaN()};  ///< 全制动电流，单位 A。
};

}  // namespace encos::ec_joint_hardware

/**
 * @file ec_joint_hardware.hpp
 * @brief 声明基于 joint_sdk 的 ros2_control SystemInterface 插件。
 */

#pragma once

#include <string>
#include <vector>

#include "JointManager.h"
#include "ec_joint_hardware/joint_runtime.hpp"
#include "ec_joint_hardware/types.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace encos::ec_joint_hardware {

/** @brief 测试专用访问器的前向声明。 */
struct EcJointHardwareTestAccess;

/** @brief 生命周期回调返回类型的包内别名。 */
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @brief 将 joint_sdk 关节和总线接入 ros2_control 的系统硬件插件。
 *
 * 插件按控制器认领的命令接口集合绑定每个关节的控制模式，并在每个写周期通过单次
 * DefaultJointManager::Commit() 提交本硬件实例的命令批次。
 */
class EcJointHardware final : public hardware_interface::SystemInterface {
 public:
  /** @brief 声明 ROS 2 共享指针类型和工厂辅助成员。 */
  RCLCPP_SHARED_PTR_DEFINITIONS(EcJointHardware)

  /**
   * @brief 初始化 ros2_control 硬件描述和关节运行时。
   * @param[in] info ros2_control 解析得到的硬件描述。
   * @return 初始化成功返回 SUCCESS，否则返回 ERROR。
   */
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  /**
   * @brief 加载 JSON、创建 adapter 和 SDK 关节并按名称完成绑定。
   * @param[in] previous_state 进入配置阶段前的生命周期状态。
   * @return 全部资源配置成功返回 SUCCESS，否则返回 ERROR。
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * @brief 等待所有关节取得完整有限反馈并进入 active 状态。
   * @param[in] previous_state 进入激活阶段前的生命周期状态。
   * @return 所有关节反馈有效返回 SUCCESS，超时或 SDK 异常返回 ERROR。
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * @brief 停止硬件读写而不主动发送电机制动命令。
   * @param[in] previous_state 进入停用阶段前的生命周期状态。
   * @return 始终返回 SUCCESS。
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * @brief 导出所有关节的六个状态接口。
   * @return 按 HardwareInfo 关节顺序排列的状态接口。
   */
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
   * @brief 导出所有关节的八个命令接口。
   * @return 按 HardwareInfo 关节顺序排列的命令接口。
   */
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /**
   * @brief 验证应用本次增量后的每关节认领集合能否解析为合法模式。
   * @param[in] start_interfaces 本次新增认领的全局接口键。
   * @param[in] stop_interfaces 本次移除认领的全局接口键。
   * @return 全部关节合法返回 OK，任一关节组合非法返回 ERROR。
   */
  hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                              const std::vector<std::string>& stop_interfaces) override;

  /**
   * @brief 记录经 prepare 验证的每关节控制模式。
   * @param[in] start_interfaces 本次新增认领的全局接口键。
   * @param[in] stop_interfaces 本次移除认领的全局接口键。
   * @return 模式解析并记录成功返回 OK，否则返回 ERROR。
   */
  hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                              const std::vector<std::string>& stop_interfaces) override;

  /**
   * @brief 从 joint_sdk 读取并发布所有 active 关节的最新状态。
   * @param[in] time 当前控制循环时间，本实现不使用。
   * @param[in] period 当前控制循环周期，本实现不使用。
   * @return 状态完整返回 OK，反馈缺失或 SDK 异常返回 ERROR。
   */
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  /**
   * @brief 校验并派发每个已绑定关节的单周期命令。
   * @param[in] time 当前控制循环时间，本实现不使用。
   * @param[in] period 当前控制循环周期，本实现不使用。
   * @return SDK 控制或提交异常返回 ERROR，其他情况返回 OK。
   */
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  friend struct EcJointHardwareTestAccess;  ///< 允许测试构造受控的生命周期和
                                            ///< SDK 状态。

  encos::DefaultJointManager manager_;    ///< 管理本插件实例的 adapter、关节和提交边界。
  std::vector<JointRuntime> joints_;      ///< 按 HardwareInfo 顺序保存的关节运行时。
  std::string config_file_;               ///< on_init 读取的 JSON 配置文件路径。
  std::string log_path_;                  ///< 可选的电机命令和状态日志目录。
  bool canfd_enable_{true};               ///< 配置阶段应用到关节电机的 CANFD 帧开关。
  int status_max_life_cycle_{10};         ///< 适配器反馈缓存允许持续的最大控制周期数。
  AdapterSelections adapter_selections_;  ///< hardware 参数选择的 adapter 表。
  rclcpp::Logger logger_{rclcpp::get_logger("EcJointHardware")};  ///< 插件日志器。
  bool configured_{false};  ///< 是否已成功完成当前生命周期的配置。
  bool active_{false};      ///< 是否已取得有效反馈并允许 read/write 访问 SDK。
};

}  // namespace encos::ec_joint_hardware

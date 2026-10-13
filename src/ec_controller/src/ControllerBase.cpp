// =============================================================================
//  Copyright (c) 2026 Phanerox. All rights reserved.
// =============================================================================

#include "ec_controller/ControllerBase.h"

#include <algorithm>
#include <unordered_map>

namespace ec
{

controller_interface::CallbackReturn ControllerBase::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ControllerBase::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  node->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>{});
  joint_names_ = node->get_parameter("joints").as_string_array();

  const size_t n = joint_names_.size();

  // Pre-allocate buffers
  joint_states_.resize(n);
  joint_commands_.resize(n);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ControllerBase::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  buildIndexMap();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ControllerBase::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

static constexpr size_t INVALID_IDX = static_cast<size_t>(-1);

void ControllerBase::buildIndexMap()
{
  const size_t n = joint_names_.size();
  state_pos_idx_.resize(n, INVALID_IDX);
  state_vel_idx_.resize(n, INVALID_IDX);
  state_eff_idx_.resize(n, INVALID_IDX);
  cmd_pos_idx_.resize(n, INVALID_IDX);
  cmd_vel_idx_.resize(n, INVALID_IDX);
  cmd_eff_idx_.resize(n, INVALID_IDX);
  cmd_kp_idx_.resize(n, INVALID_IDX);
  cmd_kd_idx_.resize(n, INVALID_IDX);
  cmd_torque_mode_idx_.resize(n, INVALID_IDX);

  // Build a lookup map for state interfaces: "prefix/interface" -> index
  std::unordered_map<std::string, size_t> state_map;
  state_map.reserve(state_interfaces_.size());
  for (size_t i = 0; i < state_interfaces_.size(); ++i) {
    state_map[state_interfaces_[i].get_prefix_name() + "/" +
      state_interfaces_[i].get_interface_name()] = i;
  }

  // Build a lookup map for command interfaces
  std::unordered_map<std::string, size_t> cmd_map;
  cmd_map.reserve(command_interfaces_.size());
  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    cmd_map[command_interfaces_[i].get_prefix_name() + "/" +
      command_interfaces_[i].get_interface_name()] = i;
  }

  for (size_t i = 0; i < n; ++i) {
    const auto & name = joint_names_[i];

    auto it = state_map.find(name + "/position");
    if (it != state_map.end()) {state_pos_idx_[i] = it->second;}

    it = state_map.find(name + "/velocity");
    if (it != state_map.end()) {state_vel_idx_[i] = it->second;}

    it = state_map.find(name + "/effort");
    if (it != state_map.end()) {state_eff_idx_[i] = it->second;}

    auto cit = cmd_map.find(name + "/position");
    if (cit != cmd_map.end()) {cmd_pos_idx_[i] = cit->second;}

    cit = cmd_map.find(name + "/velocity");
    if (cit != cmd_map.end()) {cmd_vel_idx_[i] = cit->second;}

    cit = cmd_map.find(name + "/effort");
    if (cit != cmd_map.end()) {cmd_eff_idx_[i] = cit->second;}

    cit = cmd_map.find(name + "/kp");
    if (cit != cmd_map.end()) {cmd_kp_idx_[i] = cit->second;}

    cit = cmd_map.find(name + "/kd");
    if (cit != cmd_map.end()) {cmd_kd_idx_[i] = cit->second;}

    cit = cmd_map.find(name + "/torque_mode");
    if (cit != cmd_map.end()) {cmd_torque_mode_idx_[i] = cit->second;}
  }
}

controller_interface::InterfaceConfiguration ControllerBase::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint_name : joint_names_) {
    config.names.push_back(joint_name + "/position");
    config.names.push_back(joint_name + "/velocity");
    config.names.push_back(joint_name + "/effort");
    config.names.push_back(joint_name + "/kp");
    config.names.push_back(joint_name + "/kd");
    config.names.push_back(joint_name + "/torque_mode");
  }

  return config;
}

controller_interface::InterfaceConfiguration ControllerBase::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint_name : joint_names_) {
    config.names.push_back(joint_name + "/position");
    config.names.push_back(joint_name + "/velocity");
    config.names.push_back(joint_name + "/effort");
  }

  return config;
}

controller_interface::return_type ControllerBase::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const size_t n = joint_names_.size();

  // 读取状态（直接索引访问，跳过不可用的接口）
  for (size_t i = 0; i < n; ++i) {
    if (state_pos_idx_[i] != INVALID_IDX) {
      auto val = state_interfaces_[state_pos_idx_[i]].get_optional();
      joint_states_[i].position = val.has_value() ? val.value() : 0.0;
    }
    if (state_vel_idx_[i] != INVALID_IDX) {
      auto val = state_interfaces_[state_vel_idx_[i]].get_optional();
      joint_states_[i].velocity = val.has_value() ? val.value() : 0.0;
    }
    if (state_eff_idx_[i] != INVALID_IDX) {
      auto val = state_interfaces_[state_eff_idx_[i]].get_optional();
      joint_states_[i].effort = val.has_value() ? val.value() : 0.0;
    }
  }

  // 重置命令
  for (size_t i = 0; i < n; ++i) {
    joint_commands_[i] = JointCommand{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  }

  // 调用子类更新
  onUpdate(time, period);

  // 写入命令（直接索引访问，跳过不可用的接口）
  for (size_t i = 0; i < n; ++i) {
    if (cmd_pos_idx_[i] != INVALID_IDX) {
      (void)command_interfaces_[cmd_pos_idx_[i]].set_value(joint_commands_[i].position);
    }
    if (cmd_vel_idx_[i] != INVALID_IDX) {
      (void)command_interfaces_[cmd_vel_idx_[i]].set_value(joint_commands_[i].velocity);
    }
    if (cmd_eff_idx_[i] != INVALID_IDX) {
      (void)command_interfaces_[cmd_eff_idx_[i]].set_value(joint_commands_[i].effort);
    }
    if (cmd_kp_idx_[i] != INVALID_IDX) {
      (void)command_interfaces_[cmd_kp_idx_[i]].set_value(joint_commands_[i].kp);
    }
    if (cmd_kd_idx_[i] != INVALID_IDX) {
      (void)command_interfaces_[cmd_kd_idx_[i]].set_value(joint_commands_[i].kd);
    }
    if (cmd_torque_mode_idx_[i] != INVALID_IDX) {
      (void)command_interfaces_[cmd_torque_mode_idx_[i]].set_value(joint_commands_[i].torque_mode);
    }
  }

  return controller_interface::return_type::OK;
}

} // namespace ec

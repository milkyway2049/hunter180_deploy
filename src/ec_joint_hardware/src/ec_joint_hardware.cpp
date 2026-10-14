#include "ec_joint_hardware/ec_joint_hardware.hpp"

#include "JointSdkLog.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include "ec_joint_hardware/utils.hpp"

namespace encos::ec_joint_hardware {
namespace {

constexpr std::size_t kActivationFeedbackMaxAttempts = 20;  ///< 激活期等待关节反馈的最大尝试次数。
constexpr auto kActivationFeedbackRetryDelay = std::chrono::milliseconds{2};  ///< 激活期等待关节反馈的尝试间隔。

}  // namespace

CallbackReturn EcJointHardware::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Suppress joint_sdk warnings (e.g. "Joint protection: ... minimum kd")
  encos::JointSdkLogger::Get()->SetLevel(encos::LogLevel::Error);

  const auto config_file = info_.hardware_parameters.find("config_file");
  if (config_file == info_.hardware_parameters.end() || config_file->second.empty()) {
    RCLCPP_ERROR(logger_, "missing required non-empty hardware parameter 'config_file'");
    return CallbackReturn::ERROR;
  }
  config_file_ = config_file->second;

  const auto log_path = info_.hardware_parameters.find("log_path");
  log_path_ = log_path == info_.hardware_parameters.end() ? "" : log_path->second;

  canfd_enable_ = true;
  const auto canfd_enable = info_.hardware_parameters.find("canfd_enable");
  if (canfd_enable != info_.hardware_parameters.end()) {
    if (canfd_enable->second == "true" || canfd_enable->second == "True" || canfd_enable->second == "TRUE" ||
        canfd_enable->second == "1") {
      canfd_enable_ = true;
    } else if (canfd_enable->second == "false" || canfd_enable->second == "False" || canfd_enable->second == "FALSE" ||
               canfd_enable->second == "0") {
      canfd_enable_ = false;
    } else {
      RCLCPP_ERROR(logger_, "hardware parameter 'canfd_enable' must be a boolean, got '%s'",
                   canfd_enable->second.c_str());
      return CallbackReturn::ERROR;
    }
  }

  try {
    adapter_selections_ = ParseAdapterSelections(info_.hardware_parameters);
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(logger_, "invalid numbered adapter parameters: %s", exception.what());
    return CallbackReturn::ERROR;
  }

  const auto status_max_life_cycle = info_.hardware_parameters.find("status_max_life_cycle");
  if (status_max_life_cycle != info_.hardware_parameters.end()) {
    const auto& text = status_max_life_cycle->second;
    const int parsed_value = std::atoi(text.c_str());
    if (parsed_value <= 0) {
      RCLCPP_ERROR(logger_, "hardware parameter 'status_max_life_cycle' must be positive, got '%s'", text.c_str());
      return CallbackReturn::ERROR;
    }
    status_max_life_cycle_ = parsed_value;
  }

  if (info_.joints.empty()) {
    RCLCPP_ERROR(logger_, "hardware must define at least one joint");
    return CallbackReturn::ERROR;
  }

  joints_.clear();
  joints_.reserve(info_.joints.size());
  for (const auto& joint : info_.joints) {
    joints_.emplace_back(joint.name);
  }
  configured_ = false;
  active_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn EcJointHardware::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  configured_ = false;
  active_ = false;
  for (auto& joint : joints_) {
    joint.ptr.reset();
  }

  try {
    const auto config = LoadJointHardwareConfig(config_file_, adapter_selections_);

    std::unordered_set<std::string> hardware_joint_names;
    hardware_joint_names.reserve(joints_.size());
    for (const auto& joint : joints_) {
      hardware_joint_names.insert(joint.name);
    }

    std::unordered_set<std::string> config_joint_names;
    config_joint_names.reserve(config.joints.size());
    for (const auto& joint_config : config.joints) {
      config_joint_names.insert(joint_config.at("name").get<std::string>());
    }
    if (config_joint_names != hardware_joint_names) {
      throw std::runtime_error("JSON joint names must match ros2_control joint names");
    }

    manager_ = encos::DefaultJointManager{};
    if (canfd_enable_) {
      manager_.EnableJointMotorCanfd();
    } else {
      manager_.DisableJointMotorCanfd();
    }
    for (const auto& adapter : config.adapters) {
      manager_.CreateAdapter(adapter.type, adapter.name);
    }
    manager_.SetAdaptersStatusMaxLifeCycle(status_max_life_cycle_);

    for (const auto& joint_config : config.joints) {
      const auto name = joint_config.at("name").get<std::string>();
      auto joint = manager_.AddJoint(joint_config);
      if (!joint) {
        throw std::runtime_error("joint_sdk returned null for joint '" + name + "'");
      }

      const auto runtime = std::find_if(joints_.begin(), joints_.end(),
                                        [&name](const JointRuntime& entry) { return entry.name == name; });
      if (runtime == joints_.end()) {
        throw std::runtime_error("joint_sdk did not create joint '" + name + "'");
      }
      runtime->ptr = joint;
    }

    if (!log_path_.empty()) {
      manager_.EnableLog(std::filesystem::path(log_path_));
      RCLCPP_INFO(logger_, "enabled motor data logging at '%s'", log_path_.c_str());
    }
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(logger_, "failed to configure joint hardware: %s", exception.what());
    return CallbackReturn::ERROR;
  }

  configured_ = true;
  RCLCPP_INFO(logger_, "configured %zu joints, canfd_enable=%s", joints_.size(), canfd_enable_ ? "true" : "false");
  return CallbackReturn::SUCCESS;
}

CallbackReturn EcJointHardware::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  if (!configured_) {
    RCLCPP_ERROR(logger_, "cannot activate before configure");
    return CallbackReturn::ERROR;
  }

  active_ = false;
  // 先取消软同步：激活期的反馈请求走直发模式；软同步由控制期 write() 的首个
  // Commit 自然进入
  manager_.SetAdaptersSyncMode(false);
  for (std::size_t attempt = 0; attempt < kActivationFeedbackMaxAttempts; ++attempt) {
    std::vector<encos::JointStatus> statuses;
    statuses.reserve(joints_.size());
    bool all_valid = true;

    for (const auto& joint : joints_) {
      if (!joint.ptr) {
        RCLCPP_ERROR(logger_, "cannot activate: joint '%s' is not configured", joint.name.c_str());
        return CallbackReturn::ERROR;
      }

      try {
        auto status = joint.ptr->GetStatus(true, 0);
        if (!status) {
          all_valid = false;
        } else if (!std::isfinite(status->pos) || !std::isfinite(status->spd) || !std::isfinite(status->tor) ||
                   !std::isfinite(status->mosTemperature) || !std::isfinite(status->motorTemperature)) {
          // error 是 MotorError 枚举，不存在非有限值，无需检查
          all_valid = false;
        } else {
          statuses.push_back(*status);
        }
      } catch (const std::exception& exception) {
        RCLCPP_ERROR(logger_, "activation feedback failed for joint '%s': %s", joint.name.c_str(), exception.what());
        return CallbackReturn::ERROR;
      }
    }

    if (all_valid) {
      for (std::size_t index = 0; index < joints_.size(); ++index) {
        // 激活瞬间已存在的电机故障无法被 read 的变化检测覆盖，在此补报
        if (statuses.at(index).error != encos::MotorError::NoError) {
          RCLCPP_ERROR(logger_, "joint '%s' activated with motor error: %s (%u)", joints_[index].name.c_str(),
                       MotorErrorName(statuses.at(index).error), static_cast<unsigned int>(statuses.at(index).error));
        }
        joints_[index].StoreStatus(statuses.at(index));
      }
      active_ = true;
      RCLCPP_INFO(logger_, "activated with valid feedback from %zu joints", joints_.size());
      return CallbackReturn::SUCCESS;
    }

    if (attempt + 1U < kActivationFeedbackMaxAttempts) {
      std::this_thread::sleep_for(kActivationFeedbackRetryDelay);
    }
  }

  RCLCPP_ERROR(logger_, "activation timed out waiting for valid feedback from all joints");
  return CallbackReturn::ERROR;
}

CallbackReturn EcJointHardware::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
  active_ = false;
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> EcJointHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(joints_.size() * kStateInterfaceNames.size());
  for (auto& joint : joints_) {
    interfaces.emplace_back(joint.name, "position", &joint.state_position);
    interfaces.emplace_back(joint.name, "velocity", &joint.state_velocity);
    interfaces.emplace_back(joint.name, "effort", &joint.state_effort);
    interfaces.emplace_back(joint.name, "mos_temperature", &joint.state_mos_temperature);
    interfaces.emplace_back(joint.name, "motor_temperature", &joint.state_motor_temperature);
    interfaces.emplace_back(joint.name, "motor_error", &joint.state_motor_error);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> EcJointHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(joints_.size() * kCommandInterfaceNames.size());
  for (auto& joint : joints_) {
    interfaces.emplace_back(joint.name, "position", &joint.command_position);
    interfaces.emplace_back(joint.name, "velocity", &joint.command_velocity);
    interfaces.emplace_back(joint.name, "effort", &joint.command_effort);
    interfaces.emplace_back(joint.name, "current", &joint.command_current);
    interfaces.emplace_back(joint.name, "kp", &joint.command_kp);
    interfaces.emplace_back(joint.name, "kd", &joint.command_kd);
    interfaces.emplace_back(joint.name, "torque_mode", &joint.command_torque_mode);
    interfaces.emplace_back(joint.name, "stop_current", &joint.command_stop_current);
  }
  return interfaces;
}

hardware_interface::return_type EcJointHardware::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces) {
  for (const auto& joint : joints_) {
    if (!joint.ResolveMode(start_interfaces, stop_interfaces).has_value()) {
      RCLCPP_ERROR(logger_, "joint '%s' claimed command interfaces do not match any control mode", joint.name.c_str());
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type EcJointHardware::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces) {
  std::vector<ModeDefinition> resolved_modes;
  resolved_modes.reserve(joints_.size());
  for (const auto& joint : joints_) {
    const auto resolved = joint.ResolveMode(start_interfaces, stop_interfaces);
    if (!resolved.has_value()) {
      RCLCPP_ERROR(logger_, "joint '%s' claimed command interfaces do not match any control mode", joint.name.c_str());
      return hardware_interface::return_type::ERROR;
    }
    resolved_modes.push_back(*resolved);
  }

  for (std::size_t index = 0; index < joints_.size(); ++index) {
    joints_[index].SetMode(resolved_modes[index]);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type EcJointHardware::read(const rclcpp::Time& /*time*/,
                                                      const rclcpp::Duration& /*period*/) {
  if (!active_) {
    return hardware_interface::return_type::OK;
  }

  for (auto& joint : joints_) {
    try {
      const auto status = joint.ptr->GetStatus(false, 1);
      if (!status) {
        RCLCPP_ERROR(logger_, "missing feedback for joint '%s'", joint.name.c_str());
        return hardware_interface::return_type::OK;
      }
      // 错误码变化时才打日志：持续故障不刷屏，恢复也有迹可循
      if (status->error != encos::MotorError::NoError &&
          static_cast<double>(static_cast<unsigned int>(status->error)) != joint.state_motor_error) {
        RCLCPP_ERROR(logger_, "motor error for joint '%s': %s (%u)", joint.name.c_str(), MotorErrorName(status->error),
                     static_cast<unsigned int>(status->error));
      }
      if (status->error == encos::MotorError::NoError &&
          joint.state_motor_error != static_cast<double>(static_cast<unsigned int>(encos::MotorError::NoError))) {
        RCLCPP_WARN(logger_, "motor error cleared for joint '%s'", joint.name.c_str());
      }
      joint.StoreStatus(*status);
    } catch (const std::exception& exception) {
      RCLCPP_ERROR(logger_, "read failed for joint '%s': %s", joint.name.c_str(), exception.what());
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type EcJointHardware::write(const rclcpp::Time& /*time*/,
                                                       const rclcpp::Duration& /*period*/) {
  const auto finish = [this](hardware_interface::return_type result) {
    for (auto& joint : joints_) {
      joint.ResetCommands();
    }
    return result;
  };
  if (!active_) {
    return finish(hardware_interface::return_type::OK);
  }

  try {
    for (auto& joint : joints_) {
      const auto mode = joint.GetMode();
      switch (mode) {
        case ModeDefinition::None:
          joint.ptr->PVTControl(0.0F, 0.1F, 0.0F, 0.0F, 0.0F);
          continue;
        case ModeDefinition::Pvt:
          if (std::isfinite(joint.command_position) && std::isfinite(joint.command_velocity) &&
              std::isfinite(joint.command_effort) && std::isfinite(joint.command_kp) &&
              std::isfinite(joint.command_kd) && std::isfinite(joint.command_torque_mode) &&
              (joint.command_torque_mode == 0.0 || joint.command_torque_mode == 1.0 ||
               joint.command_torque_mode == 2.0)) {
            joint.ptr->PVTControl(static_cast<float>(joint.command_kp), static_cast<float>(joint.command_kd),
                                  static_cast<float>(joint.command_position),
                                  static_cast<float>(joint.command_velocity), static_cast<float>(joint.command_effort),
                                  static_cast<encos::PVTControlMode>(static_cast<int>(joint.command_torque_mode)));
          } else {
            RCLCPP_ERROR(logger_,
                         "invalid command for joint '%s', mode %s: missing or "
                         "invalid interfaces: position, velocity, "
                         "effort, kp, kd, torque_mode",
                         joint.name.c_str(), ModeName(mode));
          }
          break;
        case ModeDefinition::Position:
          if (std::isfinite(joint.command_position) && std::isfinite(joint.command_velocity) &&
              std::isfinite(joint.command_current)) {
            joint.ptr->PosControl(static_cast<float>(joint.command_position),
                                  static_cast<float>(joint.command_velocity), static_cast<float>(joint.command_current),
                                  1);
          } else {
            RCLCPP_ERROR(logger_,
                         "invalid command for joint '%s', mode %s: missing or "
                         "invalid interfaces: position, velocity, current",
                         joint.name.c_str(), ModeName(mode));
          }
          break;
        case ModeDefinition::Velocity:
          if (std::isfinite(joint.command_velocity) && std::isfinite(joint.command_current)) {
            joint.ptr->SpdControl(static_cast<float>(joint.command_velocity), static_cast<float>(joint.command_current),
                                  1);
          } else {
            RCLCPP_ERROR(logger_,
                         "invalid command for joint '%s', mode %s: missing or "
                         "invalid interfaces: velocity, current",
                         joint.name.c_str(), ModeName(mode));
          }
          break;
        case ModeDefinition::Current:
          if (std::isfinite(joint.command_current)) {
            joint.ptr->CurControl(static_cast<float>(joint.command_current), 1);
          } else {
            RCLCPP_ERROR(logger_,
                         "invalid command for joint '%s', mode %s: missing or "
                         "invalid interface: current",
                         joint.name.c_str(), ModeName(mode));
          }
          break;
        case ModeDefinition::Torque:
          if (std::isfinite(joint.command_effort)) {
            joint.ptr->TorControl(static_cast<float>(joint.command_effort), 1);
          } else {
            RCLCPP_ERROR(logger_,
                         "invalid command for joint '%s', mode %s: missing or "
                         "invalid interface: effort",
                         joint.name.c_str(), ModeName(mode));
          }
          break;
        case ModeDefinition::Stop:
          if (std::isfinite(joint.command_stop_current)) {
            joint.ptr->Stop(encos::MotorStopMode::FullBrake, static_cast<float>(joint.command_stop_current), 1);
          } else {
            RCLCPP_ERROR(logger_,
                         "invalid command for joint '%s', mode %s: missing or "
                         "invalid interface: stop_current",
                         joint.name.c_str(), ModeName(mode));
          }
          break;
      }
    }

    manager_.Commit();
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(logger_, "write failed: %s", exception.what());
    return finish(hardware_interface::return_type::ERROR);
  }
  return finish(hardware_interface::return_type::OK);
}

}  // namespace encos::ec_joint_hardware

PLUGINLIB_EXPORT_CLASS(encos::ec_joint_hardware::EcJointHardware, hardware_interface::SystemInterface)

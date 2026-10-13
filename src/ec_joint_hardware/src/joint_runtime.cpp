#include "ec_joint_hardware/joint_runtime.hpp"

#include <algorithm>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "ec_joint_hardware/types.hpp"
#include "ec_joint_hardware/utils.hpp"

namespace encos::ec_joint_hardware {

JointRuntime::JointRuntime(std::string joint_name) : name(std::move(joint_name)) {}

void JointRuntime::ResetCommands() {
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  command_position = nan;
  command_velocity = nan;
  command_effort = nan;
  command_current = nan;
  command_kp = nan;
  command_kd = nan;
  command_torque_mode = nan;
  command_stop_current = nan;
}

void JointRuntime::StoreStatus(const encos::JointStatus& status) {
  state_position = status.pos;
  state_velocity = status.spd;
  state_effort = status.tor;
  state_mos_temperature = status.mosTemperature;
  state_motor_temperature = status.motorTemperature;
  state_motor_error = static_cast<double>(static_cast<unsigned int>(status.error));
}

std::optional<ModeDefinition> JointRuntime::ResolveMode(const std::vector<std::string>& claimed_interfaces) {
  const auto definition = std::find_if(
      kModeDefinitions.begin(), kModeDefinitions.end(),
      [&claimed_interfaces](const auto& entry) { return SameInterfaces(claimed_interfaces, entry.second); });
  if (definition == kModeDefinitions.end()) {
    return std::nullopt;
  }
  return definition->first;
}

std::optional<ModeDefinition> JointRuntime::ResolveMode(const std::vector<std::string>& added_interface_keys,
                                                        const std::vector<std::string>& removed_interface_keys) const {
  auto claimed_interfaces = kModeDefinitions.at(mode);

  for (const auto& key : removed_interface_keys) {
    const auto parts = SplitInterfaceKey(key);
    const auto joint_name = parts.first;
    const auto interface_name = parts.second;
    if (joint_name != name) {
      continue;
    }
    claimed_interfaces.erase(
        std::remove_if(claimed_interfaces.begin(), claimed_interfaces.end(),
                       [interface_name](const std::string& claimed) { return claimed == interface_name; }),
        claimed_interfaces.end());
  }

  for (const auto& key : added_interface_keys) {
    const auto parts = SplitInterfaceKey(key);
    const auto joint_name = parts.first;
    const auto interface_name = parts.second;
    if (joint_name != name) {
      continue;
    }
    const auto existing =
        std::find_if(claimed_interfaces.begin(), claimed_interfaces.end(),
                     [interface_name](const std::string& claimed) { return claimed == interface_name; });
    if (existing == claimed_interfaces.end()) {
      claimed_interfaces.emplace_back(interface_name);
    }
  }

  return ResolveMode(claimed_interfaces);
}

void JointRuntime::SetMode(ModeDefinition new_mode) noexcept { mode = new_mode; }

ModeDefinition JointRuntime::GetMode() const noexcept { return mode; }

}  // namespace encos::ec_joint_hardware

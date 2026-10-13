#include "ec_joint_hardware/utils.hpp"

#include <algorithm>
#include <charconv>
#include <fstream>
#include <nlohmann/json.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace encos::ec_joint_hardware {

std::pair<std::string_view, std::string_view> SplitInterfaceKey(const std::string& key) {
  const auto separator = key.rfind('/');
  const std::string_view view = key;
  if (separator == std::string::npos) {
    return {view, {}};
  }
  return {view.substr(0, separator), view.substr(separator + 1)};
}

bool SameInterfaces(const std::vector<std::string>& lhs, const std::vector<std::string>& rhs) {
  return lhs.size() == rhs.size() && std::is_permutation(lhs.begin(), lhs.end(), rhs.begin(), rhs.end());
}

const char* ModeName(ModeDefinition mode) {
  switch (mode) {
    case ModeDefinition::None:
      return "none";
    case ModeDefinition::Pvt:
      return "pvt";
    case ModeDefinition::Position:
      return "position";
    case ModeDefinition::Velocity:
      return "velocity";
    case ModeDefinition::Current:
      return "current";
    case ModeDefinition::Torque:
      return "torque";
    case ModeDefinition::Stop:
      return "stop";
  }
  return "unknown";
}

const char* MotorErrorName(encos::MotorError error) {
  switch (error) {
    case encos::MotorError::NoError:
      return "none";
    case encos::MotorError::OverTemperature:
      return "over_temperature";
    case encos::MotorError::OverCurrent:
      return "over_current";
    case encos::MotorError::VoltageHigh:
      return "voltage_high";
    case encos::MotorError::VoltageLow:
      return "voltage_low";
    case encos::MotorError::EncoderError:
      return "encoder_error";
    case encos::MotorError::BrakeVoltageHigh:
      return "brake_voltage_high";
    case encos::MotorError::DriverError:
      return "driver_error";
    case encos::MotorError::OverTemperatureWarning:
      return "over_temperature_warning";
    case encos::MotorError::NoResponse:
      return "no_response";
  }
  return "unknown";
}

void from_json(const nlohmann::json& json, AdapterConfig& adapter) {
  json.at("type").get_to(adapter.type);
  json.at("name").get_to(adapter.name);
}

namespace {

constexpr std::string_view kAdapterTypePrefix = "adapter_type_";
constexpr std::string_view kInterfaceNamePrefix = "interface_name_";
constexpr std::string_view kPluginMarkerPrefix = "plugin-replace-me-";
constexpr std::string_view kInterfaceMarkerPrefix = "enx-replace-me-";

bool StartsWith(const std::string& value, std::string_view prefix) {
  return value.size() >= prefix.size() && value.compare(0, prefix.size(), prefix) == 0;
}

std::optional<std::uint32_t> TryParseNumberedSuffix(const std::string& value, std::string_view prefix) {
  if (!StartsWith(value, prefix)) {
    return std::nullopt;
  }

  const std::string_view suffix = std::string_view{value}.substr(prefix.size());
  if (suffix.empty()) {
    return std::nullopt;
  }

  std::uint32_t id = 0;
  const auto result = std::from_chars(suffix.data(), suffix.data() + suffix.size(), id);
  if (result.ec != std::errc{} || result.ptr != suffix.data() + suffix.size()) {
    return std::nullopt;
  }
  return id;
}

void ValidateAdapterSelections(const AdapterSelections& adapter_selections) {
  for (const auto& [id, selection] : adapter_selections) {
    if (selection.type.empty() || selection.interface_name.empty()) {
      throw std::runtime_error("adapter selection " + std::to_string(id) + " must have non-empty type and interface");
    }
  }
}

}  // namespace

AdapterSelections ParseAdapterSelections(const std::unordered_map<std::string, std::string>& hardware_parameters) {
  AdapterSelections adapter_selections;
  for (const auto& [name, value] : hardware_parameters) {
    const bool is_type = StartsWith(name, kAdapterTypePrefix);
    const bool is_interface = StartsWith(name, kInterfaceNamePrefix);
    if (!is_type && !is_interface) {
      continue;
    }
    if (value.empty()) {
      throw std::runtime_error("hardware parameter '" + name + "' must be non-empty");
    }

    const auto prefix = is_type ? kAdapterTypePrefix : kInterfaceNamePrefix;
    const std::string_view suffix = std::string_view{name}.substr(prefix.size());
    if (suffix.empty()) {
      throw std::runtime_error("parameter '" + name + "' must have a decimal adapter id");
    }

    std::uint32_t id = 0;
    const auto result = std::from_chars(suffix.data(), suffix.data() + suffix.size(), id);
    if (result.ec != std::errc{} || result.ptr != suffix.data() + suffix.size()) {
      throw std::runtime_error("parameter '" + name + "' has an invalid adapter id");
    }
    if (is_type) {
      adapter_selections[id].type = value;
    } else {
      adapter_selections[id].interface_name = value;
    }
  }

  if (adapter_selections.empty()) {
    throw std::runtime_error("at least one numbered adapter selection is required");
  }
  ValidateAdapterSelections(adapter_selections);
  return adapter_selections;
}

JointHardwareConfig LoadJointHardwareConfig(const std::string& config_file,
                                            const AdapterSelections& adapter_selections) {
  std::ifstream input(config_file);
  if (!input.is_open()) {
    throw std::runtime_error("failed to load '" + config_file + "': cannot open file");
  }

  try {
    ValidateAdapterSelections(adapter_selections);
    nlohmann::json root;
    input >> root;
    if (!root.is_object()) {
      throw std::runtime_error("root must be an object");
    }

    auto replace_adapter_markers = [&](auto&& self, nlohmann::json& value) -> void {
      if (value.is_string()) {
        const auto& marker_value = value.get_ref<const std::string&>();
        std::optional<std::uint32_t> marker_id;
        bool selects_type = false;
        if (StartsWith(marker_value, kPluginMarkerPrefix)) {
          marker_id = TryParseNumberedSuffix(marker_value, kPluginMarkerPrefix);
          selects_type = true;
        } else if (StartsWith(marker_value, kInterfaceMarkerPrefix)) {
          marker_id = TryParseNumberedSuffix(marker_value, kInterfaceMarkerPrefix);
        }

        if (!marker_id) {
          return;
        }

        const auto selection = adapter_selections.find(*marker_id);
        if (selection == adapter_selections.end()) {
          throw std::runtime_error("JSON adapter marker references unknown adapter id " + std::to_string(*marker_id));
        }
        value = selects_type ? selection->second.type : selection->second.interface_name;
        return;
      }

      if (value.is_array()) {
        for (auto& child : value) {
          self(self, child);
        }
        return;
      }

      if (value.is_object()) {
        for (auto& [key, child] : value.items()) {
          (void)key;
          self(self, child);
        }
      }
    };
    replace_adapter_markers(replace_adapter_markers, root);

    JointHardwareConfig config;
    root.at("adapters").get_to(config.adapters);
    root.at("joints").get_to(config.joints);
    return config;
  } catch (const nlohmann::json::exception& exception) {
    throw std::runtime_error("failed to load '" + config_file + "': " + exception.what());
  } catch (const std::runtime_error& exception) {
    throw std::runtime_error("failed to load '" + config_file + "': " + exception.what());
  }
}

}  // namespace encos::ec_joint_hardware

#pragma once

#include <encos/adapter/base_adapter.h>
#include <encos/imu/imu.h>

#include <array>
#include <cstddef>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/sensor_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <string>
#include <vector>

namespace encos::ec_imu_hardware {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class EcImuHardware final : public hardware_interface::SensorInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(EcImuHardware)

  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  static constexpr std::size_t kStateInterfaceCount = 10;

  bool RefreshState();
  void ResetState();

  encos::BaseAdapterPtr adapter_;
  encos::Imu* imu_{nullptr};
  std::array<double, kStateInterfaceCount> state_{};
  rclcpp::Logger logger_{rclcpp::get_logger("EcImuHardware")};
  std::string sensor_name_;
  std::string adapter_type_;
  std::string interface_name_;
  int imu_idx_{0};
  int slave_idx_{0};
  int bus_idx_{0};
  bool configured_{false};
  bool active_{false};
};

}  // namespace encos::ec_imu_hardware

#include "ec_imu_hardware/ec_imu_hardware.hpp"

#include <Eigen/Geometry>
#include <encos/bus/bus.h>
#include <encos/encos_driver.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <string_view>
#include <thread>

namespace encos::ec_imu_hardware {
namespace {

constexpr std::size_t kImuStateInterfaceCount = 10;
constexpr std::array<std::string_view, kImuStateInterfaceCount> kImuStateInterfaceNames = {
    "orientation.x",         "orientation.y",         "orientation.z",      "orientation.w",
    "angular_velocity.x",    "angular_velocity.y",    "angular_velocity.z", "linear_acceleration.x",
    "linear_acceleration.y", "linear_acceleration.z",
};
constexpr auto kActivationPollDelay = std::chrono::milliseconds{5};
constexpr auto kInitialFeedbackTimeout = std::chrono::milliseconds{1000};
constexpr double kDegreesToRadians = 3.141592653589793238462643383279502884 / 180.0;
constexpr double kMinQuaternionNorm = 1.0e-12;
constexpr int kMaxImuIndex = std::numeric_limits<std::uint16_t>::max();
constexpr int kMaxSlaveIndex = std::numeric_limits<int>::max() >> 16;
constexpr int kMaxBusIndex = 0xFF;

bool IsEthercatAdapter(const std::string& adapter_type) {
  return adapter_type == "Ethercat" || adapter_type == "EthercatIGH";
}

bool ConvertStatusToState(const encos::ImuStatus& status,
                          std::array<double, kImuStateInterfaceCount>& state) {
  if (!status.quaternion || !status.angular_velocity || !status.acceleration) {
    return false;
  }

  // 原始数据
  const double w_orig = static_cast<double>(status.quaternion->qw);
  const double x_orig = static_cast<double>(status.quaternion->qx);
  const double y_orig = static_cast<double>(status.quaternion->qy);
  const double z_orig = static_cast<double>(status.quaternion->qz);
  const double ax_raw = static_cast<double>(status.acceleration->x);
  const double ay_raw = static_cast<double>(status.acceleration->y);
  const double az_raw = static_cast<double>(status.acceleration->z);
  const double wx_raw = static_cast<double>(status.angular_velocity->x) * kDegreesToRadians;
  const double wy_raw = static_cast<double>(status.angular_velocity->y) * kDegreesToRadians;
  const double wz_raw = static_cast<double>(status.angular_velocity->z) * kDegreesToRadians;

  // 四元数坐标变换: IMU(x前,z右,y上) -> Robot(x前,y左,z上)
  // q_robot = q_imu * inverse(q_RI), q_RI = (-√2/2, 0, 0, √2/2)
  // inverse(q_RI) = (√2/2, 0, 0, √2/2)
  const double s = 0.7071067811865476;
  const double qx_t = -s;
  const double qy_t = 0.0;
  const double qz_t = 0.0;
  const double qw_t = s;

  double w_new = w_orig * qw_t - x_orig * qx_t - y_orig * qy_t - z_orig * qz_t;
  double x_new = w_orig * qx_t + x_orig * qw_t + y_orig * qz_t - z_orig * qy_t;
  double y_new = w_orig * qy_t - x_orig * qz_t + y_orig * qw_t + z_orig * qx_t;
  double z_new = w_orig * qz_t + x_orig * qy_t - y_orig * qx_t + z_orig * qw_t;

  // 归一化
  const double qnorm = std::sqrt(w_new * w_new + x_new * x_new + y_new * y_new + z_new * z_new);
  if (!std::isfinite(qnorm) || qnorm <= kMinQuaternionNorm) {
    return false;
  }
  w_new /= qnorm;
  x_new /= qnorm;
  y_new /= qnorm;
  z_new /= qnorm;

  // 向量坐标变换: robot_x=imu_x, robot_y=-imu_z, robot_z=imu_y
  Eigen::Matrix<double, kImuStateInterfaceCount, 1> candidate;
  candidate << x_new, y_new, z_new, w_new,
      wx_raw, -wz_raw, wy_raw,
      ax_raw, -az_raw, ay_raw;
  if (!candidate.allFinite()) {
    return false;
  }

  std::copy_n(candidate.data(), state.size(), state.begin());
  return true;
}

}  // namespace

CallbackReturn EcImuHardware::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SensorInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  adapter_ = nullptr;
  imu_ = nullptr;
  configured_ = false;
  active_ = false;
  ResetState();

  try {
    if (info_.sensors.size() != 1) {
      return CallbackReturn::ERROR;
    }

    const auto& sensor = info_.sensors.front();
    sensor_name_ = sensor.name;
    adapter_type_ = info_.hardware_parameters.at("adapter_type");
    interface_name_ = info_.hardware_parameters.at("interface_name");
    imu_idx_ = std::stoi(sensor.parameters.at("imu_idx"));
    slave_idx_ = 0;
    bus_idx_ = 0;
    if (IsEthercatAdapter(adapter_type_)) {
      slave_idx_ = std::stoi(sensor.parameters.at("slave_idx"));
      bus_idx_ = std::stoi(sensor.parameters.at("bus_idx"));
    }
    if (imu_idx_ < 0 || imu_idx_ > kMaxImuIndex ||
        (IsEthercatAdapter(adapter_type_) &&
         (slave_idx_ < 0 || slave_idx_ > kMaxSlaveIndex || bus_idx_ < 0 || bus_idx_ > kMaxBusIndex))) {
      return CallbackReturn::ERROR;
    }
  } catch (...) {
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn EcImuHardware::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  configured_ = false;
  active_ = false;
  imu_ = nullptr;
  ResetState();

  try {
    adapter_ = encos::MakeAdapter(adapter_type_, interface_name_, logger_.get_name());
    encos::Bus* bus = adapter_ ? (IsEthercatAdapter(adapter_type_) ? adapter_->GetBus(slave_idx_, bus_idx_)
                                                                    : adapter_->GetBus(0))
                               : nullptr;
    imu_ = bus ? bus->GetImu(imu_idx_) : nullptr;
  } catch (...) {
  }

  if (!imu_) {
    RCLCPP_ERROR(logger_, "failed to configure IMU hardware");
    return CallbackReturn::ERROR;
  }

  configured_ = true;
  return CallbackReturn::SUCCESS;
}

CallbackReturn EcImuHardware::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  if (!configured_) {
    RCLCPP_ERROR(logger_, "IMU hardware is not configured");
    return CallbackReturn::ERROR;
  }

  active_ = false;
  ResetState();
  const auto deadline = std::chrono::steady_clock::now() + kInitialFeedbackTimeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (RefreshState()) {
      active_ = true;
      return CallbackReturn::SUCCESS;
    }
    std::this_thread::sleep_for(kActivationPollDelay);
  }

  RCLCPP_ERROR(logger_, "IMU feedback unavailable during activation");
  return CallbackReturn::ERROR;
}

CallbackReturn EcImuHardware::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
  active_ = false;
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> EcImuHardware::export_state_interfaces() {
  static_assert(kStateInterfaceCount == kImuStateInterfaceCount);

  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(kStateInterfaceCount);
  for (std::size_t index = 0; index < kStateInterfaceCount; ++index) {
    state_interfaces.emplace_back(sensor_name_, std::string{kImuStateInterfaceNames[index]}, &state_.at(index));
  }
  return state_interfaces;
}

hardware_interface::return_type EcImuHardware::read(const rclcpp::Time& /*time*/,
                                                     const rclcpp::Duration& /*period*/) {
  if (!active_) {
    return hardware_interface::return_type::OK;
  }
  RefreshState();
  return hardware_interface::return_type::OK;
}

bool EcImuHardware::RefreshState() {
  if (!imu_) {
    return false;
  }

  std::array<double, kStateInterfaceCount> candidate{};
  if (!ConvertStatusToState(imu_->GetStatus(), candidate)) {
    return false;
  }

  state_ = candidate;
  return true;
}

void EcImuHardware::ResetState() {
  state_.fill(std::numeric_limits<double>::quiet_NaN());
}

}  // namespace encos::ec_imu_hardware

PLUGINLIB_EXPORT_CLASS(encos::ec_imu_hardware::EcImuHardware, hardware_interface::SensorInterface)

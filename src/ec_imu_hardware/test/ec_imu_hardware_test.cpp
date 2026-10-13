#include "ec_imu_hardware/ec_imu_hardware.hpp"

#include <encos/encos_driver.h>
#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_return_values.hpp"

#ifndef IMU_HARDWARE_TEST_PLUGIN_PATH
#define IMU_HARDWARE_TEST_PLUGIN_PATH ""
#endif

extern "C" bool EcImuHardwareTestCanPluginInjectRawMessage(const char* interface_name, int bus_idx,
                                                           std::uint32_t can_id, const std::uint8_t* data,
                                                           std::size_t size);

namespace encos::ec_imu_hardware {
namespace {

constexpr std::array<const char*, 10> kExpectedImuStateInterfaceNames = {
    "orientation.x",         "orientation.y",         "orientation.z",      "orientation.w",
    "angular_velocity.x",    "angular_velocity.y",    "angular_velocity.z", "linear_acceleration.x",
    "linear_acceleration.y", "linear_acceleration.z",
};
constexpr auto kImuStatusSettleDelay = std::chrono::milliseconds{80};
constexpr auto kImuStatusExpiryDelay = std::chrono::milliseconds{1100};
constexpr double kDegreesToRadians = 3.141592653589793238462643383279502884 / 180.0;
constexpr double kSqrtHalf = 0.7071067811865475244;
constexpr std::size_t kXAxis = 0;
constexpr std::size_t kYAxis = 1;
constexpr std::size_t kZAxis = 2;

class ScopedCanInterface {
public:
  explicit ScopedCanInterface(std::string name) : name_(std::move(name)) {}
  ~ScopedCanInterface() { encos::EncosDriverManager::Instance().DestroyAdapterByInterfaceName(name_); }

  const std::string& name() const { return name_; }

private:
  std::string name_;
};

hardware_interface::InterfaceInfo MakeInterface(const std::string& name) {
  hardware_interface::InterfaceInfo interface;
  interface.name = name;
  return interface;
}

hardware_interface::ComponentInfo MakeImuSensor(const std::string& name, int imu_idx = 0, int slave_idx = 0,
                                                int bus_idx = 0) {
  hardware_interface::ComponentInfo sensor;
  sensor.name = name;
  sensor.parameters["imu_idx"] = std::to_string(imu_idx);
  sensor.parameters["slave_idx"] = std::to_string(slave_idx);
  sensor.parameters["bus_idx"] = std::to_string(bus_idx);
  sensor.state_interfaces.reserve(kExpectedImuStateInterfaceNames.size());
  for (const auto* interface_name : kExpectedImuStateInterfaceNames) {
    sensor.state_interfaces.push_back(MakeInterface(interface_name));
  }
  return sensor;
}

hardware_interface::HardwareInfo MakeHardwareInfo(const std::string& adapter_type = "EthercatIGH",
                                                  const std::string& interface_name = "0") {
  hardware_interface::HardwareInfo info;
  info.name = "ec_imu_hardware_hw";
  info.type = "sensor";
  info.hardware_plugin_name = "ec_imu_hardware/EcImuHardware";
  info.hardware_parameters["adapter_type"] = adapter_type;
  info.hardware_parameters["interface_name"] = interface_name;
  info.sensors.push_back(MakeImuSensor("pelvis_imu"));
  return info;
}

hardware_interface::HardwareInfo MakeCanHardwareInfo(const std::string& interface_name, int imu_idx = 0) {
  EXPECT_EQ(setenv("ENCOS_PLUGIN_PATH", IMU_HARDWARE_TEST_PLUGIN_PATH, 1), 0);
  auto info = MakeHardwareInfo("Can", interface_name);
  info.sensors.front().parameters["imu_idx"] = std::to_string(imu_idx);
  info.sensors.front().parameters.erase("slave_idx");
  info.sensors.front().parameters.erase("bus_idx");
  return info;
}

void SetMountAngles(hardware_interface::ComponentInfo& sensor, const std::string& roll_deg,
                    const std::string& pitch_deg, const std::string& yaw_deg) {
  sensor.parameters["mount_roll_deg"] = roll_deg;
  sensor.parameters["mount_pitch_deg"] = pitch_deg;
  sensor.parameters["mount_yaw_deg"] = yaw_deg;
}

void SetMountAngles(hardware_interface::ComponentInfo& sensor, double roll_deg, double pitch_deg, double yaw_deg) {
  SetMountAngles(sensor, std::to_string(roll_deg), std::to_string(pitch_deg), std::to_string(yaw_deg));
}

void WriteU16Le(std::array<std::uint8_t, 8>& target, std::size_t offset, std::uint16_t value) {
  target.at(offset) = static_cast<std::uint8_t>(value & 0xFFu);
  target.at(offset + 1) = static_cast<std::uint8_t>(value >> 8u);
}

void WriteBitsLe(std::array<std::uint8_t, 8>& target, std::uint8_t start_bit, std::uint8_t bit_len,
                 std::uint32_t value) {
  for (std::uint8_t bit = 0; bit < bit_len; ++bit) {
    if ((value & (1u << bit)) == 0) {
      continue;
    }
    const auto absolute_bit = static_cast<std::uint8_t>(start_bit + bit);
    target.at(absolute_bit / 8u) =
        static_cast<std::uint8_t>(target.at(absolute_bit / 8u) | (1u << (absolute_bit % 8u)));
  }
}

void InjectRawMessage(const std::string& interface_name, std::uint32_t can_id, const std::vector<std::uint8_t>& data) {
  ASSERT_TRUE(EcImuHardwareTestCanPluginInjectRawMessage(interface_name.c_str(), 0, can_id, data.data(), data.size()));
}

void InjectRawMessage(const std::string& interface_name, std::uint32_t can_id,
                      const std::array<std::uint8_t, 8>& data) {
  ASSERT_TRUE(EcImuHardwareTestCanPluginInjectRawMessage(interface_name.c_str(), 0, can_id, data.data(), data.size()));
}

void InjectCompleteImuFeedback(const std::string& interface_name, std::uint16_t y_acceleration_raw = 0x7D64,
                               std::uint16_t z_acceleration_raw = 0x7C9C) {
  std::array<std::uint8_t, 8> angular_data{};
  WriteBitsLe(angular_data, 0, 20, 512000);
  WriteBitsLe(angular_data, 20, 20, 512128);
  WriteBitsLe(angular_data, 40, 20, 511872);

  std::array<std::uint8_t, 8> quaternion_data{};
  WriteU16Le(quaternion_data, 0, 32766);
  WriteU16Le(quaternion_data, 2, 0);
  WriteU16Le(quaternion_data, 4, 65535);
  WriteU16Le(quaternion_data, 6, 32768);

  InjectRawMessage(interface_name, 0x0CF02D59,
                   std::vector<std::uint8_t>{0x00, 0x7D, static_cast<std::uint8_t>(y_acceleration_raw & 0xFFu),
                                             static_cast<std::uint8_t>(y_acceleration_raw >> 8u),
                                             static_cast<std::uint8_t>(z_acceleration_raw & 0xFFu),
                                             static_cast<std::uint8_t>(z_acceleration_raw >> 8u)});
  InjectRawMessage(interface_name, 0x0CF02A59, angular_data);
  InjectRawMessage(interface_name, 0x0CF02959, std::vector<std::uint8_t>{0x00, 0x7D, 0x80, 0x7D, 0x80, 0x7C});
  InjectRawMessage(interface_name, 0x0CF03059, quaternion_data);
  std::this_thread::sleep_for(kImuStatusSettleDelay);
}

void InjectUnitAxisMotion(const std::string& interface_name, std::size_t axis,
                          std::uint16_t quaternion_x_raw = 32768) {
  constexpr std::uint16_t kAccelerationZeroRaw = 32000;
  constexpr std::uint16_t kAccelerationOneRaw = 32100;
  constexpr std::uint32_t kAngularVelocityZeroRaw = 512000;
  constexpr std::uint32_t kAngularVelocityOneDegPerSecondRaw = 512128;

  std::array<std::uint16_t, 3> acceleration_raw{
      kAccelerationZeroRaw,
      kAccelerationZeroRaw,
      kAccelerationZeroRaw,
  };
  acceleration_raw.at(axis) = kAccelerationOneRaw;

  std::array<std::uint8_t, 8> acceleration_data{};
  WriteU16Le(acceleration_data, 0, acceleration_raw.at(kXAxis));
  WriteU16Le(acceleration_data, 2, acceleration_raw.at(kYAxis));
  WriteU16Le(acceleration_data, 4, acceleration_raw.at(kZAxis));

  std::array<std::uint32_t, 3> angular_velocity_raw{
      kAngularVelocityZeroRaw,
      kAngularVelocityZeroRaw,
      kAngularVelocityZeroRaw,
  };
  angular_velocity_raw.at(axis) = kAngularVelocityOneDegPerSecondRaw;

  std::array<std::uint8_t, 8> angular_data{};
  WriteBitsLe(angular_data, 0, 20, angular_velocity_raw.at(kXAxis));
  WriteBitsLe(angular_data, 20, 20, angular_velocity_raw.at(kYAxis));
  WriteBitsLe(angular_data, 40, 20, angular_velocity_raw.at(kZAxis));

  std::array<std::uint8_t, 8> quaternion_data{};
  WriteU16Le(quaternion_data, 0, 65535);
  WriteU16Le(quaternion_data, 2, quaternion_x_raw);
  WriteU16Le(quaternion_data, 4, 32768);
  WriteU16Le(quaternion_data, 6, 32768);

  InjectRawMessage(interface_name, 0x0CF02D59, acceleration_data);
  InjectRawMessage(interface_name, 0x0CF02A59, angular_data);
  InjectRawMessage(interface_name, 0x0CF02959, std::vector<std::uint8_t>{0x00, 0x7D, 0x00, 0x7D, 0x00, 0x7D});
  InjectRawMessage(interface_name, 0x0CF03059, quaternion_data);
  std::this_thread::sleep_for(kImuStatusSettleDelay);
}

void InjectIdentityOrientationWithXAxisMotion(const std::string& interface_name) {
  InjectUnitAxisMotion(interface_name, kXAxis);
}

std::vector<double> ReadStateValues(const std::vector<hardware_interface::StateInterface>& state_interfaces) {
  std::vector<double> values;
  values.reserve(state_interfaces.size());
  for (const auto& state_interface : state_interfaces) {
    const auto value = state_interface.get_optional<double>();
    EXPECT_TRUE(value.has_value());
    values.push_back(value.value_or(0.0));
  }
  return values;
}

}  // namespace

TEST(EcImuHardwareTest, RejectsMissingRequiredHardwareParameters) {
  EcImuHardware hardware;
  auto info = MakeHardwareInfo();
  info.hardware_parameters.erase("adapter_type");
  EXPECT_EQ(hardware.on_init(info), CallbackReturn::ERROR);

  EcImuHardware second_hardware;
  info = MakeHardwareInfo();
  info.hardware_parameters.erase("interface_name");
  EXPECT_EQ(second_hardware.on_init(info), CallbackReturn::ERROR);
}

TEST(EcImuHardwareTest, RejectsMissingRequiredSensorParameter) {
  EcImuHardware hardware;
  auto info = MakeHardwareInfo();
  info.sensors.front().parameters.erase("imu_idx");

  EXPECT_EQ(hardware.on_init(info), CallbackReturn::ERROR);
}

TEST(EcImuHardwareTest, AcceptsAdapterTypes) {
  EcImuHardware can_hardware;
  auto can_info = MakeHardwareInfo("Can", "can0");
  can_info.sensors.front().parameters.erase("slave_idx");
  can_info.sensors.front().parameters.erase("bus_idx");
  EXPECT_EQ(can_hardware.on_init(can_info), CallbackReturn::SUCCESS);

  EcImuHardware ethercat_hardware;
  EXPECT_EQ(ethercat_hardware.on_init(MakeHardwareInfo("Ethercat", "eth0")), CallbackReturn::SUCCESS);

  EcImuHardware ethercat_igh_hardware;
  EXPECT_EQ(ethercat_igh_hardware.on_init(MakeHardwareInfo("EthercatIGH", "0")), CallbackReturn::SUCCESS);

  EcImuHardware other_hardware;
  auto other_info = MakeHardwareInfo("Other", "other0");
  other_info.sensors.front().parameters.erase("slave_idx");
  other_info.sensors.front().parameters.erase("bus_idx");
  EXPECT_EQ(other_hardware.on_init(other_info), CallbackReturn::SUCCESS);
}

TEST(EcImuHardwareTest, RejectsMissingEthercatBusParameter) {
  EcImuHardware hardware;
  auto info = MakeHardwareInfo("EthercatIGH", "0");
  info.sensors.front().parameters.erase("slave_idx");
  EXPECT_EQ(hardware.on_init(info), CallbackReturn::ERROR);
}

TEST(EcImuHardwareTest, RejectsMultipleSensors) {
  EcImuHardware hardware;
  auto info = MakeHardwareInfo();
  info.sensors.push_back(MakeImuSensor("torso_imu", 1, 0, 1));

  EXPECT_EQ(hardware.on_init(info), CallbackReturn::ERROR);
}

TEST(EcImuHardwareTest, RejectsIndicesOutsideDriverApiRanges) {
  EcImuHardware negative_imu_hardware;
  auto negative_imu_info = MakeHardwareInfo();
  negative_imu_info.sensors.front().parameters["imu_idx"] = "-1";
  EXPECT_EQ(negative_imu_hardware.on_init(negative_imu_info), CallbackReturn::ERROR);

  EcImuHardware large_imu_hardware;
  auto large_imu_info = MakeHardwareInfo();
  large_imu_info.sensors.front().parameters["imu_idx"] = "65536";
  EXPECT_EQ(large_imu_hardware.on_init(large_imu_info), CallbackReturn::ERROR);

  EcImuHardware negative_slave_hardware;
  auto negative_slave_info = MakeHardwareInfo();
  negative_slave_info.sensors.front().parameters["slave_idx"] = "-1";
  EXPECT_EQ(negative_slave_hardware.on_init(negative_slave_info), CallbackReturn::ERROR);

  EcImuHardware large_slave_hardware;
  auto large_slave_info = MakeHardwareInfo();
  large_slave_info.sensors.front().parameters["slave_idx"] = "32768";
  EXPECT_EQ(large_slave_hardware.on_init(large_slave_info), CallbackReturn::ERROR);

  EcImuHardware large_bus_hardware;
  auto large_bus_info = MakeHardwareInfo();
  large_bus_info.sensors.front().parameters["bus_idx"] = "256";
  EXPECT_EQ(large_bus_hardware.on_init(large_bus_info), CallbackReturn::ERROR);

  EcImuHardware negative_bus_hardware;
  auto negative_bus_info = MakeHardwareInfo();
  negative_bus_info.sensors.front().parameters["bus_idx"] = "-1";
  EXPECT_EQ(negative_bus_hardware.on_init(negative_bus_info), CallbackReturn::ERROR);
}

TEST(EcImuHardwareTest, ExportsExpectedStateInterfacesAsNaNBeforeFirstRead) {
  EcImuHardware hardware;

  ASSERT_EQ(hardware.on_init(MakeHardwareInfo()), CallbackReturn::SUCCESS);
  const auto state_interfaces = hardware.export_state_interfaces();

  ASSERT_EQ(state_interfaces.size(), kExpectedImuStateInterfaceNames.size());
  std::vector<std::string> names;
  names.reserve(state_interfaces.size());
  for (const auto& state_interface : state_interfaces) {
    names.push_back(state_interface.get_prefix_name() + "/" + state_interface.get_interface_name());
    const auto value = state_interface.get_optional<double>();
    ASSERT_TRUE(value.has_value());
    EXPECT_TRUE(std::isnan(value.value()));
  }

  std::vector<std::string> expected_names;
  expected_names.reserve(kExpectedImuStateInterfaceNames.size());
  for (const auto* interface_name : kExpectedImuStateInterfaceNames) {
    expected_names.push_back(std::string{"pelvis_imu/"} + interface_name);
  }
  EXPECT_EQ(names, expected_names);
}

TEST(EcImuHardwareTest, ReadReturnsOkWhileInactive) {
  EcImuHardware hardware;
  ASSERT_EQ(hardware.on_init(MakeHardwareInfo()), CallbackReturn::SUCCESS);

  EXPECT_EQ(hardware.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)), hardware_interface::return_type::OK);
}

TEST(EcImuHardwareTest, ActivateFailsWhenInitialFeedbackIsMissing) {
  ScopedCanInterface can_interface{"ec_imu_hardware_test_no_initial_feedback"};
  EcImuHardware hardware;
  ASSERT_EQ(hardware.on_init(MakeCanHardwareInfo(can_interface.name())), CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);

  EXPECT_EQ(hardware.on_activate(rclcpp_lifecycle::State{}), CallbackReturn::ERROR);
}

TEST(EcImuHardwareTest, ReadKeepsLastValidStateAfterFeedbackExpires) {
  ScopedCanInterface can_interface{"ec_imu_hardware_test_stale_feedback"};
  EcImuHardware hardware;
  ASSERT_EQ(hardware.on_init(MakeCanHardwareInfo(can_interface.name())), CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);

  InjectCompleteImuFeedback(can_interface.name());
  ASSERT_EQ(hardware.on_activate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  const auto state_interfaces = hardware.export_state_interfaces();
  const auto activated_values = ReadStateValues(state_interfaces);
  ASSERT_EQ(activated_values.size(), kExpectedImuStateInterfaceNames.size());
  for (const auto value : activated_values) {
    EXPECT_TRUE(std::isfinite(value));
  }

  std::this_thread::sleep_for(kImuStatusExpiryDelay);
  EXPECT_EQ(hardware.read(rclcpp::Time{0, 0, RCL_ROS_TIME}, rclcpp::Duration::from_seconds(0.01)),
            hardware_interface::return_type::OK);
  EXPECT_EQ(hardware.read(rclcpp::Time{0, 200000000, RCL_ROS_TIME}, rclcpp::Duration::from_seconds(0.01)),
            hardware_interface::return_type::OK);
  EXPECT_EQ(ReadStateValues(state_interfaces), activated_values);

  InjectCompleteImuFeedback(can_interface.name(), 0x7DC8, 0x7C38);
  EXPECT_EQ(hardware.read(rclcpp::Time{0, 400000000, RCL_ROS_TIME}, rclcpp::Duration::from_seconds(0.01)),
            hardware_interface::return_type::OK);
  const auto recovered_values = ReadStateValues(state_interfaces);
  ASSERT_EQ(recovered_values.size(), kExpectedImuStateInterfaceNames.size());
  EXPECT_NEAR(recovered_values.at(8), 2.0, 1.0e-5);
  EXPECT_NEAR(recovered_values.at(9), -2.0, 1.0e-5);
}

TEST(EcImuHardwareTest, NormalizesQuaternionFeedback) {
  ScopedCanInterface can_interface{"ec_imu_hardware_test_normalized_quaternion"};
  EcImuHardware hardware;
  ASSERT_EQ(hardware.on_init(MakeCanHardwareInfo(can_interface.name())), CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);

  InjectUnitAxisMotion(can_interface.name(), kXAxis, 65535);
  ASSERT_EQ(hardware.on_activate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);

  const auto values = ReadStateValues(hardware.export_state_interfaces());
  ASSERT_EQ(values.size(), kExpectedImuStateInterfaceNames.size());
  EXPECT_NEAR(values.at(0), kSqrtHalf, 1.0e-3);
  EXPECT_NEAR(values.at(1), 0.0, 1.0e-3);
  EXPECT_NEAR(values.at(2), 0.0, 1.0e-3);
  EXPECT_NEAR(values.at(3), kSqrtHalf, 1.0e-3);
}

TEST(EcImuHardwareTest, IgnoresMountParametersAndPublishesRawSensorFrame) {
  ScopedCanInterface can_interface{"ec_imu_hardware_test_mount_yaw"};
  EcImuHardware hardware;
  auto info = MakeCanHardwareInfo(can_interface.name());
  SetMountAngles(info.sensors.front(), 0.0, 0.0, 90.0);
  ASSERT_EQ(hardware.on_init(info), CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);

  InjectIdentityOrientationWithXAxisMotion(can_interface.name());
  ASSERT_EQ(hardware.on_activate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);

  const auto values = ReadStateValues(hardware.export_state_interfaces());
  ASSERT_EQ(values.size(), kExpectedImuStateInterfaceNames.size());
  EXPECT_NEAR(values.at(0), 0.0, 1.0e-3);
  EXPECT_NEAR(values.at(1), 0.0, 1.0e-3);
  EXPECT_NEAR(values.at(2), 0.0, 1.0e-3);
  EXPECT_NEAR(values.at(3), 1.0, 1.0e-3);
  EXPECT_NEAR(values.at(4), kDegreesToRadians, 1.0e-5);
  EXPECT_NEAR(values.at(5), 0.0, 1.0e-5);
  EXPECT_NEAR(values.at(6), 0.0, 1.0e-5);
  EXPECT_NEAR(values.at(7), 1.0, 1.0e-5);
  EXPECT_NEAR(values.at(8), 0.0, 1.0e-5);
  EXPECT_NEAR(values.at(9), 0.0, 1.0e-5);
}

TEST(EcImuHardwareTest, ActivateBeforeConfigureReturnsError) {
  EcImuHardware hardware;
  ASSERT_EQ(hardware.on_init(MakeHardwareInfo()), CallbackReturn::SUCCESS);

  EXPECT_EQ(hardware.on_activate(rclcpp_lifecycle::State{}), CallbackReturn::ERROR);
}

}  // namespace encos::ec_imu_hardware

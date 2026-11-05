#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "BaseJoint.h"
#include "adapter/base_adapter.h"
#include "adapter/fake_adapter_control.h"
#include "ec_joint_hardware/ec_joint_hardware.hpp"
#include "encos/driver_manager.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace encos::ec_joint_hardware {
namespace {

CallbackReturn InitializeHardware(EcJointHardware& hardware, const hardware_interface::HardwareInfo& info) {
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;
  return hardware.on_init(params);
}

constexpr char kJointName[] = "test_joint";

hardware_interface::InterfaceInfo MakeInterface(const std::string& name) {
  hardware_interface::InterfaceInfo interface;
  interface.name = name;
  return interface;
}

hardware_interface::ComponentInfo MakeJoint(const std::string& name) {
  hardware_interface::ComponentInfo joint;
  joint.name = name;
  for (const auto& interface : kCommandInterfaceNames) {
    joint.command_interfaces.push_back(MakeInterface(interface));
  }
  for (const auto& interface : kStateInterfaceNames) {
    joint.state_interfaces.push_back(MakeInterface(interface));
  }
  return joint;
}

hardware_interface::HardwareInfo MakeHardwareInfo(
    const std::vector<std::string>& joint_names = {kJointName},
    const std::string& config_file = "/tmp/ec_joint_hardware_unused_test_config.json") {
  hardware_interface::HardwareInfo info;
  info.name = "ec_joint_hardware_test";
  info.type = "system";
  info.hardware_plugin_name = "ec_joint_hardware/EcJointHardware";
  info.hardware_parameters["config_file"] = config_file;
  info.hardware_parameters["adapter_type_1"] = "Fake";
  info.hardware_parameters["interface_name_1"] = "test_adapter";
  for (const auto& joint_name : joint_names) {
    info.joints.push_back(MakeJoint(joint_name));
  }
  return info;
}

encos::JointStatus MakeStatus(float position = 1.0F, float velocity = 2.0F, float effort = 3.0F,
                              float mos_temperature = 4.0F, float motor_temperature = 5.0F,
                              encos::MotorError error = encos::MotorError::OverCurrent) {
  return {position, velocity, effort, mos_temperature, motor_temperature, error};
}

nlohmann::json RangeJson(double minimum, double maximum) { return {{"min", minimum}, {"max", maximum}}; }

nlohmann::json FakeRJointJson(const std::string& name, const std::string& adapter_name, int motor_id) {
  return {
      {"name", name},
      {"type", "RJoint"},
      {"config",
       {
           {"motorParams",
            {
                {"adapterId", adapter_name},
                {"busId", 0},
                {"motorId", motor_id},
                {"initParam",
                 {
                     {"ranges",
                      {
                          {"kp", RangeJson(0.0, 500.0)},
                          {"kd", RangeJson(0.0, 5.0)},
                          {"position", RangeJson(-12.5, 12.5)},
                          {"speed", RangeJson(-18.0, 18.0)},
                          {"torque", RangeJson(-30.0, 30.0)},
                          {"current", RangeJson(-30.0, 30.0)},
                          {"kt", 1.0},
                      }},
                 }},
            }},
           {"limit", RangeJson(-1.0, 1.0)},
           {"maxSpd", 2.0},
           {"maxTor", 3.0},
           {"reverse", false},
       }},
  };
}

class ScopedRuntimeConfig {
 public:
  ScopedRuntimeConfig(const std::string& name, const nlohmann::json& config)
      : path_(std::filesystem::temp_directory_path() / (name + ".json")) {
    std::ofstream output(path_);
    output << config.dump(2);
  }

  ~ScopedRuntimeConfig() { std::filesystem::remove(path_); }

  std::string path() const { return path_.string(); }

 private:
  std::filesystem::path path_;
};

class ScopedLogDirectory {
 public:
  explicit ScopedLogDirectory(std::string name)
      : path_(std::filesystem::temp_directory_path() / (std::move(name) + "_logs")) {
    std::filesystem::remove_all(path_);
  }

  ~ScopedLogDirectory() { std::filesystem::remove_all(path_); }

  std::string path() const { return path_.string(); }

 private:
  std::filesystem::path path_;
};

class ScopedAdapterCleanup {
 public:
  explicit ScopedAdapterCleanup(std::string adapter_name) : adapter_name_(std::move(adapter_name)) {}

  ~ScopedAdapterCleanup() { (void)encos::EncosDriverManager::Instance().DestroyAdapterByInterfaceName(adapter_name_); }

 private:
  std::string adapter_name_;
};

class FakeCommandRecorder {
 public:
  explicit FakeCommandRecorder(encos::FakeAdapterControl& control) : control_(control) {
    control_.SetDecodedCommandObserver([this](const encos::FakeCommandRecord& record) {
      std::lock_guard<std::mutex> lock(records_mutex_);
      records_.push_back(record);
    });
  }

  ~FakeCommandRecorder() { control_.ClearDecodedCommandObserver(); }

  void Clear() {
    std::lock_guard<std::mutex> lock(records_mutex_);
    records_.clear();
  }

  std::vector<encos::FakeCommandRecord> Snapshot() const {
    std::lock_guard<std::mutex> lock(records_mutex_);
    return records_;
  }

 private:
  encos::FakeAdapterControl& control_;
  mutable std::mutex records_mutex_;
  std::vector<encos::FakeCommandRecord> records_;
};

bool SeedFakeMotorFeedback(encos::BaseAdapter& adapter, int motor_id) {
  // Feedback seeding waits for a reply, including after configure enabled synchronization.
  adapter.SetSyncMode(false);
  auto* bus = adapter.GetBus(0);
  if (bus == nullptr) {
    return false;
  }
  auto* motor = bus->GetMotor(motor_id, encos::MotorModel::EC_A4310_P2);
  if (motor == nullptr) {
    return false;
  }
  return motor->PVTControl<1>(0.0F, 1.0F, 0.0F, 0.0F, 0.0F).error == encos::MotorError::NoError;
}

class MockJoint final : public encos::BaseJoint {
 public:
  MockJoint() : BaseJoint(encos::Range<float>{-10.0F, 10.0F}, 10.0F, 10.0F, false) {}

  encos::JointStatus& RequireStatus() {
    if (!status.has_value()) throw std::runtime_error("test fixture requires joint feedback");
    return *status;
  }

  void PVTControl(float kp, float kd, float position, float velocity, float effort) override {
    pvt_commands.push_back({kp, kd, position, velocity, effort, encos::PVTControlMode::MotorPVT});
  }

  void PVTControl(float kp, float kd, float position, float velocity, float effort,
                  encos::PVTControlMode torque_mode) override {
    pvt_commands.push_back({kp, kd, position, velocity, effort, torque_mode});
    if (events) events->push_back(label + ":pvt");
    if (throw_on_pvt) {
      throw std::runtime_error("injected PVT failure");
    }
  }

  void PosControl(float position, float velocity, float current, int feedback) override {
    position_commands.push_back({position, velocity, current, feedback});
    if (events) events->push_back(label + ":position");
    if (throw_on_position) throw std::runtime_error("injected position failure");
  }

  void SpdControl(float velocity, float current, int feedback) override {
    velocity_commands.push_back({velocity, current, feedback});
  }

  void CurControl(float current, int feedback) override { current_commands.push_back({current, feedback}); }

  void TorControl(float effort, int feedback) override { torque_commands.push_back({effort, feedback}); }

  void Stop(encos::MotorStopMode mode, float current, int feedback) override {
    stop_commands.push_back({mode, current, feedback});
  }

  float PosAsOffset(int) override { return 0.0F; }
  void SetOnStatus(std::function<void(const encos::JointStatus&)>) override {}
  bool HasBrake() const override {
    ++has_brake_calls;
    if (on_has_brake) on_has_brake();
    return has_brake;
  }
  bool Brake(bool enabled) override {
    brake_calls.push_back(enabled);
    if (events) events->push_back(label + (enabled ? ":engage" : ":release"));
    if (on_brake) on_brake(enabled);
    if (enabled ? throw_on_engage : throw_on_release) throw std::runtime_error("injected brake failure");
    return enabled ? engage_result : release_result;
  }
  void SetLimit(encos::Range<float>) override {}

  std::optional<encos::JointStatus> GetStatus(bool auto_send_command, int life_cycle_deduction) override {
    status_calls.push_back({auto_send_command, life_cycle_deduction});
    if (on_get_status) on_get_status();
    return status;
  }

  struct PvtCommand {
    float kp;
    float kd;
    float position;
    float velocity;
    float effort;
    encos::PVTControlMode torque_mode;
  };
  struct PositionCommand {
    float position;
    float velocity;
    float current;
    int feedback;
  };
  struct VelocityCommand {
    float velocity;
    float current;
    int feedback;
  };
  struct CurrentCommand {
    float current;
    int feedback;
  };
  struct TorqueCommand {
    float effort;
    int feedback;
  };
  struct StopCommand {
    encos::MotorStopMode mode;
    float current;
    int feedback;
  };
  struct StatusCall {
    bool auto_send_command;
    int life_cycle_deduction;
  };

  std::vector<PvtCommand> pvt_commands;
  std::vector<PositionCommand> position_commands;
  std::vector<VelocityCommand> velocity_commands;
  std::vector<CurrentCommand> current_commands;
  std::vector<TorqueCommand> torque_commands;
  std::vector<StopCommand> stop_commands;
  std::vector<StatusCall> status_calls;
  std::optional<encos::JointStatus> status;
  bool throw_on_pvt{false};
  bool throw_on_position{false};
  bool has_brake{false};
  mutable int has_brake_calls{0};
  bool release_result{true};
  bool engage_result{true};
  bool throw_on_release{false};
  bool throw_on_engage{false};
  std::function<void()> on_has_brake;
  std::function<void(bool)> on_brake;
  std::function<void()> on_get_status;
  std::vector<bool> brake_calls;
  std::vector<std::string>* events{nullptr};
  std::string label;
};

bool IsNan(double value) { return std::isnan(value); }

}  // namespace

struct EcJointHardwareTestAccess {
  static void SetConfigured(EcJointHardware& hardware, bool configured) { hardware.configured_ = configured; }
  static void SetActive(EcJointHardware& hardware, bool active) { hardware.active_ = active; }
  static int StatusMaxLifeCycle(const EcJointHardware& hardware) { return hardware.status_max_life_cycle_; }
  static bool IsLogged(const EcJointHardware& hardware) { return hardware.manager_.IsLogged(); }
  static bool IsConfigured(const EcJointHardware& hardware) { return hardware.configured_; }
  static bool IsActive(const EcJointHardware& hardware) { return hardware.active_; }
  static void CacheBrakes(EcJointHardware& hardware) {
    for (auto& joint : hardware.joints_)
      joint.brake_state = joint.ptr->HasBrake() ? BrakeState::Engaged : BrakeState::Unsupported;
    hardware.configured_ = true;
  }
  static bool HasBrake(const EcJointHardware& hardware, std::size_t index) {
    return hardware.joints_.at(index).brake_state != BrakeState::Unsupported;
  }
  static bool BrakeReleased(const EcJointHardware& hardware, std::size_t index = 0) {
    return hardware.joints_.at(index).brake_state == BrakeState::Released;
  }
  static BrakeState State(const EcJointHardware& hardware, std::size_t index = 0) {
    return hardware.joints_.at(index).brake_state;
  }
  static BrakeState EngageBrake(EcJointHardware& hardware, std::size_t index = 0) {
    return hardware.EngageBrake(hardware.joints_.at(index));
  }
  static BrakeState ReleaseBrake(EcJointHardware& hardware, std::size_t index = 0) {
    return hardware.ReleaseBrake(hardware.joints_.at(index));
  }
  static void SetBrakeState(EcJointHardware& hardware, std::size_t index, BrakeState state) {
    hardware.joints_.at(index).brake_state = state;
  }
  static bool HasJoint(const EcJointHardware& hardware, std::size_t index) { return !!hardware.joints_.at(index).ptr; }
  static ModeDefinition Mode(const EcJointHardware& hardware, std::size_t index) {
    return hardware.joints_.at(index).mode;
  }

  static void SetJoint(EcJointHardware& hardware, const std::shared_ptr<MockJoint>& joint) {
    hardware.joints_.front().ptr = joint;
  }

  static void SetJointForJoint(EcJointHardware& hardware, std::size_t joint_index,
                               const std::shared_ptr<MockJoint>& joint) {
    hardware.joints_.at(joint_index).ptr = joint;
  }

  static void SetCommand(EcJointHardware& hardware, double position = 0.1, double velocity = 0.2, double effort = 0.3,
                         double current = 0.4, double kp = 8.0, double kd = 0.7, double torque_mode = 1.0,
                         double stop_current = std::numeric_limits<double>::quiet_NaN()) {
    auto& runtime = hardware.joints_.front();
    runtime.command_position = position;
    runtime.command_velocity = velocity;
    runtime.command_effort = effort;
    runtime.command_current = current;
    runtime.command_kp = kp;
    runtime.command_kd = kd;
    runtime.command_torque_mode = torque_mode;
    runtime.command_stop_current = stop_current;
  }

  static void SetCommandForJoint(EcJointHardware& hardware, std::size_t joint_index, double position = 0.1,
                                 double velocity = 0.2, double effort = 0.3, double current = 0.4, double kp = 8.0,
                                 double kd = 0.7, double torque_mode = 1.0,
                                 double stop_current = std::numeric_limits<double>::quiet_NaN()) {
    auto& runtime = hardware.joints_.at(joint_index);
    runtime.command_position = position;
    runtime.command_velocity = velocity;
    runtime.command_effort = effort;
    runtime.command_current = current;
    runtime.command_kp = kp;
    runtime.command_kd = kd;
    runtime.command_torque_mode = torque_mode;
    runtime.command_stop_current = stop_current;
  }

  static std::vector<double> Commands(const EcJointHardware& hardware) {
    const auto& runtime = hardware.joints_.front();
    return {
        runtime.command_position, runtime.command_velocity, runtime.command_effort,      runtime.command_current,
        runtime.command_kp,       runtime.command_kd,       runtime.command_torque_mode, runtime.command_stop_current,
    };
  }
};

namespace {

void PrepareActiveHardware(EcJointHardware& hardware, const std::shared_ptr<MockJoint>& joint) {
  ASSERT_EQ(InitializeHardware(hardware, MakeHardwareInfo()), CallbackReturn::SUCCESS);
  EcJointHardwareTestAccess::SetJoint(hardware, joint);
  EcJointHardwareTestAccess::SetActive(hardware, true);
}

// 通过真实的命令模式切换 API
// 为关节绑定模式：先停掉全部接口（模拟停用旧控制器）， 再认领
// interfaces（模拟激活新控制器）
void BindMode(EcJointHardware& hardware, const std::vector<std::string>& joint_names,
              const std::vector<std::string>& interfaces) {
  std::vector<std::string> stop;
  std::vector<std::string> start;
  for (const auto& joint_name : joint_names) {
    for (const auto& interface : kCommandInterfaceNames) {
      stop.push_back(std::string(joint_name) + "/" + interface);
    }
    for (const auto& interface : interfaces) {
      start.push_back(joint_name + "/" + interface);
    }
  }
  EXPECT_EQ(hardware.prepare_command_mode_switch(start, stop), hardware_interface::return_type::OK);
  EXPECT_EQ(hardware.perform_command_mode_switch(start, stop), hardware_interface::return_type::OK);
}

void ExpectCommandsReset(const EcJointHardware& hardware) {
  for (const double value : EcJointHardwareTestAccess::Commands(hardware)) {
    EXPECT_TRUE(IsNan(value));
  }
}

hardware_interface::return_type Write(EcJointHardware& hardware) {
  return hardware.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01));
}

hardware_interface::return_type Read(EcJointHardware& hardware) {
  return hardware.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01));
}

std::vector<double> StateValues(EcJointHardware& hardware) {
  std::vector<double> values;
  for (const auto& interface : hardware.export_state_interfaces()) {
    values.push_back(interface.get_optional<double>().value());
  }
  return values;
}

bool HasLogFileWithSuffix(const std::filesystem::path& directory, const std::string& suffix) {
  if (!std::filesystem::is_directory(directory)) {
    return false;
  }
  for (const auto& entry : std::filesystem::directory_iterator(directory)) {
    const auto filename = entry.path().filename().string();
    if (entry.is_regular_file() && filename.size() >= suffix.size() &&
        filename.compare(filename.size() - suffix.size(), suffix.size(), suffix) == 0) {
      return true;
    }
  }
  return false;
}

}  // namespace

TEST(EcJointHardwareHardwareTest, UsesDefaultStatusMaxLifeCycleWhenParameterIsOmitted) {
  EcJointHardware hardware;

  ASSERT_EQ(InitializeHardware(hardware, MakeHardwareInfo()), CallbackReturn::SUCCESS);
  EXPECT_EQ(EcJointHardwareTestAccess::StatusMaxLifeCycle(hardware), 10);
}

TEST(EcJointHardwareHardwareTest, AcceptsEmptyAdapterSelection) {
  EcJointHardware hardware;
  auto info = MakeHardwareInfo();
  info.hardware_parameters.erase("adapter_type_1");
  info.hardware_parameters.erase("interface_name_1");

  EXPECT_EQ(InitializeHardware(hardware, info), CallbackReturn::SUCCESS);
}

TEST(EcJointHardwareHardwareTest, RejectsIncompleteOrEmptyAdapterSelection) {
  for (const auto& parameters : {
           std::pair<std::string, std::string>{"adapter_type_1", ""},
           std::pair<std::string, std::string>{"interface_name_1", ""},
       }) {
    EcJointHardware hardware;
    auto info = MakeHardwareInfo();
    info.hardware_parameters[parameters.first] = parameters.second;

    EXPECT_EQ(InitializeHardware(hardware, info), CallbackReturn::ERROR);
  }

  {
    EcJointHardware hardware;
    auto info = MakeHardwareInfo();
    info.hardware_parameters.erase("interface_name_1");
    EXPECT_EQ(InitializeHardware(hardware, info), CallbackReturn::ERROR);
  }
}

TEST(EcJointHardwareHardwareTest, AcceptsMultipleNumberedAdapterSelections) {
  EcJointHardware hardware;
  auto info = MakeHardwareInfo();
  info.hardware_parameters["adapter_type_2"] = "Ethercat";
  info.hardware_parameters["interface_name_2"] = "enp86s0";

  EXPECT_EQ(InitializeHardware(hardware, info), CallbackReturn::SUCCESS);
}

TEST(EcJointHardwareHardwareTest, AcceptsPositiveStatusMaxLifeCycleParameter) {
  EcJointHardware hardware;
  auto info = MakeHardwareInfo();
  info.hardware_parameters["status_max_life_cycle"] = "7";

  ASSERT_EQ(InitializeHardware(hardware, info), CallbackReturn::SUCCESS);
  EXPECT_EQ(EcJointHardwareTestAccess::StatusMaxLifeCycle(hardware), 7);
}

TEST(EcJointHardwareHardwareTest, UsesAtoiStyleStatusMaxLifeCycleParsing) {
  EcJointHardware hardware;
  auto info = MakeHardwareInfo();
  info.hardware_parameters["status_max_life_cycle"] = "3.5";

  ASSERT_EQ(InitializeHardware(hardware, info), CallbackReturn::SUCCESS);
  EXPECT_EQ(EcJointHardwareTestAccess::StatusMaxLifeCycle(hardware), 3);
}

TEST(EcJointHardwareHardwareTest, RejectsInvalidStatusMaxLifeCycleParameter) {
  for (const auto& value : {"", "0", "-1", "invalid"}) {
    EcJointHardware hardware;
    auto info = MakeHardwareInfo();
    info.hardware_parameters["status_max_life_cycle"] = value;

    EXPECT_EQ(InitializeHardware(hardware, info), CallbackReturn::ERROR) << "value: " << value;
  }
}

TEST(EcJointHardwareHardwareTest, IgnoresAnyLegacyCanfdValue) {
  for (const auto& value : {"true", "false", "", "yes", "invalid", "True", "0"}) {
    EcJointHardware hardware;
    auto info = MakeHardwareInfo();
    info.hardware_parameters["canfd_enable"] = value;
    EXPECT_EQ(InitializeHardware(hardware, info), CallbackReturn::SUCCESS);
  }
}

class EcJointHardwareCanfdTest : public testing::TestWithParam<std::string> {};

TEST_P(EcJointHardwareCanfdTest, ConfigureAndReconfigureApplyCanfdToTransmittedFrames) {
  const bool enabled = GetParam() == "true";
  const std::string adapter_name = "ec_joint_canfd_" + (GetParam().empty() ? "default" : GetParam());
  ScopedAdapterCleanup adapter_cleanup(adapter_name);
  auto* adapter = encos::MakeAdapter("Fake", adapter_name, adapter_name);
  ASSERT_NE(adapter, nullptr);
  auto* control = adapter->GetFakeAdapterControl();
  ASSERT_NE(control, nullptr);
  control->EnableAutoCreateMotor();
  control->SetReplyMode(encos::FakeReplyMode::Automatic);
  nlohmann::json config = {
      {"adapters", {{{"type", "Fake"}, {"name", adapter_name}}}},
      {"joints", {FakeRJointJson(kJointName, adapter_name, 1)}},
  };
  if (!GetParam().empty()) config["joints"][0]["config"]["motorParams"]["canfd"] = enabled;
  ScopedRuntimeConfig config_file(adapter_name, config);
  EcJointHardware hardware;
  auto info = MakeHardwareInfo({kJointName}, config_file.path());
  info.hardware_parameters["canfd_enable"] = enabled ? "false" : "true";
  ASSERT_EQ(InitializeHardware(hardware, info), CallbackReturn::SUCCESS);
  FakeCommandRecorder recorder(*control);

  for (int attempt = 0; attempt < 2; ++attempt) {
    SCOPED_TRACE(attempt);
    ASSERT_EQ(hardware.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
    ASSERT_TRUE(SeedFakeMotorFeedback(*adapter, 1));
    ASSERT_EQ(hardware.on_activate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
    adapter->SetSyncMode(true);
    recorder.Clear();

    BindMode(hardware, {kJointName}, {"position", "velocity", "effort", "kp", "kd", "torque_mode"});
    EcJointHardwareTestAccess::SetCommand(hardware);
    ASSERT_EQ(Write(hardware), hardware_interface::return_type::OK);
    const auto records = recorder.Snapshot();
    const auto command = std::find_if(records.begin(), records.end(), [](const auto& record) {
      return record.motor_idx == 1 && record.kind == encos::FakeCommandKind::PVTControl;
    });
    ASSERT_NE(command, records.end());
    EXPECT_TRUE(encos::CanFrameFlagsHaveValidCanFdBits(command->raw_frame_flags));
    EXPECT_EQ(encos::CanFrameFlagsUseCanFd(command->raw_frame_flags), enabled);
    ASSERT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);

    // Reconfiguration must overwrite the mode of a motor reused by the driver.
    auto* motor = adapter->GetBus(0)->GetMotors().at(1);
    if (enabled) {
      motor->DisableCanFd();
    } else {
      motor->EnableCanFd();
    }
  }
}

TEST_P(EcJointHardwareCanfdTest, SdkConfigurationFailureLeavesHardwareUnconfigured) {
  const std::string adapter_name = "ec_joint_canfd_failure_" + (GetParam().empty() ? "default" : GetParam());
  ScopedAdapterCleanup adapter_cleanup(adapter_name);
  auto joint_config = FakeRJointJson(kJointName, adapter_name, 1);
  joint_config["type"] = "UnsupportedCanfdTestJoint";
  ScopedRuntimeConfig config_file(adapter_name, {
                                                    {"adapters", {{{"type", "Fake"}, {"name", adapter_name}}}},
                                                    {"joints", {joint_config}},
                                                });
  EcJointHardware hardware;
  auto info = MakeHardwareInfo({kJointName}, config_file.path());
  if (!GetParam().empty()) {
    info.hardware_parameters["canfd_enable"] = GetParam();
  }
  ASSERT_EQ(InitializeHardware(hardware, info), CallbackReturn::SUCCESS);
  EXPECT_EQ(hardware.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::ERROR);
  EXPECT_FALSE(EcJointHardwareTestAccess::IsConfigured(hardware));
  EXPECT_EQ(hardware.on_activate(rclcpp_lifecycle::State{}), CallbackReturn::ERROR);
}

INSTANTIATE_TEST_SUITE_P(CanfdSettings, EcJointHardwareCanfdTest, testing::Values("", "true", "false"));

TEST(EcJointHardwareHardwareTest, DoesNotEnableLogsWhenLogPathIsOmitted) {
  const std::string adapter_name = "ec_joint_hardware_log_omitted";
  ScopedAdapterCleanup adapter_cleanup(adapter_name);
  auto* adapter = encos::MakeAdapter("Fake", adapter_name, adapter_name);
  ASSERT_NE(adapter, nullptr);
  ASSERT_NE(adapter->GetFakeAdapterControl(), nullptr);
  adapter->GetFakeAdapterControl()->EnableAutoCreateMotor();

  const nlohmann::json config = {
      {"adapters", {{{"type", "Fake"}, {"name", adapter_name}}}},
      {"joints", {FakeRJointJson(kJointName, adapter_name, 1)}},
  };
  ScopedRuntimeConfig config_file(adapter_name, config);
  EcJointHardware hardware;
  ASSERT_EQ(InitializeHardware(hardware, MakeHardwareInfo({kJointName}, config_file.path())), CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);

  EXPECT_FALSE(EcJointHardwareTestAccess::IsLogged(hardware));
}

TEST(EcJointHardwareHardwareTest, EnablesLogsWhenLogPathIsProvided) {
  const std::string adapter_name = "ec_joint_hardware_log_provided";
  ScopedAdapterCleanup adapter_cleanup(adapter_name);
  auto* adapter = encos::MakeAdapter("Fake", adapter_name, adapter_name);
  ASSERT_NE(adapter, nullptr);
  ASSERT_NE(adapter->GetFakeAdapterControl(), nullptr);
  adapter->GetFakeAdapterControl()->EnableAutoCreateMotor();

  const nlohmann::json config = {
      {"adapters", {{{"type", "Fake"}, {"name", adapter_name}}}},
      {"joints", {FakeRJointJson(kJointName, adapter_name, 1)}},
  };
  ScopedRuntimeConfig config_file(adapter_name, config);
  ScopedLogDirectory log_directory(adapter_name);
  EcJointHardware hardware;
  auto info = MakeHardwareInfo({kJointName}, config_file.path());
  info.hardware_parameters["log_path"] = log_directory.path();

  ASSERT_EQ(InitializeHardware(hardware, info), CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);

  EXPECT_TRUE(EcJointHardwareTestAccess::IsLogged(hardware));
  EXPECT_TRUE(HasLogFileWithSuffix(log_directory.path(), "_command.csv.zstd"));
  EXPECT_TRUE(HasLogFileWithSuffix(log_directory.path(), "_status.csv.zstd"));
}

TEST(EcJointHardwareHardwareTest, ExportsExactEightCommandAndSixStateInterfaces) {
  EcJointHardware hardware;
  ASSERT_EQ(InitializeHardware(hardware, MakeHardwareInfo()), CallbackReturn::SUCCESS);

  const auto command_interfaces = hardware.export_command_interfaces();
  const auto state_interfaces = hardware.export_state_interfaces();
  ASSERT_EQ(command_interfaces.size(), kCommandInterfaceNames.size());
  ASSERT_EQ(state_interfaces.size(), kStateInterfaceNames.size());

  for (std::size_t index = 0; index < kCommandInterfaceNames.size(); ++index) {
    EXPECT_EQ(command_interfaces[index].get_interface_name(), kCommandInterfaceNames[index]);
  }
  for (std::size_t index = 0; index < kStateInterfaceNames.size(); ++index) {
    EXPECT_EQ(state_interfaces[index].get_interface_name(), kStateInterfaceNames[index]);
    EXPECT_TRUE(std::isnan(state_interfaces[index].get_optional<double>().value()));
  }
}

TEST(EcJointHardwareHardwareTest, DispatchesEverySdkControlMode) {
  EcJointHardware hardware;
  const auto joint = std::make_shared<MockJoint>();
  PrepareActiveHardware(hardware, joint);

  BindMode(hardware, {kJointName}, {"position", "velocity", "effort", "kp", "kd", "torque_mode"});
  EcJointHardwareTestAccess::SetCommand(hardware);
  EXPECT_EQ(Write(hardware), hardware_interface::return_type::OK);
  ASSERT_EQ(joint->pvt_commands.size(), 1U);
  EXPECT_FLOAT_EQ(joint->pvt_commands.front().kp, 8.0F);
  EXPECT_EQ(joint->pvt_commands.front().torque_mode, encos::PVTControlMode::HostTorque);
  ExpectCommandsReset(hardware);

  BindMode(hardware, {kJointName}, {"position", "velocity", "current"});
  EcJointHardwareTestAccess::SetCommand(hardware);
  EXPECT_EQ(Write(hardware), hardware_interface::return_type::OK);
  ASSERT_EQ(joint->position_commands.size(), 1U);
  EXPECT_EQ(joint->position_commands.front().feedback, 1);

  BindMode(hardware, {kJointName}, {"velocity", "current"});
  EcJointHardwareTestAccess::SetCommand(hardware);
  EXPECT_EQ(Write(hardware), hardware_interface::return_type::OK);
  ASSERT_EQ(joint->velocity_commands.size(), 1U);
  EXPECT_EQ(joint->velocity_commands.front().feedback, 1);

  BindMode(hardware, {kJointName}, {"current"});
  EcJointHardwareTestAccess::SetCommand(hardware);
  EXPECT_EQ(Write(hardware), hardware_interface::return_type::OK);
  ASSERT_EQ(joint->current_commands.size(), 1U);
  EXPECT_EQ(joint->current_commands.front().feedback, 1);

  BindMode(hardware, {kJointName}, {"effort"});
  EcJointHardwareTestAccess::SetCommand(hardware);
  EXPECT_EQ(Write(hardware), hardware_interface::return_type::OK);
  ASSERT_EQ(joint->torque_commands.size(), 1U);
  EXPECT_EQ(joint->torque_commands.front().feedback, 1);

  BindMode(hardware, {kJointName}, {"stop_current"});
  EcJointHardwareTestAccess::SetCommand(hardware, 0.1, 0.2, 0.3, 0.4, 8.0, 0.7, 0.0, 1.5);
  EXPECT_EQ(Write(hardware), hardware_interface::return_type::OK);
  ASSERT_EQ(joint->stop_commands.size(), 1U);
  EXPECT_EQ(joint->stop_commands.front().mode, encos::MotorStopMode::FullBrake);
  EXPECT_FLOAT_EQ(joint->stop_commands.front().current, 1.5F);
  EXPECT_EQ(joint->stop_commands.front().feedback, 1);
  ExpectCommandsReset(hardware);
}

TEST(EcJointHardwareHardwareTest, PrepareRejectsIncompleteClaim) {
  EcJointHardware hardware;
  const auto joint = std::make_shared<MockJoint>();
  PrepareActiveHardware(hardware, joint);

  const std::vector<std::string> start = {"test_joint/position", "test_joint/velocity"};
  EXPECT_EQ(hardware.prepare_command_mode_switch(start, {}), hardware_interface::return_type::ERROR);
}

TEST(EcJointHardwareHardwareTest, PrepareRejectsMixedModeClaim) {
  EcJointHardware hardware;
  const auto joint = std::make_shared<MockJoint>();
  PrepareActiveHardware(hardware, joint);

  const std::vector<std::string> start = {"test_joint/kp", "test_joint/current"};
  EXPECT_EQ(hardware.prepare_command_mode_switch(start, {}), hardware_interface::return_type::ERROR);
}

TEST(EcJointHardwareHardwareTest, PrepareRejectsConflictingClaimsAcrossControllers) {
  EcJointHardware hardware;
  const auto joint = std::make_shared<MockJoint>();
  PrepareActiveHardware(hardware, joint);
  BindMode(hardware, {kJointName}, {"effort"});

  // 第二个控制器再认领同一关节的 current：与既有 effort 拼成跨模式混合
  const std::vector<std::string> start = {"test_joint/current"};
  EXPECT_EQ(hardware.prepare_command_mode_switch(start, {}), hardware_interface::return_type::ERROR);
}

TEST(EcJointHardwareHardwareTest, PrepareDoesNotApplyModeUntilPerform) {
  EcJointHardware hardware;
  const auto joint = std::make_shared<MockJoint>();
  PrepareActiveHardware(hardware, joint);
  BindMode(hardware, {kJointName}, {"effort"});

  const std::vector<std::string> start = {"test_joint/current"};
  const std::vector<std::string> stop = {"test_joint/effort"};
  ASSERT_EQ(hardware.prepare_command_mode_switch(start, stop), hardware_interface::return_type::OK);

  EcJointHardwareTestAccess::SetCommand(hardware);
  EXPECT_EQ(Write(hardware), hardware_interface::return_type::OK);
  EXPECT_EQ(joint->torque_commands.size(), 1U);
  EXPECT_TRUE(joint->current_commands.empty());

  ASSERT_EQ(hardware.perform_command_mode_switch(start, stop), hardware_interface::return_type::OK);
  EcJointHardwareTestAccess::SetCommand(hardware);
  EXPECT_EQ(Write(hardware), hardware_interface::return_type::OK);
  EXPECT_EQ(joint->torque_commands.size(), 1U);
  EXPECT_EQ(joint->current_commands.size(), 1U);
}

TEST(EcJointHardwareHardwareTest, PrepareAcceptsUnclaimedJoint) {
  EcJointHardware hardware;
  const auto joint = std::make_shared<MockJoint>();
  PrepareActiveHardware(hardware, joint);

  EXPECT_EQ(hardware.prepare_command_mode_switch({}, {}), hardware_interface::return_type::OK);
  EXPECT_EQ(hardware.perform_command_mode_switch({}, {}), hardware_interface::return_type::OK);
}

TEST(EcJointHardwareHardwareTest, ModeClearedWhenClaimReleased) {
  EcJointHardware hardware;
  const auto joint = std::make_shared<MockJoint>();
  PrepareActiveHardware(hardware, joint);
  BindMode(hardware, {kJointName}, {"position", "velocity", "current"});

  // 控制器停用并释放全部认领后，关节回到未绑定状态，写入的命令不再派发
  const std::vector<std::string> stop = {"test_joint/position", "test_joint/velocity", "test_joint/current"};
  EXPECT_EQ(hardware.prepare_command_mode_switch({}, stop), hardware_interface::return_type::OK);
  EXPECT_EQ(hardware.perform_command_mode_switch({}, stop), hardware_interface::return_type::OK);
  EcJointHardwareTestAccess::SetCommand(hardware);
  EXPECT_EQ(Write(hardware), hardware_interface::return_type::OK);
  EXPECT_TRUE(joint->position_commands.empty());
}

TEST(EcJointHardwareHardwareTest, UnwrittenRequiredInterfacesSkipJointAndEveryWriteResetsCommands) {
  EcJointHardware hardware;
  const auto joint = std::make_shared<MockJoint>();
  PrepareActiveHardware(hardware, joint);
  BindMode(hardware, {kJointName}, {"position", "velocity", "effort", "kp", "kd", "torque_mode"});

  // 无人写命令时，沿用各模式校验跳过该关节，命令槽每周期复位。
  EXPECT_EQ(Write(hardware), hardware_interface::return_type::OK);
  EXPECT_TRUE(joint->pvt_commands.empty());
  EXPECT_TRUE(joint->position_commands.empty());
  EXPECT_TRUE(joint->stop_commands.empty());
  ExpectCommandsReset(hardware);
}

TEST(EcJointHardwareHardwareTest, ValidCommandDispatchesAndMissingCommandSkipsJoint) {
  EcJointHardware hardware;
  const auto joint = std::make_shared<MockJoint>();
  PrepareActiveHardware(hardware, joint);
  BindMode(hardware, {kJointName}, {"position", "velocity", "effort", "kp", "kd", "torque_mode"});

  // 无人写命令时不派发。
  ASSERT_EQ(Write(hardware), hardware_interface::return_type::OK);
  ASSERT_TRUE(joint->pvt_commands.empty());

  // 有效命令在同一周期按模式派发。
  EcJointHardwareTestAccess::SetCommand(hardware);
  ASSERT_EQ(Write(hardware), hardware_interface::return_type::OK);
  ASSERT_EQ(joint->pvt_commands.size(), 1U);

  // 后续漏写命令仍跳过该关节，write 返回 OK。
  EXPECT_EQ(Write(hardware), hardware_interface::return_type::OK);
  EXPECT_EQ(joint->pvt_commands.size(), 1U);
  ExpectCommandsReset(hardware);
}

TEST(EcJointHardwareHardwareTest, ValidJointDoesNotWaitForOtherClaimedJoints) {
  constexpr char kFirstJoint[] = "test_joint_one";
  constexpr char kSecondJoint[] = "test_joint_two";
  EcJointHardware hardware;
  const auto first_joint = std::make_shared<MockJoint>();
  const auto second_joint = std::make_shared<MockJoint>();
  ASSERT_EQ(InitializeHardware(hardware, MakeHardwareInfo({kFirstJoint, kSecondJoint})), CallbackReturn::SUCCESS);
  EcJointHardwareTestAccess::SetJointForJoint(hardware, 0, first_joint);
  EcJointHardwareTestAccess::SetJointForJoint(hardware, 1, second_joint);
  EcJointHardwareTestAccess::SetActive(hardware, true);
  BindMode(hardware, {kFirstJoint, kSecondJoint}, {"position", "velocity", "current"});

  // 两个关节都没写命令时，各自校验失败，不派发。
  ASSERT_EQ(Write(hardware), hardware_interface::return_type::OK);
  EXPECT_TRUE(first_joint->position_commands.empty());
  EXPECT_TRUE(second_joint->position_commands.empty());

  // 一个关节有命令即可派发，不等待其他关节。
  EcJointHardwareTestAccess::SetCommandForJoint(hardware, 0);
  EXPECT_EQ(Write(hardware), hardware_interface::return_type::OK);
  EXPECT_EQ(first_joint->position_commands.size(), 1U);
  EXPECT_TRUE(second_joint->position_commands.empty());

  // 两个关节都写命令时，分别派发。
  EcJointHardwareTestAccess::SetCommandForJoint(hardware, 0);
  EcJointHardwareTestAccess::SetCommandForJoint(hardware, 1);
  EXPECT_EQ(Write(hardware), hardware_interface::return_type::OK);
  EXPECT_EQ(first_joint->position_commands.size(), 2U);
  EXPECT_EQ(second_joint->position_commands.size(), 1U);
}

TEST(EcJointHardwareHardwareTest, ReleasingEveryClaimResumesZeroCurrentCommand) {
  EcJointHardware hardware;
  const auto joint = std::make_shared<MockJoint>();
  PrepareActiveHardware(hardware, joint);
  BindMode(hardware, {kJointName}, {"position", "velocity", "current"});
  EcJointHardwareTestAccess::SetCommand(hardware);
  ASSERT_EQ(Write(hardware), hardware_interface::return_type::OK);
  ASSERT_EQ(joint->position_commands.size(), 1U);

  // 释放全部认领后，未绑定模式下发零电流。
  const std::vector<std::string> stop = {"test_joint/position", "test_joint/velocity", "test_joint/current"};
  ASSERT_EQ(hardware.prepare_command_mode_switch({}, stop), hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.perform_command_mode_switch({}, stop), hardware_interface::return_type::OK);
  EXPECT_EQ(Write(hardware), hardware_interface::return_type::OK);
  EXPECT_EQ(joint->position_commands.size(), 1U);
  EXPECT_TRUE(joint->pvt_commands.empty());
  ASSERT_EQ(joint->current_commands.size(), 1U);
  EXPECT_FLOAT_EQ(joint->current_commands.front().current, 0.0F);
  EXPECT_EQ(joint->current_commands.front().feedback, 1);
}

TEST(EcJointHardwareHardwareTest, UnboundJointSendsZeroCurrentFeedbackCommand) {
  EcJointHardware hardware;
  const auto joint = std::make_shared<MockJoint>();
  PrepareActiveHardware(hardware, joint);

  // 未绑定 controller 时发送零电流，维持底层反馈请求链路。
  EXPECT_EQ(Write(hardware), hardware_interface::return_type::OK);
  EXPECT_TRUE(joint->pvt_commands.empty());
  ASSERT_EQ(joint->current_commands.size(), 1U);
  EXPECT_FLOAT_EQ(joint->current_commands.front().current, 0.0F);
  EXPECT_EQ(joint->current_commands.front().feedback, 1);
  EXPECT_TRUE(joint->position_commands.empty());
  ExpectCommandsReset(hardware);
}

TEST(EcJointHardwareHardwareTest, InvalidCommandSkipsJointWithoutBlockingBatch) {
  EcJointHardware hardware;
  const auto joint = std::make_shared<MockJoint>();
  PrepareActiveHardware(hardware, joint);
  BindMode(hardware, {kJointName}, {"position", "velocity", "effort", "kp", "kd", "torque_mode"});

  // 必填 position 为 NaN：非法命令跳过并记录日志；其余关节正常派发，write 返回
  // OK（逐关节独立语义）
  EcJointHardwareTestAccess::SetCommand(hardware, std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(Write(hardware), hardware_interface::return_type::OK);
  EXPECT_TRUE(joint->pvt_commands.empty());
  EXPECT_TRUE(joint->position_commands.empty());
  EXPECT_TRUE(joint->stop_commands.empty());
  ExpectCommandsReset(hardware);
}

TEST(EcJointHardwareHardwareTest, RejectsInvalidDiscreteInterfaces) {
  EcJointHardware hardware;
  const auto joint = std::make_shared<MockJoint>();
  PrepareActiveHardware(hardware, joint);

  // 非法枚举值：记录日志并跳过该关节，write 仍返回 OK（逐关节独立语义）。
  BindMode(hardware, {kJointName}, {"position", "velocity", "effort", "kp", "kd", "torque_mode"});
  for (double mode :
       {-1.0, 3.0, 0.5, 1.5, 2.5, std::numeric_limits<double>::max(), std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
    SCOPED_TRACE(mode);
    EcJointHardwareTestAccess::SetCommand(hardware, 0.1, 0.2, 0.3, 0.4, 8.0, 0.7, mode);
    EXPECT_EQ(Write(hardware), hardware_interface::return_type::OK);
    EXPECT_TRUE(joint->pvt_commands.empty());
    ExpectCommandsReset(hardware);
  }

  BindMode(hardware, {kJointName}, {"velocity", "current"});
  // 只写 velocity 不写 current：部分缺失，跳过该关节
  EcJointHardwareTestAccess::SetCommand(hardware, 0.1, 0.2, 0.3, std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(Write(hardware), hardware_interface::return_type::OK);
  EXPECT_TRUE(joint->velocity_commands.empty());
}

TEST(EcJointHardwareHardwareTest, ForwardsAllPvtControlModesWithoutBooleanConversion) {
  EcJointHardware hardware;
  const auto joint = std::make_shared<MockJoint>();
  PrepareActiveHardware(hardware, joint);
  BindMode(hardware, {kJointName}, {"position", "velocity", "effort", "kp", "kd", "torque_mode"});

  for (auto mode :
       {encos::PVTControlMode::MotorPVT, encos::PVTControlMode::HostTorque, encos::PVTControlMode::HostKpMotorKd}) {
    SCOPED_TRACE(static_cast<int>(mode));
    joint->pvt_commands.clear();
    EcJointHardwareTestAccess::SetCommand(hardware, 0.1, 0.2, 0.3, 0.4, 8.0, 0.7, static_cast<double>(mode));
    EXPECT_EQ(Write(hardware), hardware_interface::return_type::OK);
    ASSERT_EQ(joint->pvt_commands.size(), 1U);
    const auto& command = joint->pvt_commands.front();
    EXPECT_EQ(command.torque_mode, mode);
    EXPECT_FLOAT_EQ(command.kp, 8.0F);
    EXPECT_FLOAT_EQ(command.kd, 0.7F);
    EXPECT_FLOAT_EQ(command.position, 0.1F);
    EXPECT_FLOAT_EQ(command.velocity, 0.2F);
    EXPECT_FLOAT_EQ(command.effort, 0.3F);
    ExpectCommandsReset(hardware);
  }
}

TEST(EcJointHardwareHardwareTest, RejectsInfiniteRequiredValues) {
  EcJointHardware hardware;
  const auto joint = std::make_shared<MockJoint>();
  PrepareActiveHardware(hardware, joint);

  BindMode(hardware, {kJointName}, {"current"});
  EcJointHardwareTestAccess::SetCommand(hardware, 0.1, 0.2, 0.3, std::numeric_limits<double>::infinity());
  EXPECT_EQ(Write(hardware), hardware_interface::return_type::OK);
  EXPECT_TRUE(joint->current_commands.empty());

  BindMode(hardware, {kJointName}, {"effort"});
  EcJointHardwareTestAccess::SetCommand(hardware, 0.1, 0.2, std::numeric_limits<double>::infinity());
  EXPECT_EQ(Write(hardware), hardware_interface::return_type::OK);
  EXPECT_TRUE(joint->torque_commands.empty());
}

TEST(EcJointHardwareHardwareTest, ReadReturnsAllSixSdkStatusInterfacesDirectly) {
  EcJointHardware hardware;
  const auto joint = std::make_shared<MockJoint>();
  joint->status = MakeStatus();
  PrepareActiveHardware(hardware, joint);

  EXPECT_EQ(Read(hardware), hardware_interface::return_type::OK);
  EXPECT_EQ(StateValues(hardware),
            (std::vector<double>{1.0, 2.0, 3.0, 4.0, 5.0, static_cast<double>(encos::MotorError::OverCurrent)}));
  ASSERT_EQ(joint->status_calls.size(), 1U);
  EXPECT_FALSE(joint->status_calls.front().auto_send_command);
  EXPECT_EQ(joint->status_calls.front().life_cycle_deduction, 1);
}

TEST(EcJointHardwareHardwareTest, ReadReturnsOkOnMissingFeedback) {
  EcJointHardware hardware;
  const auto joint = std::make_shared<MockJoint>();
  // 不设置 joint->status，模拟看门狗过期/从未收到反馈时 SDK 返回 nullopt
  PrepareActiveHardware(hardware, joint);

  EXPECT_EQ(Read(hardware), hardware_interface::return_type::OK);
  ASSERT_EQ(joint->status_calls.size(), 1U);
  EXPECT_EQ(joint->status_calls.front().life_cycle_deduction, 1);
}

TEST(EcJointHardwareHardwareTest, MissingFeedbackDoesNotPreventOtherJointsFromUpdating) {
  EcJointHardware hardware;
  ASSERT_EQ(InitializeHardware(hardware, MakeHardwareInfo({"offline", "healthy"})), CallbackReturn::SUCCESS);
  auto offline = std::make_shared<MockJoint>();
  auto healthy = std::make_shared<MockJoint>();
  healthy->status = MakeStatus(7.0F);
  EcJointHardwareTestAccess::SetJointForJoint(hardware, 0, offline);
  EcJointHardwareTestAccess::SetJointForJoint(hardware, 1, healthy);
  EcJointHardwareTestAccess::SetActive(hardware, true);

  ASSERT_EQ(Read(hardware), hardware_interface::return_type::OK);
  EXPECT_EQ(healthy->status_calls.size(), 1U);
  EXPECT_DOUBLE_EQ(StateValues(hardware).at(6), 7.0);
  healthy->status = MakeStatus(8.0F);
  ASSERT_EQ(Read(hardware), hardware_interface::return_type::OK);
  EXPECT_EQ(healthy->status_calls.size(), 2U);
  EXPECT_DOUBLE_EQ(StateValues(hardware).at(6), 8.0);
}

TEST(EcJointHardwareHardwareTest, ActivationRequiresAndPublishesFullFiniteFeedback) {
  EcJointHardware hardware;
  const auto joint = std::make_shared<MockJoint>();
  joint->status = MakeStatus();
  ASSERT_EQ(InitializeHardware(hardware, MakeHardwareInfo()), CallbackReturn::SUCCESS);
  EcJointHardwareTestAccess::SetJoint(hardware, joint);
  EcJointHardwareTestAccess::SetConfigured(hardware, true);
  joint->on_get_status = [&] {
    ASSERT_EQ(joint->current_commands.size(), 1U);
    EXPECT_FLOAT_EQ(joint->current_commands.front().current, 0.0F);
    EXPECT_EQ(joint->current_commands.front().feedback, 1);
  };

  EXPECT_EQ(hardware.on_activate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  EXPECT_EQ(StateValues(hardware),
            (std::vector<double>{1.0, 2.0, 3.0, 4.0, 5.0, static_cast<double>(encos::MotorError::OverCurrent)}));
  ASSERT_EQ(joint->status_calls.size(), 1U);
  EXPECT_FALSE(joint->status_calls.front().auto_send_command);
  EXPECT_EQ(joint->status_calls.front().life_cycle_deduction, 1);
}

TEST(EcJointHardwareHardwareTest, DeactivateDoesNotIssueStopOrCommitCommands) {
  EcJointHardware hardware;
  const auto joint = std::make_shared<MockJoint>();
  PrepareActiveHardware(hardware, joint);

  EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  EXPECT_TRUE(joint->stop_commands.empty());
  EXPECT_EQ(Write(hardware), hardware_interface::return_type::OK);
  EXPECT_TRUE(joint->pvt_commands.empty());
}

TEST(EcJointHardwareHardwareTest, SdkControlExceptionReturnsErrorWithoutAutomaticStop) {
  EcJointHardware hardware;
  const auto joint = std::make_shared<MockJoint>();
  joint->throw_on_pvt = true;
  PrepareActiveHardware(hardware, joint);
  BindMode(hardware, {kJointName}, {"position", "velocity", "effort", "kp", "kd", "torque_mode"});
  EcJointHardwareTestAccess::SetCommand(hardware);

  EXPECT_EQ(Write(hardware), hardware_interface::return_type::ERROR);
  ASSERT_EQ(joint->pvt_commands.size(), 1U);
  EXPECT_TRUE(joint->stop_commands.empty());
  ExpectCommandsReset(hardware);
}

TEST(EcJointHardwareHardwareTest, ResolvesAdapterMarkersBeforeSdkConfiguration) {
  const std::string adapter_name = "ec_joint_hardware_numbered_adapter_test";
  ScopedAdapterCleanup adapter_cleanup(adapter_name);
  auto* adapter = encos::MakeAdapter("Fake", adapter_name, adapter_name);
  ASSERT_NE(adapter, nullptr);
  auto* control = adapter->GetFakeAdapterControl();
  ASSERT_NE(control, nullptr);
  control->EnableAutoCreateMotor();

  const nlohmann::json config = {
      {"adapters", {{{"type", "plugin-replace-me-1"}, {"name", "enx-replace-me-1"}}}},
      {"joints", {FakeRJointJson(kJointName, "enx-replace-me-1", 1)}},
  };
  ScopedRuntimeConfig config_file(adapter_name + "_config", config);
  EcJointHardware hardware;
  auto info = MakeHardwareInfo({kJointName}, config_file.path());
  info.hardware_parameters["adapter_type_1"] = "Fake";
  info.hardware_parameters["interface_name_1"] = adapter_name;

  ASSERT_EQ(InitializeHardware(hardware, info), CallbackReturn::SUCCESS);
  EXPECT_EQ(hardware.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
}

TEST(EcJointHardwareHardwareTest, WriteCommitsTheSelectedJointBatchOnItsOwnBus) {
  constexpr char kFirstJoint[] = "test_joint_one";
  constexpr char kSecondJoint[] = "test_joint_two";
  const std::string adapter_name = "ec_joint_hardware_commit_test";
  ScopedAdapterCleanup adapter_cleanup(adapter_name);
  auto* adapter = encos::MakeAdapter("Fake", adapter_name, adapter_name);
  ASSERT_NE(adapter, nullptr);
  auto* control = adapter->GetFakeAdapterControl();
  ASSERT_NE(control, nullptr);
  control->EnableAutoCreateMotor();
  control->SetReplyMode(encos::FakeReplyMode::Automatic);

  const nlohmann::json config = {
      {"adapters", {{{"type", "Fake"}, {"name", adapter_name}}}},
      {"joints", {FakeRJointJson(kFirstJoint, adapter_name, 1), FakeRJointJson(kSecondJoint, adapter_name, 2)}},
  };
  ScopedRuntimeConfig config_file(adapter_name, config);
  EcJointHardware hardware;
  ASSERT_EQ(InitializeHardware(hardware, MakeHardwareInfo({kFirstJoint, kSecondJoint}, config_file.path())),
            CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  ASSERT_TRUE(SeedFakeMotorFeedback(*adapter, 1));
  ASSERT_TRUE(SeedFakeMotorFeedback(*adapter, 2));
  FakeCommandRecorder recorder(*control);

  ASSERT_EQ(hardware.on_activate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  adapter->SetSyncMode(true);
  recorder.Clear();

  BindMode(hardware, {kFirstJoint, kSecondJoint}, {"position", "velocity", "effort", "kp", "kd", "torque_mode"});
  EcJointHardwareTestAccess::SetCommandForJoint(hardware, 0, 0.5, 0.25, 0.125, 0.0, 8.0, 2.0, 0.0);
  EcJointHardwareTestAccess::SetCommandForJoint(hardware, 1, -0.25, -0.125, -0.0625, 0.0, 7.0, 1.5, 0.0);

  EXPECT_EQ(Write(hardware), hardware_interface::return_type::OK);

  const auto records = recorder.Snapshot();
  const auto first_joint_command = std::find_if(records.begin(), records.end(), [](const auto& record) {
    return record.motor_idx == 1 && record.kind == encos::FakeCommandKind::PVTControl;
  });
  const auto second_joint_command = std::find_if(records.begin(), records.end(), [](const auto& record) {
    return record.motor_idx == 2 && record.kind == encos::FakeCommandKind::PVTControl;
  });
  EXPECT_NE(first_joint_command, records.end());
  EXPECT_NE(second_joint_command, records.end());
  EXPECT_EQ(std::count_if(records.begin(), records.end(),
                          [](const auto& record) { return record.kind == encos::FakeCommandKind::PVTControl; }),
            2);
}

class EcJointHardwareBrakeTest : public testing::Test {
 protected:
  void Initialize(std::size_t count = 2) {
    std::vector<std::string> names;
    for (std::size_t index = 0; index < count; ++index) names.push_back("joint" + std::to_string(index));
    ASSERT_EQ(InitializeHardware(hardware, MakeHardwareInfo(names)), CallbackReturn::SUCCESS);
    for (std::size_t index = 0; index < count; ++index) {
      auto joint = std::make_shared<MockJoint>();
      joint->has_brake = true;
      joint->status = MakeStatus(static_cast<float>(index + 1), 0, 0, 20, 20, encos::MotorError::NoError);
      joint->events = &events;
      joint->label = names[index];
      mocks.push_back(joint);
      EcJointHardwareTestAccess::SetJointForJoint(hardware, index, joint);
    }
    for (auto& mock : mocks) {
      mock->on_has_brake = [this, count] {
        for (std::size_t index = 0; index < count; ++index)
          EXPECT_TRUE(EcJointHardwareTestAccess::HasJoint(hardware, index));
      };
    }
    EcJointHardwareTestAccess::CacheBrakes(hardware);
  }

  void Activate() {
    ASSERT_EQ(hardware.on_activate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
    // Brake tests below count runtime commands separately from startup feedback requests.
    for (auto& mock : mocks) mock->current_commands.clear();
  }
  void ClaimPvt() {
    BindMode(hardware, {"joint0", "joint1"}, {"position", "velocity", "effort", "kp", "kd", "torque_mode"});
  }
  void Commands() {
    for (std::size_t index = 0; index < mocks.size(); ++index)
      EcJointHardwareTestAccess::SetCommandForJoint(hardware, index);
  }

  EcJointHardware hardware;
  std::vector<std::shared_ptr<MockJoint>> mocks;
  std::vector<std::string> events;
};

TEST_F(EcJointHardwareBrakeTest, ActivationDoesNotReleaseOrSendHoldCommands) {
  Initialize();
  Activate();
  EXPECT_EQ(Read(hardware), hardware_interface::return_type::OK);
  EXPECT_EQ(Write(hardware), hardware_interface::return_type::OK);
  EXPECT_TRUE(events.empty());
  for (const auto& mock : mocks) {
    EXPECT_EQ(mock->has_brake_calls, 1);
    EXPECT_TRUE(mock->brake_calls.empty());
    EXPECT_TRUE(mock->position_commands.empty());
    EXPECT_TRUE(mock->pvt_commands.empty());
  }
}

TEST_F(EcJointHardwareBrakeTest, FirstValidCommandHoldsCurrentPositionsBeforeReleasingAndControlling) {
  Initialize();
  Activate();
  ClaimPvt();
  EXPECT_TRUE(events.empty());
  EXPECT_EQ(Write(hardware), hardware_interface::return_type::OK);
  EXPECT_TRUE(events.empty());
  mocks[0]->RequireStatus().pos = 0.25F;  // Use current SDK feedback, not activation state or controller target.
  Commands();
  EXPECT_EQ(Write(hardware), hardware_interface::return_type::OK);
  EXPECT_EQ(events, (std::vector<std::string>{"joint0:position", "joint0:release", "joint0:pvt", "joint1:position",
                                              "joint1:release", "joint1:pvt"}));
  Commands();
  EXPECT_EQ(Write(hardware), hardware_interface::return_type::OK);
  for (const auto& mock : mocks) {
    EXPECT_EQ(mock->brake_calls, (std::vector<bool>{false}));
    EXPECT_EQ(mock->pvt_commands.size(), 2U);
    ASSERT_EQ(mock->position_commands.size(), 1U);
    EXPECT_FLOAT_EQ(mock->position_commands.front().position, mock->RequireStatus().pos);
    EXPECT_FLOAT_EQ(mock->position_commands.front().velocity, 0.0F);
    EXPECT_FLOAT_EQ(mock->position_commands.front().current, 0.0F);
    EXPECT_EQ(mock->position_commands.front().feedback, 1);
    EXPECT_EQ(mock->has_brake_calls, 1);
  }
}

TEST_F(EcJointHardwareBrakeTest, SwitchPreservesCommandsWithoutReengagingOrReleasing) {
  Initialize();
  Activate();
  ClaimPvt();
  Commands();
  ASSERT_EQ(Write(hardware), hardware_interface::return_type::OK);
  Commands();  // Switching preserves unconsumed commands.
  ClaimPvt();
  ASSERT_EQ(Write(hardware), hardware_interface::return_type::OK);
  for (const auto& mock : mocks) {
    EXPECT_EQ(mock->brake_calls, (std::vector<bool>{false}));
    EXPECT_EQ(mock->pvt_commands.size(), 2U);
  }
  Commands();
  ASSERT_EQ(Write(hardware), hardware_interface::return_type::OK);
  for (const auto& mock : mocks) {
    EXPECT_EQ(mock->brake_calls, (std::vector<bool>{false}));
    EXPECT_EQ(mock->position_commands.size(), 1U);
    EXPECT_EQ(mock->pvt_commands.size(), 3U);
  }
}

TEST_F(EcJointHardwareBrakeTest, MissingOrInvalidCommandOnlyPreventsThatJointRelease) {
  Initialize();
  Activate();
  ClaimPvt();
  EcJointHardwareTestAccess::SetCommandForJoint(hardware, 1);
  ASSERT_EQ(Write(hardware), hardware_interface::return_type::OK);
  EXPECT_TRUE(mocks[0]->brake_calls.empty());
  EXPECT_EQ(mocks[1]->brake_calls, (std::vector<bool>{false}));
  Commands();
  EcJointHardwareTestAccess::SetCommandForJoint(hardware, 0, std::numeric_limits<double>::quiet_NaN());
  ASSERT_EQ(Write(hardware), hardware_interface::return_type::OK);
  EXPECT_TRUE(mocks[0]->brake_calls.empty());
  EXPECT_TRUE(mocks[0]->pvt_commands.empty());
  EXPECT_EQ(mocks[1]->brake_calls, (std::vector<bool>{false}));
  Commands();
  ASSERT_EQ(Write(hardware), hardware_interface::return_type::OK);
  EXPECT_EQ(mocks[0]->brake_calls, (std::vector<bool>{false}));
}

TEST_F(EcJointHardwareBrakeTest, ControllerExitEngagesOnceWithoutSendingControl) {
  Initialize();
  Activate();
  ClaimPvt();
  Commands();
  ASSERT_EQ(Write(hardware), hardware_interface::return_type::OK);
  mocks[0]->status.reset();  // None mode needs no position feedback.
  BindMode(hardware, {"joint0", "joint1"}, {});
  for (const auto& mock : mocks) EXPECT_EQ(mock->brake_calls, (std::vector<bool>{false}));
  for (int cycle = 0; cycle < 3; ++cycle) ASSERT_EQ(Write(hardware), hardware_interface::return_type::OK);
  for (const auto& mock : mocks) {
    EXPECT_EQ(mock->brake_calls, (std::vector<bool>{false, true}));
    EXPECT_EQ(mock->position_commands.size(), 1U);  // Only the hold before the earlier release.
    EXPECT_EQ(mock->pvt_commands.size(), 1U);
    EXPECT_TRUE(mock->current_commands.empty());
  }
}

TEST_F(EcJointHardwareBrakeTest, NoneBrakeFailureReturnsErrorAndRetriesWithoutControl) {
  Initialize(1);
  Activate();
  BindMode(hardware, {"joint0"}, {"current"});
  Commands();
  ASSERT_EQ(Write(hardware), hardware_interface::return_type::OK);
  BindMode(hardware, {"joint0"}, {});
  mocks[0]->engage_result = false;
  EXPECT_EQ(Write(hardware), hardware_interface::return_type::ERROR);
  EXPECT_EQ(EcJointHardwareTestAccess::State(hardware), BrakeState::Released);
  mocks[0]->engage_result = true;
  EXPECT_EQ(Write(hardware), hardware_interface::return_type::OK);
  EXPECT_EQ(EcJointHardwareTestAccess::State(hardware), BrakeState::Engaged);
  EXPECT_EQ(mocks[0]->brake_calls, (std::vector<bool>{false, true, true}));
  EXPECT_EQ(mocks[0]->current_commands.size(), 1U);
  EXPECT_EQ(mocks[0]->position_commands.size(), 1U);
}

TEST_F(EcJointHardwareBrakeTest, PrepareOnlyValidatesAndJointModesRemainIndependent) {
  Initialize();
  ASSERT_FALSE(EcJointHardwareTestAccess::IsActive(hardware));
  EXPECT_EQ(hardware.prepare_command_mode_switch({"joint0/current"}, {}), hardware_interface::return_type::OK);
  EXPECT_TRUE(events.empty());
  EXPECT_EQ(hardware.perform_command_mode_switch({"joint0/current"}, {}), hardware_interface::return_type::OK);
  EXPECT_TRUE(events.empty());
  EXPECT_EQ(EcJointHardwareTestAccess::Mode(hardware, 0), ModeDefinition::Current);
  EXPECT_EQ(EcJointHardwareTestAccess::Mode(hardware, 1), ModeDefinition::None);
  Activate();
  EXPECT_EQ(hardware.prepare_command_mode_switch({"joint1/effort"}, {}), hardware_interface::return_type::OK);
  EXPECT_EQ(hardware.perform_command_mode_switch({"joint1/effort"}, {}), hardware_interface::return_type::OK);
  EXPECT_EQ(EcJointHardwareTestAccess::Mode(hardware, 1), ModeDefinition::Torque);
}

TEST_F(EcJointHardwareBrakeTest, SuccessfulBrakeCycleUpdatesStateAfterConfirmationAndSkipsRepeatedRequests) {
  Initialize();
  Activate();
  ClaimPvt();
  for (std::size_t index = 0; index < mocks.size(); ++index) {
    EXPECT_EQ(EcJointHardwareTestAccess::State(hardware, index), BrakeState::Engaged);
    mocks[index]->on_brake = [this, index](bool enabled) {
      EXPECT_EQ(EcJointHardwareTestAccess::State(hardware, index),
                enabled ? BrakeState::Engaging : BrakeState::Releasing);
    };
  }
  for (int cycle = 0; cycle < 2; ++cycle) {
    Commands();
    ASSERT_EQ(Write(hardware), hardware_interface::return_type::OK);
    for (std::size_t index = 0; index < mocks.size(); ++index)
      EXPECT_EQ(EcJointHardwareTestAccess::State(hardware, index), BrakeState::Released);
    Commands();
    ASSERT_EQ(Write(hardware), hardware_interface::return_type::OK);
    ASSERT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
    ASSERT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
    for (std::size_t index = 0; index < mocks.size(); ++index) {
      EXPECT_EQ(EcJointHardwareTestAccess::State(hardware, index), BrakeState::Engaged);
      EXPECT_EQ(mocks[index]->brake_calls.size(), static_cast<std::size_t>((cycle + 1) * 2));
    }
    Activate();
  }
}

TEST_F(EcJointHardwareBrakeTest, WriteSilentlySkipsPendingJointsAndControlsOtherJoints) {
  Initialize();
  Activate();
  ClaimPvt();
  Commands();
  ASSERT_EQ(Write(hardware), hardware_interface::return_type::OK);
  const auto first_brakes = mocks[0]->brake_calls.size();
  const auto first_controls = mocks[0]->pvt_commands.size();
  const auto first_status_reads = mocks[0]->status_calls.size();
  testing::internal::CaptureStderr();
  for (auto state : {BrakeState::Engaging, BrakeState::Releasing}) {
    EcJointHardwareTestAccess::SetBrakeState(hardware, 0, state);
    for (int cycle = 0; cycle < 3; ++cycle) {
      Commands();
      EXPECT_EQ(Write(hardware), hardware_interface::return_type::OK);
    }
    EXPECT_EQ(EcJointHardwareTestAccess::State(hardware), state);
  }
  const auto output = testing::internal::GetCapturedStderr();
  EXPECT_TRUE(output.empty()) << output;
  EXPECT_EQ(mocks[0]->brake_calls.size(), first_brakes);
  EXPECT_EQ(mocks[0]->pvt_commands.size(), first_controls);
  EXPECT_EQ(mocks[0]->status_calls.size(), first_status_reads);
  EXPECT_EQ(mocks[1]->pvt_commands.size(), 7U);
}

TEST_F(EcJointHardwareBrakeTest, PendingBrakeRequestsDoNotRepeatSdkActions) {
  Initialize(1);
  mocks[0]->on_get_status = [this] {
    EXPECT_EQ(EcJointHardwareTestAccess::State(hardware), BrakeState::Releasing);
    EXPECT_EQ(EcJointHardwareTestAccess::ReleaseBrake(hardware), BrakeState::Releasing);
  };
  mocks[0]->on_brake = [this](bool enabled) {
    EXPECT_EQ(EcJointHardwareTestAccess::State(hardware), enabled ? BrakeState::Engaging : BrakeState::Releasing);
    const auto count = mocks[0]->brake_calls.size();
    EXPECT_EQ(EcJointHardwareTestAccess::ReleaseBrake(hardware),
              enabled ? BrakeState::Engaging : BrakeState::Releasing);
    EXPECT_EQ(EcJointHardwareTestAccess::EngageBrake(hardware), enabled ? BrakeState::Engaging : BrakeState::Releasing);
    EXPECT_EQ(mocks[0]->brake_calls.size(), count);
  };
  EXPECT_EQ(EcJointHardwareTestAccess::ReleaseBrake(hardware), BrakeState::Released);
  EXPECT_EQ(EcJointHardwareTestAccess::State(hardware), BrakeState::Released);
  EXPECT_EQ(EcJointHardwareTestAccess::ReleaseBrake(hardware), BrakeState::Released);
  EXPECT_EQ(mocks[0]->brake_calls, (std::vector<bool>{false}));
  EXPECT_EQ(EcJointHardwareTestAccess::EngageBrake(hardware), BrakeState::Engaged);
  EXPECT_EQ(EcJointHardwareTestAccess::State(hardware), BrakeState::Engaged);
  EXPECT_EQ(EcJointHardwareTestAccess::EngageBrake(hardware), BrakeState::Engaged);
  EXPECT_EQ(mocks[0]->brake_calls, (std::vector<bool>{false, true}));
}

TEST_F(EcJointHardwareBrakeTest, ReleaseFalseRollsBackAndExceptionKeepsPendingState) {
  Initialize(1);
  Activate();
  BindMode(hardware, {"joint0"}, {"current"});
  for (bool throws : {false, true}) {
    EcJointHardwareTestAccess::CacheBrakes(hardware);
    mocks[0]->release_result = false;
    mocks[0]->throw_on_release = throws;
    mocks[0]->on_brake = [this](bool) { EXPECT_EQ(EcJointHardwareTestAccess::State(hardware), BrakeState::Releasing); };
    const auto expected_state = throws ? BrakeState::Releasing : BrakeState::Engaged;
    Commands();
    EXPECT_EQ(Write(hardware), hardware_interface::return_type::ERROR);
    EXPECT_EQ(EcJointHardwareTestAccess::State(hardware), expected_state);
    EXPECT_TRUE(mocks[0]->current_commands.empty());
    const auto count = mocks[0]->brake_calls.size();
    Commands();
    EXPECT_EQ(Write(hardware), throws ? hardware_interface::return_type::OK : hardware_interface::return_type::ERROR);
    EXPECT_TRUE(mocks[0]->current_commands.empty());
    EXPECT_EQ(hardware.on_error(rclcpp_lifecycle::State{}), throws ? CallbackReturn::ERROR : CallbackReturn::SUCCESS);
    EXPECT_EQ(mocks[0]->brake_calls.size(), count + (throws ? 0U : 1U));
    EXPECT_EQ(EcJointHardwareTestAccess::State(hardware), expected_state);
    Activate();
    if (!throws) {
      mocks[0]->release_result = true;
      Commands();
      EXPECT_EQ(Write(hardware), hardware_interface::return_type::OK);
      EXPECT_EQ(EcJointHardwareTestAccess::State(hardware), BrakeState::Released);
      EXPECT_EQ(mocks[0]->current_commands.size(), 1U);
      mocks[0]->current_commands.clear();
    }
  }
}

TEST_F(EcJointHardwareBrakeTest, SwitchDoesNotCallBrakesEvenWhenEngagementWouldFail) {
  Initialize(3);
  Activate();
  mocks[0]->engage_result = false;
  mocks[1]->throw_on_engage = true;
  EXPECT_EQ(hardware.perform_command_mode_switch({"joint0/current"}, {}), hardware_interface::return_type::OK);
  for (const auto& mock : mocks) EXPECT_TRUE(mock->brake_calls.empty());
  EXPECT_EQ(EcJointHardwareTestAccess::Mode(hardware, 0), ModeDefinition::Current);
  EXPECT_EQ(EcJointHardwareTestAccess::Mode(hardware, 1), ModeDefinition::None);
}

TEST_F(EcJointHardwareBrakeTest, AllExitCallbacksAttemptAllJointsDespiteFailures) {
  Initialize(3);
  BindMode(hardware, {"joint0", "joint1", "joint2"}, {"current"});
  const auto exits = {&EcJointHardware::on_deactivate, &EcJointHardware::on_error, &EcJointHardware::on_shutdown};
  for (auto exit : exits) {
    EcJointHardwareTestAccess::CacheBrakes(hardware);
    Activate();
    Commands();
    ASSERT_EQ(Write(hardware), hardware_interface::return_type::OK);
    for (std::size_t index = 0; index < mocks.size(); ++index) {
      mocks[index]->on_brake = [this, index](bool enabled) {
        EXPECT_EQ(EcJointHardwareTestAccess::State(hardware, index),
                  enabled ? BrakeState::Engaging : BrakeState::Releasing);
      };
    }
    mocks[0]->engage_result = false;
    mocks[1]->throw_on_engage = true;
    EXPECT_EQ((hardware.*exit)(rclcpp_lifecycle::State{}), CallbackReturn::ERROR);
    EXPECT_FALSE(EcJointHardwareTestAccess::IsActive(hardware));
    for (const auto& mock : mocks) EXPECT_TRUE(mock->brake_calls.back());
    EXPECT_EQ(EcJointHardwareTestAccess::State(hardware, 0), BrakeState::Released);
    EXPECT_EQ(EcJointHardwareTestAccess::State(hardware, 1), BrakeState::Engaging);
    EXPECT_EQ(EcJointHardwareTestAccess::State(hardware, 2), BrakeState::Engaged);
    EXPECT_EQ(EcJointHardwareTestAccess::EngageBrake(hardware), BrakeState::Released);
    const auto count = mocks[0]->brake_calls.size();
    mocks[0]->engage_result = true;
    mocks[1]->throw_on_engage = false;
    EXPECT_EQ((hardware.*exit)(rclcpp_lifecycle::State{}), CallbackReturn::ERROR);
    EXPECT_EQ(mocks[0]->brake_calls.size(), count + 1);
    EXPECT_EQ(EcJointHardwareTestAccess::State(hardware, 0), BrakeState::Engaged);
    EXPECT_EQ(EcJointHardwareTestAccess::State(hardware, 1), BrakeState::Engaging);
  }
}

TEST_F(EcJointHardwareBrakeTest, ConfigureFailureClearsPriorBindingsAndBrakeState) {
  Initialize();
  Activate();
  ClaimPvt();
  Commands();
  ASSERT_EQ(Write(hardware), hardware_interface::return_type::OK);
  ASSERT_TRUE(EcJointHardwareTestAccess::BrakeReleased(hardware, 0));
  EXPECT_EQ(hardware.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::ERROR);
  EXPECT_FALSE(EcJointHardwareTestAccess::HasJoint(hardware, 0));
  EXPECT_FALSE(EcJointHardwareTestAccess::HasBrake(hardware, 0));
  EXPECT_FALSE(EcJointHardwareTestAccess::BrakeReleased(hardware, 0));
  const auto count = mocks[0]->brake_calls.size();
  EXPECT_EQ(hardware.on_error(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  EXPECT_EQ(mocks[0]->brake_calls.size(), count);
}

TEST_F(EcJointHardwareBrakeTest, ControlFailureUsesLifecycleBrakingWithoutAdditionalHold) {
  Initialize();
  Activate();
  ClaimPvt();
  mocks[0]->throw_on_pvt = true;
  Commands();
  EXPECT_EQ(Write(hardware), hardware_interface::return_type::ERROR);
  EXPECT_TRUE(EcJointHardwareTestAccess::BrakeReleased(hardware, 0));
  EXPECT_EQ(hardware.on_error(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  EXPECT_FALSE(EcJointHardwareTestAccess::BrakeReleased(hardware, 0));
  EXPECT_EQ(mocks[0]->brake_calls, (std::vector<bool>{false, true}));
  EXPECT_EQ(mocks[0]->position_commands.size(), 1U);
  EXPECT_TRUE(mocks[1]->position_commands.empty());
}

TEST_F(EcJointHardwareBrakeTest, InvalidHoldFeedbackPreventsReleaseAndControllerCommands) {
  Initialize();
  Activate();
  ClaimPvt();
  for (bool missing : {true, false}) {
    mocks[0]->status = missing ? std::nullopt
                               : std::optional{MakeStatus(std::numeric_limits<float>::quiet_NaN(), 0, 0, 20, 20,
                                                          encos::MotorError::NoError)};
    Commands();
    EXPECT_EQ(Write(hardware), hardware_interface::return_type::ERROR);
    EXPECT_EQ(EcJointHardwareTestAccess::State(hardware), BrakeState::Engaged);
    EXPECT_EQ(EcJointHardwareTestAccess::ReleaseBrake(hardware), BrakeState::Engaged);
    for (const auto& mock : mocks) {
      EXPECT_TRUE(mock->brake_calls.empty());
      EXPECT_TRUE(mock->position_commands.empty());
      EXPECT_TRUE(mock->pvt_commands.empty());
    }
  }
  mocks[0]->status = MakeStatus(0.25F, 0, 0, 20, 20, encos::MotorError::NoError);
  Commands();
  EXPECT_EQ(Write(hardware), hardware_interface::return_type::OK);
  EXPECT_EQ(EcJointHardwareTestAccess::State(hardware), BrakeState::Released);
  EXPECT_EQ(mocks[0]->brake_calls, (std::vector<bool>{false}));
  EXPECT_EQ(mocks[0]->pvt_commands.size(), 1U);
}

TEST_F(EcJointHardwareBrakeTest, HoldFailurePreventsThatJointReleaseAndControllerCommand) {
  Initialize();
  Activate();
  ClaimPvt();
  mocks[1]->throw_on_position = true;
  Commands();
  EXPECT_EQ(Write(hardware), hardware_interface::return_type::ERROR);
  EXPECT_EQ(mocks[0]->brake_calls, (std::vector<bool>{false}));
  EXPECT_EQ(mocks[0]->pvt_commands.size(), 1U);
  EXPECT_TRUE(mocks[1]->brake_calls.empty());
  EXPECT_TRUE(mocks[1]->pvt_commands.empty());
}

TEST(EcJointHardwareHardwareTest, MixedBrakeCapabilitiesPreserveNonBrakeBehavior) {
  EcJointHardware hardware;
  ASSERT_EQ(InitializeHardware(hardware, MakeHardwareInfo({"braked", "plain"})), CallbackReturn::SUCCESS);
  auto braked = std::make_shared<MockJoint>(), plain = std::make_shared<MockJoint>();
  braked->has_brake = true;
  braked->status = plain->status = MakeStatus(0, 0, 0, 20, 20, encos::MotorError::NoError);
  EcJointHardwareTestAccess::SetJointForJoint(hardware, 0, braked);
  EcJointHardwareTestAccess::SetJointForJoint(hardware, 1, plain);
  EcJointHardwareTestAccess::CacheBrakes(hardware);
  EXPECT_EQ(EcJointHardwareTestAccess::EngageBrake(hardware, 1), BrakeState::Unsupported);
  EXPECT_EQ(EcJointHardwareTestAccess::ReleaseBrake(hardware, 1), BrakeState::Unsupported);
  ASSERT_EQ(hardware.on_activate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  BindMode(hardware, {"braked", "plain"}, {"current"});
  EcJointHardwareTestAccess::SetCommandForJoint(hardware, 0);
  EcJointHardwareTestAccess::SetCommandForJoint(hardware, 1);
  ASSERT_EQ(Write(hardware), hardware_interface::return_type::OK);
  EXPECT_EQ(braked->brake_calls, (std::vector<bool>{false}));
  EXPECT_TRUE(plain->brake_calls.empty());
  EXPECT_EQ(braked->position_commands.size(), 1U);
  EXPECT_TRUE(plain->position_commands.empty());
  BindMode(hardware, {"braked", "plain"}, {});
  ASSERT_EQ(Write(hardware), hardware_interface::return_type::OK);
  EXPECT_TRUE(braked->pvt_commands.empty());
  EXPECT_TRUE(plain->pvt_commands.empty());
  ASSERT_EQ(braked->current_commands.size(), 2U);  // Startup request plus runtime current command.
  ASSERT_EQ(plain->current_commands.size(), 3U);   // Also sends zero current after releasing interfaces.
  EXPECT_EQ(braked->brake_calls, (std::vector<bool>{false, true}));
  EXPECT_EQ(EcJointHardwareTestAccess::State(hardware, 0), BrakeState::Engaged);
  EXPECT_FLOAT_EQ(plain->current_commands.back().current, 0.0F);
  EXPECT_EQ(plain->current_commands.back().feedback, 1);
  EXPECT_EQ(EcJointHardwareTestAccess::State(hardware, 1), BrakeState::Unsupported);
  EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  EXPECT_EQ(EcJointHardwareTestAccess::State(hardware, 1), BrakeState::Unsupported);
  EXPECT_TRUE(plain->brake_calls.empty());
}

TEST(EcJointHardwareHardwareTest, AdapterJsonValidationReachesSdk) {
  for (const auto& adapter_json : std::vector<nlohmann::json>{
           {{"name", "invalid"}},
           {{"type", "Fake"}, {"name", 1}},
           {{"type", "Fake"}, {"name", "invalid"}, {"extend", 1}},
       }) {
    ScopedRuntimeConfig file("ec_joint_invalid_adapter",
                             {{"adapters", {adapter_json}}, {"joints", {FakeRJointJson(kJointName, "invalid", 1)}}});
    EcJointHardware hardware;
    ASSERT_EQ(InitializeHardware(hardware, MakeHardwareInfo({kJointName}, file.path())), CallbackReturn::SUCCESS);
    EXPECT_EQ(hardware.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::ERROR);
    EXPECT_FALSE(EcJointHardwareTestAccess::HasJoint(hardware, 0));
  }
}

TEST(EcJointHardwareHardwareTest, NonemptyExtendReachesLegacyFakePluginWarning) {
  const std::string adapter_name = "ec_joint_extend_legacy";
  ScopedAdapterCleanup cleanup(adapter_name);
  nlohmann::json adapter_json = {{"type", "Fake"}, {"name", adapter_name}};
  adapter_json["extend"]["bitrate"] = 1000000;
  adapter_json["extend"]["buses"]["0:0"]["bitrate"] = 500000;
  ScopedRuntimeConfig file(adapter_name,
                           {{"adapters", {adapter_json}}, {"joints", {FakeRJointJson(kJointName, adapter_name, 1)}}});
  EcJointHardware hardware;
  ASSERT_EQ(InitializeHardware(hardware, MakeHardwareInfo({kJointName}, file.path())), CallbackReturn::SUCCESS);
  // The installed driver warns and ignores extend for legacy MakeAdapter plugins.
  // Observe that warning: dropping the JSON inside hardware would hide it.
  testing::internal::CaptureStdout();
  const auto result = hardware.on_configure(rclcpp_lifecycle::State{});
  const auto output = testing::internal::GetCapturedStdout();
  EXPECT_EQ(result, CallbackReturn::SUCCESS);
  EXPECT_NE(output.find("provided extend configuration was ignored"), std::string::npos);
}

TEST(EcJointHardwareHardwareTest, FakeMixedCanfdUsesNumericAndStringAdapterIdsInOrder) {
  const std::string first = "ec_joint_mixed_first", second = "ec_joint_mixed_second";
  ScopedAdapterCleanup cleanup_first(first), cleanup_second(second);
  auto* adapter1 = encos::MakeAdapter("Fake", first, first);
  auto* adapter2 = encos::MakeAdapter("Fake", second, second);
  ASSERT_NE(adapter1, nullptr);
  ASSERT_NE(adapter2, nullptr);
  adapter1->GetFakeAdapterControl()->EnableAutoCreateMotor();
  adapter2->GetFakeAdapterControl()->EnableAutoCreateMotor();
  auto a = FakeRJointJson("a", first, 1);
  auto b = FakeRJointJson("b", second, 2);
  auto c = FakeRJointJson("c", second, 3);
  a["config"]["motorParams"]["adapterId"] = 0;
  a["config"]["motorParams"]["canfd"] = true;
  b["config"]["motorParams"]["adapterId"] = 1;
  b["config"]["motorParams"]["canfd"] = false;
  c["config"]["motorParams"]["slaveId"] = 0;
  ScopedRuntimeConfig file("ec_joint_mixed",
                           {{"adapters", {{{"type", "Fake"}, {"name", first}}, {{"type", "Fake"}, {"name", second}}}},
                            {"joints", {a, b, c}}});
  EcJointHardware hardware;
  auto info = MakeHardwareInfo({"a", "b", "c"}, file.path());
  info.hardware_parameters.erase("adapter_type_1");
  info.hardware_parameters.erase("interface_name_1");
  info.hardware_parameters["canfd_enable"] = "ignored";
  ASSERT_EQ(InitializeHardware(hardware, info), CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  EcJointHardwareTestAccess::SetActive(hardware, true);
  FakeCommandRecorder recorder1(*adapter1->GetFakeAdapterControl()), recorder2(*adapter2->GetFakeAdapterControl());
  BindMode(hardware, {"a", "b", "c"}, {"current"});
  for (std::size_t index = 0; index < 3; ++index) EcJointHardwareTestAccess::SetCommandForJoint(hardware, index);
  ASSERT_EQ(Write(hardware), hardware_interface::return_type::OK);
  auto records1 = recorder1.Snapshot(), records2 = recorder2.Snapshot();
  const auto non_current = [](const auto& record) { return record.kind != encos::FakeCommandKind::CurControl; };
  records1.erase(std::remove_if(records1.begin(), records1.end(), non_current), records1.end());
  records2.erase(std::remove_if(records2.begin(), records2.end(), non_current), records2.end());
  ASSERT_EQ(records1.size(), 1U);
  ASSERT_EQ(records2.size(), 2U);
  EXPECT_TRUE(encos::CanFrameFlagsUseCanFd(records1[0].raw_frame_flags));
  for (const auto& record : records2) EXPECT_FALSE(encos::CanFrameFlagsUseCanFd(record.raw_frame_flags));
}

TEST(EcJointHardwareHardwareTest, FakeReconfigureRefreshesCachedBrakeCapability) {
  const std::string name = "ec_joint_brake_reconfigure";
  ScopedAdapterCleanup cleanup(name);
  auto* adapter = encos::MakeAdapter("Fake", name, name);
  ASSERT_NE(adapter, nullptr);
  adapter->GetFakeAdapterControl()->EnableAutoCreateMotor();
  auto properties = encos::GetMotorModelRanges(encos::MotorModel::EC_A4310_P2);
  properties.brake = true;
  ASSERT_NE(adapter->GetBus(0)->GetMotor(1, properties), nullptr);
  nlohmann::json config = {{"adapters", {{{"type", "Fake"}, {"name", name}}}},
                           {"joints", {FakeRJointJson(kJointName, name, 1)}}};
  ScopedRuntimeConfig file(name, config);
  EcJointHardware hardware;
  ASSERT_EQ(InitializeHardware(hardware, MakeHardwareInfo({kJointName}, file.path())), CallbackReturn::SUCCESS);
  FakeCommandRecorder recorder(*adapter->GetFakeAdapterControl());
  ASSERT_EQ(hardware.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  EXPECT_TRUE(EcJointHardwareTestAccess::HasBrake(hardware, 0));
  auto records = recorder.Snapshot();
  ASSERT_EQ(records.size(), 1U);
  ASSERT_EQ(records.front().kind, encos::FakeCommandKind::Brake);
  const auto* brake = std::get_if<encos::FakeBrakePayload>(&records.front().payload);
  ASSERT_NE(brake, nullptr);
  EXPECT_TRUE(brake->enabled);
  EXPECT_FALSE(EcJointHardwareTestAccess::BrakeReleased(hardware));
  EXPECT_EQ(EcJointHardwareTestAccess::State(hardware), BrakeState::Engaged);
  recorder.Clear();
  config["joints"][0]["config"]["motorParams"]["motorId"] = 2;
  {
    std::ofstream output(file.path());
    output << config;
  }
  ASSERT_EQ(hardware.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  EXPECT_FALSE(EcJointHardwareTestAccess::HasBrake(hardware, 0));
  EXPECT_EQ(EcJointHardwareTestAccess::State(hardware), BrakeState::Unsupported);
  records = recorder.Snapshot();
  EXPECT_TRUE(std::none_of(records.begin(), records.end(),
                           [](const auto& record) { return record.kind == encos::FakeCommandKind::Brake; }));
}

TEST(EcJointHardwareHardwareTest, ConfigureBrakeFailureClearsBindingsAndPreventsActivation) {
  for (bool throws : {false, true}) {
    const std::string name = throws ? "ec_joint_configure_brake_throw" : "ec_joint_configure_brake_false";
    ScopedAdapterCleanup cleanup(name);
    auto* adapter = encos::MakeAdapter("Fake", name, name);
    ASSERT_NE(adapter, nullptr);
    auto* control = adapter->GetFakeAdapterControl();
    control->EnableAutoCreateMotor();
    auto properties = encos::GetMotorModelRanges(encos::MotorModel::EC_A4310_P2);
    properties.brake = true;
    ASSERT_NE(adapter->GetBus(0)->GetMotor(1, properties), nullptr);
    ScopedRuntimeConfig file(
        name, {{"adapters", {{{"type", "Fake"}, {"name", name}}}}, {"joints", {FakeRJointJson(kJointName, name, 1)}}});
    EcJointHardware hardware;
    ASSERT_EQ(InitializeHardware(hardware, MakeHardwareInfo({kJointName}, file.path())), CallbackReturn::SUCCESS);
    if (throws) {
      control->SetDecodedCommandObserver([](const auto& record) {
        if (record.kind == encos::FakeCommandKind::Brake) throw std::runtime_error("injected configure brake failure");
      });
    } else {
      control->SetReplyMode(encos::FakeReplyMode::Manual);  // Missing acknowledgement makes Brake return false.
    }
    EXPECT_EQ(hardware.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::ERROR);
    control->ClearDecodedCommandObserver();
    EXPECT_FALSE(EcJointHardwareTestAccess::IsConfigured(hardware));
    EXPECT_FALSE(EcJointHardwareTestAccess::HasJoint(hardware, 0));
    EXPECT_FALSE(EcJointHardwareTestAccess::HasBrake(hardware, 0));
    EXPECT_FALSE(EcJointHardwareTestAccess::BrakeReleased(hardware));
    EXPECT_EQ(hardware.on_activate(rclcpp_lifecycle::State{}), CallbackReturn::ERROR);
  }
}

TEST(EcJointHardwareHardwareTest, RealSdkCoupledPairCachesBrakesAndPreservesPerMotorCanfd) {
  const std::string name = "ec_joint_coupled_json";
  ScopedAdapterCleanup cleanup(name);
  auto* adapter = encos::MakeAdapter("Fake", name, name);
  ASSERT_NE(adapter, nullptr);
  auto* control = adapter->GetFakeAdapterControl();
  control->EnableAutoCreateMotor();
  control->EnablePositionError(false);
  auto properties = encos::GetMotorModelRanges(encos::MotorModel::EC_A4310_P2);
  properties.brake = true;
  ASSERT_NE(adapter->GetBus(0)->GetMotor(1, properties), nullptr);
  ASSERT_NE(adapter->GetBus(0)->GetMotor(2, properties), nullptr);
  std::ifstream input(std::string(TEST_FIXTURE_DIR) + "/control_v2.json");
  nlohmann::json robot;
  input >> robot;
  std::vector<nlohmann::json> pair;
  for (const auto& joint : robot["joints"]) {
    if (joint["type"] == "DoubleCoupleJoint") pair.push_back(joint);
    if (pair.size() == 2) break;
  }
  ASSERT_EQ(pair.size(), 2U);
  for (std::size_t index = 0; index < 2; ++index) {
    pair[index]["name"] = "joint" + std::to_string(index);
    auto& parameters = pair[index]["config"];
    parameters["maxSpd"] = 0.5;
    parameters["maxTor"] = 3.0;
    parameters["limit"] = RangeJson(-1.0, 1.0);
    parameters["motorParams1"] = FakeRJointJson("", name, 1)["config"]["motorParams"];
    parameters["motorParams2"] = FakeRJointJson("", name, 2)["config"]["motorParams"];
    parameters["motorParams1"]["adapterId"] = 0;
    parameters["motorParams2"]["adapterId"] = 0;
    parameters["motorParams1"]["canfd"] = true;
    parameters["motorParams2"]["canfd"] = false;
  }
  // Slave-first ordering catches capability queries made before SDK pairing completes.
  std::reverse(pair.begin(), pair.end());
  ScopedRuntimeConfig file(name, {{"adapters", {{{"type", "Fake"}, {"name", name}}}}, {"joints", pair}});
  EcJointHardware hardware;
  ASSERT_EQ(InitializeHardware(hardware, MakeHardwareInfo({"joint0", "joint1"}, file.path())), CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  EXPECT_TRUE(EcJointHardwareTestAccess::HasBrake(hardware, 0));
  EXPECT_TRUE(EcJointHardwareTestAccess::HasBrake(hardware, 1));
  ASSERT_TRUE(SeedFakeMotorFeedback(*adapter, 1));
  ASSERT_TRUE(SeedFakeMotorFeedback(*adapter, 2));
  FakeCommandRecorder recorder(*control);
  ASSERT_EQ(hardware.on_activate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  // Activation requests feedback with zero current while keeping the brakes engaged.
  const auto activation_commands = recorder.Snapshot();
  ASSERT_FALSE(activation_commands.empty());
  for (const auto& record : activation_commands) {
    ASSERT_EQ(record.kind, encos::FakeCommandKind::CurControl);
    const auto* payload = std::get_if<encos::FakeCurControlPayload>(&record.payload);
    ASSERT_NE(payload, nullptr);
    EXPECT_FLOAT_EQ(payload->current, 0.0F);
    EXPECT_EQ(payload->feedback_type, 1);
    EXPECT_EQ(encos::CanFrameFlagsUseCanFd(record.raw_frame_flags), record.motor_idx == 1);
  }
  EXPECT_EQ(EcJointHardwareTestAccess::State(hardware, 0), BrakeState::Engaged);
  EXPECT_EQ(EcJointHardwareTestAccess::State(hardware, 1), BrakeState::Engaged);
  BindMode(hardware, {"joint0", "joint1"}, {"position", "velocity", "current"});
  recorder.Clear();
  ASSERT_EQ(Write(hardware), hardware_interface::return_type::OK);
  EXPECT_TRUE(recorder.Snapshot().empty());
  for (std::size_t index = 0; index < 2; ++index) EcJointHardwareTestAccess::SetCommandForJoint(hardware, index);
  ASSERT_EQ(Write(hardware), hardware_interface::return_type::OK);
  std::size_t holds = 0, commands = 0, releases = 0;
  for (const auto& record : recorder.Snapshot()) {
    if (record.kind == encos::FakeCommandKind::PosControl) {
      if (releases == 0)
        ++holds;
      else
        ++commands;
      EXPECT_EQ(encos::CanFrameFlagsUseCanFd(record.raw_frame_flags), record.motor_idx == 1);
    }
    if (record.kind == encos::FakeCommandKind::Brake) {
      EXPECT_EQ(holds, 2U);
      ++releases;
    }
  }
  EXPECT_EQ(holds, 2U);
  EXPECT_EQ(commands, 2U);
  EXPECT_GT(releases, 0U);  // Physical motor coordination belongs to the SDK.
  EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
}

TEST(EcJointHardwareHardwareTest, FailedPartialConfigurationClearsEveryBinding) {
  const std::string name = "ec_joint_partial_config";
  ScopedAdapterCleanup cleanup(name);
  auto* adapter = encos::MakeAdapter("Fake", name, name);
  ASSERT_NE(adapter, nullptr);
  adapter->GetFakeAdapterControl()->EnableAutoCreateMotor();
  auto bad = FakeRJointJson("bad", name, 2);
  bad["type"] = "UnsupportedJoint";
  ScopedRuntimeConfig file(
      name, {{"adapters", {{{"type", "Fake"}, {"name", name}}}}, {"joints", {FakeRJointJson("good", name, 1), bad}}});
  EcJointHardware hardware;
  ASSERT_EQ(InitializeHardware(hardware, MakeHardwareInfo({"good", "bad"}, file.path())), CallbackReturn::SUCCESS);
  EXPECT_EQ(hardware.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::ERROR);
  for (std::size_t index = 0; index < 2; ++index) {
    EXPECT_FALSE(EcJointHardwareTestAccess::HasJoint(hardware, index));
    EXPECT_FALSE(EcJointHardwareTestAccess::HasBrake(hardware, index));
  }
  EXPECT_EQ(hardware.on_shutdown(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
}

TEST(EcJointHardwareHardwareTest, FakeNoneModeEngagesBrakeAfterSynchronizedControl) {
  const std::string name = "ec_joint_none_brake";
  ScopedAdapterCleanup cleanup(name);
  auto* adapter = encos::MakeAdapter("Fake", name, name);
  ASSERT_NE(adapter, nullptr);
  auto* control = adapter->GetFakeAdapterControl();
  control->EnableAutoCreateMotor();
  control->EnablePositionError(false);
  auto properties = encos::GetMotorModelRanges(encos::MotorModel::EC_A4310_P2);
  properties.brake = true;
  ASSERT_NE(adapter->GetBus(0)->GetMotor(1, properties), nullptr);
  ScopedRuntimeConfig file(
      name, {{"adapters", {{{"type", "Fake"}, {"name", name}}}}, {"joints", {FakeRJointJson(kJointName, name, 1)}}});
  EcJointHardware hardware;
  ASSERT_EQ(InitializeHardware(hardware, MakeHardwareInfo({kJointName}, file.path())), CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  ASSERT_TRUE(SeedFakeMotorFeedback(*adapter, 1));
  ASSERT_EQ(hardware.on_activate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  BindMode(hardware, {kJointName}, {"current"});
  EcJointHardwareTestAccess::SetCommand(hardware);
  ASSERT_EQ(Write(hardware), hardware_interface::return_type::OK);
  ASSERT_EQ(EcJointHardwareTestAccess::State(hardware), BrakeState::Released);
  adapter->SetSyncMode(true);
  FakeCommandRecorder recorder(*control);
  BindMode(hardware, {kJointName}, {});
  for (int cycle = 0; cycle < 3; ++cycle) ASSERT_EQ(Write(hardware), hardware_interface::return_type::OK);
  EXPECT_EQ(EcJointHardwareTestAccess::State(hardware), BrakeState::Engaged);
  const auto records = recorder.Snapshot();
  ASSERT_EQ(records.size(), 1U);
  ASSERT_EQ(records.front().kind, encos::FakeCommandKind::Brake);
  const auto* brake = std::get_if<encos::FakeBrakePayload>(&records.front().payload);
  ASSERT_NE(brake, nullptr);
  EXPECT_TRUE(brake->enabled);
}

TEST(EcJointHardwareHardwareTest, FakeCommitFailureReturnsErrorAndLifecycleEngagesBrake) {
  const std::string name = "ec_joint_commit_failure";
  ScopedAdapterCleanup cleanup(name);
  auto* adapter = encos::MakeAdapter("Fake", name, name);
  ASSERT_NE(adapter, nullptr);
  auto* control = adapter->GetFakeAdapterControl();
  control->EnableAutoCreateMotor();
  control->EnablePositionError(false);
  auto properties = encos::GetMotorModelRanges(encos::MotorModel::EC_A4310_P2);
  properties.brake = true;
  ASSERT_NE(adapter->GetBus(0)->GetMotor(1, properties), nullptr);
  ScopedRuntimeConfig file(
      name, {{"adapters", {{{"type", "Fake"}, {"name", name}}}}, {"joints", {FakeRJointJson(kJointName, name, 1)}}});
  EcJointHardware hardware;
  ASSERT_EQ(InitializeHardware(hardware, MakeHardwareInfo({kJointName}, file.path())), CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  ASSERT_TRUE(SeedFakeMotorFeedback(*adapter, 1));
  ASSERT_EQ(hardware.on_activate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  BindMode(hardware, {kJointName}, {"current"});
  // Hold and release must also reach the driver when a previous cycle enabled buffering.
  adapter->SetSyncMode(true);
  EcJointHardwareTestAccess::SetCommand(hardware);
  ASSERT_EQ(Write(hardware), hardware_interface::return_type::OK);
  ASSERT_TRUE(EcJointHardwareTestAccess::BrakeReleased(hardware));
  adapter->SetSyncMode(true);
  bool dispatched = false;
  control->SetDecodedCommandObserver([&](const auto& record) {
    if (record.kind == encos::FakeCommandKind::CurControl) {
      dispatched = true;
      throw std::runtime_error("injected commit transport failure");
    }
  });
  EcJointHardwareTestAccess::SetCommand(hardware);
  const auto result = Write(hardware);
  control->ClearDecodedCommandObserver();
  EXPECT_TRUE(dispatched);
  EXPECT_EQ(result, hardware_interface::return_type::ERROR);
  EXPECT_TRUE(EcJointHardwareTestAccess::BrakeReleased(hardware));
  EXPECT_EQ(hardware.on_error(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  EXPECT_FALSE(EcJointHardwareTestAccess::BrakeReleased(hardware));
}

}  // namespace encos::ec_joint_hardware

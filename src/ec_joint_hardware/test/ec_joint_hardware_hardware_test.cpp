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

  void PVTControl(float kp, float kd, float position, float velocity, float effort) override {
    pvt_commands.push_back({kp, kd, position, velocity, effort, encos::PVTControlMode::MotorPVT});
  }

  void PVTControl(float kp, float kd, float position, float velocity, float effort,
                  encos::PVTControlMode torque_mode) override {
    pvt_commands.push_back({kp, kd, position, velocity, effort, torque_mode});
    if (throw_on_pvt) {
      throw std::runtime_error("injected PVT failure");
    }
  }

  void PosControl(float position, float velocity, float current, int feedback) override {
    position_commands.push_back({position, velocity, current, feedback});
  }

  void SpdControl(float velocity, float current, int feedback) override {
    velocity_commands.push_back({velocity, current, feedback});
  }

  void CurControl(float current, int feedback) override { current_commands.push_back({current, feedback}); }

  void TorControl(float effort, int feedback) override { torque_commands.push_back({effort, feedback}); }

  void Stop(encos::MotorStopMode mode, float current, int feedback) override {
    stop_commands.push_back({mode, current, feedback});
  }

  bool SetZero() override { return true; }
  float GotoLimit(int, float, float, std::chrono::milliseconds) override { return 0.0F; }
  bool GotoZero(float, float, float, std::chrono::milliseconds) override { return true; }
  bool Calibrate(int, float, float, std::chrono::milliseconds) override { return true; }
  float PosAsOffset(int) override { return 0.0F; }
  void SetOnStatus(std::function<void(const encos::JointStatus&)>) override {}
  void SetLimit(encos::Range<float>) override {}

  std::optional<encos::JointStatus> GetStatus(bool auto_send_command, int life_cycle_deduction) override {
    status_calls.push_back({auto_send_command, life_cycle_deduction});
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
};

bool IsNan(double value) { return std::isnan(value); }

}  // namespace

struct EcJointHardwareTestAccess {
  static void SetConfigured(EcJointHardware& hardware, bool configured) { hardware.configured_ = configured; }
  static void SetActive(EcJointHardware& hardware, bool active) { hardware.active_ = active; }
  static int StatusMaxLifeCycle(const EcJointHardware& hardware) { return hardware.status_max_life_cycle_; }
  static bool IsLogged(const EcJointHardware& hardware) { return hardware.manager_.IsLogged(); }
  static bool CanfdEnabled(const EcJointHardware& hardware) { return hardware.canfd_enable_; }
  static bool ManagerCanfdEnabled(const EcJointHardware& hardware) { return hardware.manager_.IsJointMotorCanfd(); }
  static bool IsConfigured(const EcJointHardware& hardware) { return hardware.configured_; }

  static void SetJoint(EcJointHardware& hardware, const std::shared_ptr<MockJoint>& joint) {
    hardware.joints_.front().ptr = joint;
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
  ASSERT_EQ(hardware.on_init(MakeHardwareInfo()), CallbackReturn::SUCCESS);
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

  ASSERT_EQ(hardware.on_init(MakeHardwareInfo()), CallbackReturn::SUCCESS);
  EXPECT_EQ(EcJointHardwareTestAccess::StatusMaxLifeCycle(hardware), 10);
}

TEST(EcJointHardwareHardwareTest, RequiresAtLeastOneCompleteAdapterSelection) {
  EcJointHardware hardware;
  auto info = MakeHardwareInfo();
  info.hardware_parameters.erase("adapter_type_1");
  info.hardware_parameters.erase("interface_name_1");

  EXPECT_EQ(hardware.on_init(info), CallbackReturn::ERROR);
}

TEST(EcJointHardwareHardwareTest, RejectsIncompleteOrEmptyAdapterSelection) {
  for (const auto& parameters : {
           std::pair<std::string, std::string>{"adapter_type_1", ""},
           std::pair<std::string, std::string>{"interface_name_1", ""},
       }) {
    EcJointHardware hardware;
    auto info = MakeHardwareInfo();
    info.hardware_parameters[parameters.first] = parameters.second;

    EXPECT_EQ(hardware.on_init(info), CallbackReturn::ERROR);
  }

  {
    EcJointHardware hardware;
    auto info = MakeHardwareInfo();
    info.hardware_parameters.erase("interface_name_1");
    EXPECT_EQ(hardware.on_init(info), CallbackReturn::ERROR);
  }
}

TEST(EcJointHardwareHardwareTest, AcceptsMultipleNumberedAdapterSelections) {
  EcJointHardware hardware;
  auto info = MakeHardwareInfo();
  info.hardware_parameters["adapter_type_2"] = "Ethercat";
  info.hardware_parameters["interface_name_2"] = "enp86s0";

  EXPECT_EQ(hardware.on_init(info), CallbackReturn::SUCCESS);
}

TEST(EcJointHardwareHardwareTest, AcceptsPositiveStatusMaxLifeCycleParameter) {
  EcJointHardware hardware;
  auto info = MakeHardwareInfo();
  info.hardware_parameters["status_max_life_cycle"] = "7";

  ASSERT_EQ(hardware.on_init(info), CallbackReturn::SUCCESS);
  EXPECT_EQ(EcJointHardwareTestAccess::StatusMaxLifeCycle(hardware), 7);
}

TEST(EcJointHardwareHardwareTest, UsesAtoiStyleStatusMaxLifeCycleParsing) {
  EcJointHardware hardware;
  auto info = MakeHardwareInfo();
  info.hardware_parameters["status_max_life_cycle"] = "3.5";

  ASSERT_EQ(hardware.on_init(info), CallbackReturn::SUCCESS);
  EXPECT_EQ(EcJointHardwareTestAccess::StatusMaxLifeCycle(hardware), 3);
}

TEST(EcJointHardwareHardwareTest, RejectsInvalidStatusMaxLifeCycleParameter) {
  for (const auto& value : {"", "0", "-1", "invalid"}) {
    EcJointHardware hardware;
    auto info = MakeHardwareInfo();
    info.hardware_parameters["status_max_life_cycle"] = value;

    EXPECT_EQ(hardware.on_init(info), CallbackReturn::ERROR) << "value: " << value;
  }
}

TEST(EcJointHardwareHardwareTest, EnablesCanfdWhenParameterIsOmitted) {
  EcJointHardware hardware;
  ASSERT_EQ(hardware.on_init(MakeHardwareInfo()), CallbackReturn::SUCCESS);
  EXPECT_TRUE(EcJointHardwareTestAccess::CanfdEnabled(hardware));
}

TEST(EcJointHardwareHardwareTest, AcceptsCanfdBooleanSpellings) {
  for (const auto& value : {"true", "True", "TRUE", "1", "false", "False", "FALSE", "0"}) {
    SCOPED_TRACE(value);
    EcJointHardware hardware;
    auto info = MakeHardwareInfo();
    info.hardware_parameters["canfd_enable"] = value;
    ASSERT_EQ(hardware.on_init(info), CallbackReturn::SUCCESS);
    const bool enabled = value[0] == 't' || value[0] == 'T' || value[0] == '1';
    EXPECT_EQ(EcJointHardwareTestAccess::CanfdEnabled(hardware), enabled);
  }
}

TEST(EcJointHardwareHardwareTest, RejectsInvalidCanfdParameters) {
  for (const auto& value : {"", "yes", "no", "2", "-1", " true ", "false ", "\ttrue", "TrUe", "False\n"}) {
    SCOPED_TRACE(value);
    EcJointHardware hardware;
    auto info = MakeHardwareInfo();
    info.hardware_parameters["canfd_enable"] = value;
    EXPECT_EQ(hardware.on_init(info), CallbackReturn::ERROR);
    EXPECT_FALSE(EcJointHardwareTestAccess::IsConfigured(hardware));
  }
}

TEST(EcJointHardwareHardwareTest, RestoresDefaultCanfdOnInitializationWithOmittedParameter) {
  EcJointHardware hardware;
  auto info = MakeHardwareInfo();
  info.hardware_parameters["canfd_enable"] = "false";
  ASSERT_EQ(hardware.on_init(info), CallbackReturn::SUCCESS);
  ASSERT_FALSE(EcJointHardwareTestAccess::CanfdEnabled(hardware));

  ASSERT_EQ(hardware.on_init(MakeHardwareInfo()), CallbackReturn::SUCCESS);
  EXPECT_TRUE(EcJointHardwareTestAccess::CanfdEnabled(hardware));
}

class EcJointHardwareCanfdTest : public testing::TestWithParam<std::string> {};

TEST_P(EcJointHardwareCanfdTest, ConfigureAndReconfigureApplyCanfdToTransmittedFrames) {
  const bool enabled = GetParam() != "false";
  const std::string adapter_name = "ec_joint_canfd_" + (GetParam().empty() ? "default" : GetParam());
  ScopedAdapterCleanup adapter_cleanup(adapter_name);
  auto* adapter = encos::MakeAdapter("Fake", adapter_name, adapter_name);
  ASSERT_NE(adapter, nullptr);
  auto* control = adapter->GetFakeAdapterControl();
  ASSERT_NE(control, nullptr);
  control->EnableAutoCreateMotor();
  control->SetReplyMode(encos::FakeReplyMode::Automatic);
  const nlohmann::json config = {
      {"adapters", {{{"type", "Fake"}, {"name", adapter_name}}}},
      {"joints", {FakeRJointJson(kJointName, adapter_name, 1)}},
  };
  ScopedRuntimeConfig config_file(adapter_name, config);
  EcJointHardware hardware;
  auto info = MakeHardwareInfo({kJointName}, config_file.path());
  if (!GetParam().empty()) {
    info.hardware_parameters["canfd_enable"] = GetParam();
  }
  ASSERT_EQ(hardware.on_init(info), CallbackReturn::SUCCESS);
  FakeCommandRecorder recorder(*control);

  for (int attempt = 0; attempt < 2; ++attempt) {
    SCOPED_TRACE(attempt);
    ASSERT_EQ(hardware.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
    EXPECT_EQ(EcJointHardwareTestAccess::ManagerCanfdEnabled(hardware), enabled);
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
  ASSERT_EQ(hardware.on_init(info), CallbackReturn::SUCCESS);
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
  ASSERT_EQ(hardware.on_init(MakeHardwareInfo({kJointName}, config_file.path())), CallbackReturn::SUCCESS);
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

  ASSERT_EQ(hardware.on_init(info), CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);

  EXPECT_TRUE(EcJointHardwareTestAccess::IsLogged(hardware));
  EXPECT_TRUE(HasLogFileWithSuffix(log_directory.path(), "_command.csv.zstd"));
  EXPECT_TRUE(HasLogFileWithSuffix(log_directory.path(), "_status.csv.zstd"));
}

TEST(EcJointHardwareHardwareTest, ExportsExactEightCommandAndSixStateInterfaces) {
  EcJointHardware hardware;
  ASSERT_EQ(hardware.on_init(MakeHardwareInfo()), CallbackReturn::SUCCESS);

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

  // 不写任何必填值：绑定模式的关节本周期无命令，跳过且不视为错误
  EXPECT_EQ(Write(hardware), hardware_interface::return_type::OK);
  EXPECT_TRUE(joint->pvt_commands.empty());
  EXPECT_TRUE(joint->position_commands.empty());
  EXPECT_TRUE(joint->stop_commands.empty());
  ExpectCommandsReset(hardware);
}

TEST(EcJointHardwareHardwareTest, UnboundJointSendsZeroTorquePvtFeedbackCommand) {
  EcJointHardware hardware;
  const auto joint = std::make_shared<MockJoint>();
  PrepareActiveHardware(hardware, joint);

  // 未绑定 controller 时发送零扭矩 PVT，维持底层反馈请求链路。
  EXPECT_EQ(Write(hardware), hardware_interface::return_type::OK);
  ASSERT_EQ(joint->pvt_commands.size(), 1U);
  EXPECT_FLOAT_EQ(joint->pvt_commands.front().kp, 0.0F);
  EXPECT_FLOAT_EQ(joint->pvt_commands.front().kd, 0.1F);
  EXPECT_FLOAT_EQ(joint->pvt_commands.front().position, 0.0F);
  EXPECT_FLOAT_EQ(joint->pvt_commands.front().velocity, 0.0F);
  EXPECT_FLOAT_EQ(joint->pvt_commands.front().effort, 0.0F);
  EXPECT_EQ(joint->pvt_commands.front().torque_mode, encos::PVTControlMode::MotorPVT);
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

TEST(EcJointHardwareHardwareTest, ActivationRequiresAndPublishesFullFiniteFeedback) {
  EcJointHardware hardware;
  const auto joint = std::make_shared<MockJoint>();
  joint->status = MakeStatus();
  ASSERT_EQ(hardware.on_init(MakeHardwareInfo()), CallbackReturn::SUCCESS);
  EcJointHardwareTestAccess::SetJoint(hardware, joint);
  EcJointHardwareTestAccess::SetConfigured(hardware, true);

  EXPECT_EQ(hardware.on_activate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  EXPECT_EQ(StateValues(hardware),
            (std::vector<double>{1.0, 2.0, 3.0, 4.0, 5.0, static_cast<double>(encos::MotorError::OverCurrent)}));
  ASSERT_EQ(joint->status_calls.size(), 1U);
  EXPECT_TRUE(joint->status_calls.front().auto_send_command);
  EXPECT_EQ(joint->status_calls.front().life_cycle_deduction, 0);
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

  ASSERT_EQ(hardware.on_init(info), CallbackReturn::SUCCESS);
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
  ASSERT_EQ(hardware.on_init(MakeHardwareInfo({kFirstJoint, kSecondJoint}, config_file.path())),
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

}  // namespace encos::ec_joint_hardware

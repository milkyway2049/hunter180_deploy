#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <controller_interface/controller_interface_params.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <nlohmann/json.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{
using CallbackReturn = controller_interface::CallbackReturn;
using Return = controller_interface::return_type;

class WalkControllerAmpTest : public testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    std::ifstream input(AMP_REFERENCE_PATH);
    input >> reference_;
    names_ = reference_.at("joint_names").get<std::vector<std::string>>();
    log_dir_ = std::filesystem::temp_directory_path() /
      ("hunter180_amp_test_" + std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count()));
    loader_ = std::make_unique<pluginlib::ClassLoader<controller_interface::ControllerInterface>>(
      "controller_interface", "controller_interface::ControllerInterface");
    controller_ = loader_->createSharedInstance("ec/WalkController");
  }

  void TearDown() override
  {
    if (activated_) {controller_->on_deactivate(rclcpp_lifecycle::State{});}
    if (controller_) {controller_->release_interfaces();}
    controller_.reset();
    loader_.reset();
    std::filesystem::remove_all(log_dir_);
  }

  void initialize(std::vector<rclcpp::Parameter> overrides = {})
  {
    overrides.emplace_back("log_dir", log_dir_.string());
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "walk_controller";
    params.update_rate = 500;
    params.controller_manager_update_rate = 500;
    params.node_options.use_global_arguments(false);
    params.node_options.arguments({"--ros-args", "--params-file", AMP_CONFIG_PATH});
    params.node_options.parameter_overrides(overrides);
    ASSERT_EQ(controller_->init(params), Return::OK);
  }

  void loadState(size_t step)
  {
    const auto & state = reference_.at("steps").at(step);
    for (size_t i = 0; i < 23; ++i) {
      position_[i] = state.at("position").at(i).get<double>();
      velocity_[i] = state.at("velocity").at(i).get<double>();
    }
    for (size_t i = 0; i < 4; ++i) {quaternion_[i] = state.at("quaternion_xyzw").at(i);}
    for (size_t i = 0; i < 3; ++i) {gyro_[i] = state.at("angular_velocity").at(i);}
  }

  void assignInterfaces()
  {
    std::vector<hardware_interface::LoanedStateInterface> state_loans;
    std::vector<hardware_interface::LoanedCommandInterface> command_loans;
    const auto state = [&](const std::string & prefix, const std::string & name, double * value) {
      states_.push_back(std::make_unique<hardware_interface::StateInterface>(prefix, name, value));
      state_loans.emplace_back(*states_.back());
    };
    const std::array<std::string, 6> command_names = {
      "position", "velocity", "effort", "kp", "kd", "torque_mode"};
    for (size_t i = 0; i < names_.size(); ++i) {
      state(names_[i], "position", &position_[i]);
      state(names_[i], "velocity", &velocity_[i]);
      state(names_[i], "effort", &effort_[i]);
      for (size_t j = 0; j < command_names.size(); ++j) {
        commands_.push_back(std::make_unique<hardware_interface::CommandInterface>(
          names_[i], command_names[j], &command_values_[i][j]));
        command_loans.emplace_back(*commands_.back());
      }
    }
    const std::array<std::string, 4> axes = {"x", "y", "z", "w"};
    for (size_t i = 0; i < 4; ++i) {state("imu_sensor", "orientation." + axes[i], &quaternion_[i]);}
    for (size_t i = 0; i < 3; ++i) {
      state("imu_sensor", "angular_velocity." + axes[i], &gyro_[i]);
      state("imu_sensor", "linear_acceleration." + axes[i], &acceleration_[i]);
    }
    controller_->assign_interfaces(std::move(command_loans), std::move(state_loans));
  }

  nlohmann::json reference_;
  std::vector<std::string> names_;
  std::filesystem::path log_dir_;
  std::unique_ptr<pluginlib::ClassLoader<controller_interface::ControllerInterface>> loader_;
  std::shared_ptr<controller_interface::ControllerInterface> controller_;
  std::vector<std::unique_ptr<hardware_interface::StateInterface>> states_;
  std::vector<std::unique_ptr<hardware_interface::CommandInterface>> commands_;
  std::array<double, 23> position_{}, velocity_{}, effort_{};
  std::array<double, 4> quaternion_{};
  std::array<double, 3> gyro_{}, acceleration_{};
  std::array<std::array<double, 6>, 23> command_values_{};
  bool activated_{false};
};

TEST_F(WalkControllerAmpTest, MatchesCheckpointAcrossHistoryAndReactivation)
{
  initialize();
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  assignInterfaces();
  int64_t ticks = 0;
  for (int activation = 0; activation < 2; ++activation) {
    loadState(0);
    ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
    activated_ = true;
    const auto kp = controller_->get_node()->get_parameter("walk_kp").as_double_array();
    const auto kd = controller_->get_node()->get_parameter("walk_kd").as_double_array();
    const auto defaults = controller_->get_node()->get_parameter("walk_pos").as_double_array();
    for (size_t step = 0; step < reference_.at("steps").size(); ++step) {
      loadState(step);
      for (int cycle = 0; cycle < 10; ++cycle) {
        const auto time = rclcpp::Time(++ticks * 2000000, RCL_ROS_TIME);
        ASSERT_EQ(controller_->update(time, rclcpp::Duration::from_seconds(0.002)), Return::OK);
        if (step == 0 && cycle < 9) {
          for (size_t i = 0; i < 23; ++i) {EXPECT_DOUBLE_EQ(command_values_[i][0], defaults[i]);}
        }
      }
      for (size_t i = 0; i < 23; ++i) {
        SCOPED_TRACE(names_[i] + " step=" + std::to_string(step));
        EXPECT_NEAR(command_values_[i][0], reference_["steps"][step]["expected_position"][i].get<double>(), 1e-5);
        EXPECT_DOUBLE_EQ(command_values_[i][3], kp[i]);
        EXPECT_DOUBLE_EQ(command_values_[i][4], kd[i]);
        const bool coupled = i == 4 || i == 5 || i == 10 || i == 11 || i == 13 || i == 14;
        EXPECT_DOUBLE_EQ(command_values_[i][5], coupled ? 2.0 : 0.0);
      }
    }
    ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
    activated_ = false;
  }
}

TEST_F(WalkControllerAmpTest, RejectsIncompatibleHistoryBeforeActivation)
{
  initialize({rclcpp::Parameter("action.num_history", 10)});
  EXPECT_EQ(controller_->on_configure(rclcpp_lifecycle::State{}), CallbackReturn::ERROR);
}

TEST_F(WalkControllerAmpTest, RejectsJointOrderMismatchBeforeActivation)
{
  auto wrong_order = names_;
  std::swap(wrong_order[0], wrong_order[6]);
  initialize({rclcpp::Parameter("joints", wrong_order)});
  EXPECT_EQ(controller_->on_configure(rclcpp_lifecycle::State{}), CallbackReturn::ERROR);
}

TEST_F(WalkControllerAmpTest, RejectsDefaultPoseMismatchBeforeActivation)
{
  initialize({rclcpp::Parameter("walk_pos", std::vector<double>(23, 0.0))});
  EXPECT_EQ(controller_->on_configure(rclcpp_lifecycle::State{}), CallbackReturn::ERROR);
}
}  // namespace

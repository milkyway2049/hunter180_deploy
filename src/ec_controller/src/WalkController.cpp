// =============================================================================
//  Copyright (c) 2026 Phanerox. All rights reserved.
// =============================================================================

#include <pluginlib/class_list_macros.hpp>
#include "ec_controller/ControllerBase.h"

#include <ec_radio/msg/radio.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <onnxruntime_cxx_api.h>

#include <cmath>
#include <cstring>
#include <algorithm>
#include <array>
#include <atomic>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <memory>
#include <string>
#include <vector>

namespace ec
{

// AMP_mjlab policy and YAML share the robot's physical joint order:
// L_leg(6), R_leg(6), waist(3), L_arm(4), R_arm(4).
// Model metadata is checked before activation; no left/right interleaving.

// IMU interface names (must match ec_imu_hardware exports)
static const std::vector<std::string> kImuInterfaces = {
  "imu_sensor/orientation.x",
  "imu_sensor/orientation.y",
  "imu_sensor/orientation.z",
  "imu_sensor/orientation.w",
  "imu_sensor/angular_velocity.x",
  "imu_sensor/angular_velocity.y",
  "imu_sensor/angular_velocity.z",
  "imu_sensor/linear_acceleration.x",
  "imu_sensor/linear_acceleration.y",
  "imu_sensor/linear_acceleration.z",
};

class WalkController : public ControllerBase {
public:
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override
  {
    auto ret = ControllerBase::on_configure(previous_state);
    if (ret != controller_interface::CallbackReturn::SUCCESS) {return ret;}

    auto node = get_node();

    // Joint parameters
    node->declare_parameter<std::vector<double>>("walk_pos", std::vector<double>{});
    node->declare_parameter<std::vector<double>>("walk_kp", std::vector<double>{});
    node->declare_parameter<std::vector<double>>("walk_kd", std::vector<double>{});

    default_pos_ = node->get_parameter("walk_pos").as_double_array();
    walk_kp_ = node->get_parameter("walk_kp").as_double_array();
    walk_kd_ = node->get_parameter("walk_kd").as_double_array();

    // Validate joint parameter sizes
    const size_t n = jointCount();
    if (n != 23 || default_pos_.size() != n || walk_kp_.size() != n || walk_kd_.size() != n) {
      RCLCPP_ERROR(logger(),
          "Parameter array size mismatch: expected %zu, got pos=%zu kp=%zu kd=%zu",
                   n, default_pos_.size(), walk_kp_.size(), walk_kd_.size());
      return controller_interface::CallbackReturn::ERROR;
    }

    // Action parameters
    node->declare_parameter<int>("action.action_size", 23);
    node->declare_parameter<int>("action.observation_size", 78);
    node->declare_parameter<int>("action.num_history", 4);
    node->declare_parameter<int>("action.policy_freq", 50);
    node->declare_parameter<std::vector<double>>("action.action_scale",
        std::vector<double>(23, 0.25));

    action_size_ = node->get_parameter("action.action_size").as_int();
    obs_size_ = node->get_parameter("action.observation_size").as_int();
    num_history_ = node->get_parameter("action.num_history").as_int();
    int policy_freq = node->get_parameter("action.policy_freq").as_int();
    action_scale_ = node->get_parameter("action.action_scale").as_double_array();

    if (action_size_ != 23 || num_history_ <= 0 || policy_freq <= 0 ||
        policy_freq > static_cast<int>(get_update_rate())) {
      RCLCPP_ERROR(logger(), "Invalid AMP action/history/frequency configuration");
      return controller_interface::CallbackReturn::ERROR;
    }
    inference_period_ = 1.0 / policy_freq;

    // Validate observation size: 9 (ang_vel + gravity + cmd) + 2*n (pos + vel) + action_size
    int expected_obs = 9 + 2 * static_cast<int>(n) + action_size_;
    if (obs_size_ != expected_obs) {
      RCLCPP_ERROR(logger(), "observation_size mismatch: expected %d (9 + 2*%zu + %d), got %d",
                   expected_obs, n, action_size_, obs_size_);
      return controller_interface::CallbackReturn::ERROR;
    }
    if (static_cast<int>(action_scale_.size()) != action_size_) {
      RCLCPP_ERROR(logger(), "action_scale size %zu < action_size %d",
                   action_scale_.size(), action_size_);
      return controller_interface::CallbackReturn::ERROR;
    }

    // Empirical normalization is embedded in ONNX. Zero disables clipping,
    // matching this training run's unclipped observations and actions.
    node->declare_parameter<double>("normalization.clip_scales.clip_observations", 0.0);
    node->declare_parameter<double>("normalization.clip_scales.clip_actions", 0.0);
    clip_obs_ = node->get_parameter("normalization.clip_scales.clip_observations").as_double();
    clip_actions_ = node->get_parameter("normalization.clip_scales.clip_actions").as_double();

    // Policy path
    node->declare_parameter<std::string>("policy_path", "");
    std::string policy_path = node->get_parameter("policy_path").as_string();

    // Resolve relative path against package share directory
    if (!policy_path.empty() && !std::filesystem::path(policy_path).is_absolute()) {
      std::string share_dir = ament_index_cpp::get_package_share_directory("ec_controller");
      policy_path = share_dir + "/" + policy_path;
    }

    // Initialize buffers
    actions_.assign(action_size_, 0.0f);
    obs_.resize(obs_size_);

    // History buffer: [num_history, obs_size]
    obs_history_.assign(num_history_ * obs_size_, 0.0f);

    // Load ONNX model
    if (!policy_path.empty()) {
      try {
        ort_env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "WalkController");
        Ort::SessionOptions session_opts;
        session_opts.SetIntraOpNumThreads(1);
        ort_session_ = std::make_unique<Ort::Session>(*ort_env_, policy_path.c_str(), session_opts);
        validatePolicyModel();

        // Get input/output names
        Ort::AllocatorWithDefaultOptions alloc;
        auto in_name = ort_session_->GetInputNameAllocated(0, alloc);
        auto out_name = ort_session_->GetOutputNameAllocated(0, alloc);
        input_name_ = in_name.get();
        output_name_ = out_name.get();

        input_names_ = {input_name_.c_str()};
        output_names_ = {output_name_.c_str()};

        // Pre-allocate ONNX tensor resources
        mem_info_ = Ort::MemoryInfo::CreateCpu(
          OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
        input_shape_ = {1, num_history_ * obs_size_};

        RCLCPP_INFO(logger(), "Loaded policy: %s (obs=%d, act=%d, history=%d)",
                    policy_path.c_str(), obs_size_, action_size_, num_history_);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(logger(), "Failed to load ONNX model: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
      }
    } else {
      RCLCPP_WARN(logger(), "No policy_path set, inference disabled");
    }

    // Subscribe to radio for command input
    radio_sub_ = node->create_subscription<ec_radio::msg::Radio>(
      "/radio", 10,
      [this](const ec_radio::msg::Radio::SharedPtr msg) {
        radio_vx_.store(msg->vx);
        radio_vy_.store(msg->vy);
        radio_wz_.store(msg->wz);
      });

    node->declare_parameter<std::string>("log_dir", "/home/milkyway/log/hunter180");
    log_dir_ = node->get_parameter("log_dir").as_string();

    RCLCPP_INFO(logger(), "WalkController configured");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override
  {
    auto ret = ControllerBase::on_activate(previous_state);
    if (ret != controller_interface::CallbackReturn::SUCCESS) {return ret;}

    // Fill history buffer with current standing observation at activation time
    // so the first policy inference has proper context from the current pose.
    const size_t n = jointCount();
    inference_accumulator_ = 0.0;
    infer_count_ = 0;

    // Read current IMU from state interfaces
    std::array<double, 3> ang_vel = {0.0, 0.0, 0.0};
    std::array<double, 4> quat = {0.0, 0.0, 0.0, 1.0};
    for (auto & iface : state_interfaces_) {
      const auto & name = iface.get_prefix_name();
      const auto & iface_name = iface.get_interface_name();
      if (name != "imu_sensor") {continue;}
      auto val = iface.get_optional();
      if (!val.has_value()) {continue;}
      if (iface_name == "angular_velocity.x") {
        ang_vel[0] = val.value();
      } else if (iface_name == "angular_velocity.y") {
        ang_vel[1] = val.value();
      } else if (iface_name == "angular_velocity.z") {
        ang_vel[2] = val.value();
      } else if (iface_name == "orientation.x") {
        quat[0] = val.value();
      } else if (iface_name == "orientation.y") {
        quat[1] = val.value();
      } else if (iface_name == "orientation.z") {
        quat[2] = val.value();
      } else if (iface_name == "orientation.w") {quat[3] = val.value();}
    }

    // Projected gravity: R^T * [0, 0, -1]
    const double qx = quat[0], qy = quat[1], qz = quat[2], qw = quat[3];
    std::array<double, 3> proj_gravity;
    proj_gravity[0] = 2.0 * (qx * qz - qw * qy) * (-1.0);
    proj_gravity[1] = 2.0 * (qy * qz + qw * qx) * (-1.0);
    proj_gravity[2] = (qw * qw - qx * qx - qy * qy + qz * qz) * (-1.0);

    // Build initial observation from current state
    obs_[0] = static_cast<float>(ang_vel[0]);
    obs_[1] = static_cast<float>(ang_vel[1]);
    obs_[2] = static_cast<float>(ang_vel[2]);
    obs_[3] = static_cast<float>(proj_gravity[0]);
    obs_[4] = static_cast<float>(proj_gravity[1]);
    obs_[5] = static_cast<float>(proj_gravity[2]);
    obs_[6] = 0.0f;  // cmd_vx
    obs_[7] = 0.0f;  // cmd_vy
    obs_[8] = 0.0f;  // cmd_wz

    // Read current joint positions and velocities from state interfaces
    for (size_t i = 0; i < n; ++i) {
      double pos = 0.0, vel = 0.0;
      for (auto & iface : state_interfaces_) {
        if (iface.get_prefix_name() != joint_names_[i]) {continue;}
        auto val = iface.get_optional();
        if (!val.has_value()) {continue;}
        if (iface.get_interface_name() == "position") {
          pos = val.value();
        } else if (iface.get_interface_name() == "velocity") {vel = val.value();}
      }
      obs_[9 + i] = static_cast<float>(pos - default_pos_[i]);
      obs_[32 + i] = static_cast<float>(vel);
    }
    // No previous action at activation
    for (int i = 0; i < action_size_; ++i) {
      obs_[55 + i] = 0.0f;
    }

    // Clip
    if (clip_obs_ > 0.0) {
      for (auto & v : obs_) {
        v = std::clamp(v, static_cast<float>(-clip_obs_), static_cast<float>(clip_obs_));
      }
    }

    // Fill all history frames with current observation
    for (int h = 0; h < num_history_; ++h) {
      std::memcpy(obs_history_.data() + h * obs_size_,
                   obs_.data(), obs_size_ * sizeof(float));
    }
    history_filled_ = true;
    actions_.assign(action_size_, 0.0f);

    openLogFile();
    RCLCPP_INFO(logger(), "WalkController activated, history buffer initialized from current pose");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override
  {
    history_filled_ = false;
    actions_.assign(action_size_, 0.0f);
    closeLogFile();
    return ControllerBase::on_deactivate(previous_state);
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    // Joint state interfaces
    for (const auto & joint_name : joint_names_) {
      config.names.push_back(joint_name + "/position");
      config.names.push_back(joint_name + "/velocity");
      config.names.push_back(joint_name + "/effort");
    }

    // IMU state interfaces
    for (const auto & iface : kImuInterfaces) {
      config.names.push_back(iface);
    }

    return config;
  }

protected:
  void onUpdate(const rclcpp::Time & time, const rclcpp::Duration & period) override
  {
    const size_t n = jointCount();

    // Read IMU data from state interfaces
    std::array<double, 3> ang_vel = {0.0, 0.0, 0.0};
    std::array<double, 4> quat = {0.0, 0.0, 0.0, 1.0};  // (x, y, z, w)

    for (auto & iface : state_interfaces_) {
      const auto & name = iface.get_prefix_name();
      const auto & iface_name = iface.get_interface_name();
      if (name != "imu_sensor") {continue;}

      auto val = iface.get_optional();
      if (!val.has_value()) {continue;}

      if (iface_name == "angular_velocity.x") {
        ang_vel[0] = val.value();
      } else if (iface_name == "angular_velocity.y") {
        ang_vel[1] = val.value();
      } else if (iface_name == "angular_velocity.z") {
        ang_vel[2] = val.value();
      } else if (iface_name == "orientation.x") {
        quat[0] = val.value();
      } else if (iface_name == "orientation.y") {
        quat[1] = val.value();
      } else if (iface_name == "orientation.z") {
        quat[2] = val.value();
      } else if (iface_name == "orientation.w") {quat[3] = val.value();}
    }

    // Compute projected gravity: R^T * [0, 0, -1] (normalized, matching Isaac Lab)
    // q = (x, y, z, w)
    const double qx = quat[0], qy = quat[1], qz = quat[2], qw = quat[3];
    std::array<double, 3> proj_gravity;
    proj_gravity[0] = 2.0 * (qx * qz - qw * qy) * (-1.0);
    proj_gravity[1] = 2.0 * (qy * qz + qw * qx) * (-1.0);
    proj_gravity[2] = (qw * qw - qx * qx - qy * qy + qz * qz) * (-1.0);

    // Read command from radio
    float cmd_vx = radio_vx_.load();
    float cmd_vy = radio_vy_.load();
    float cmd_wz = radio_wz_.load();

    // Accumulate time for inference
    inference_accumulator_ += period.seconds();

    if (inference_accumulator_ >= inference_period_ && ort_session_) {
      // Keep the 50 Hz phase without issuing catch-up inferences after an overrun.
      inference_accumulator_ = std::fmod(inference_accumulator_, inference_period_);

      // Build observation vector
      // [0:3]   ang_vel
      obs_[0] = static_cast<float>(ang_vel[0]);
      obs_[1] = static_cast<float>(ang_vel[1]);
      obs_[2] = static_cast<float>(ang_vel[2]);
      // [3:6]   projected_gravity
      obs_[3] = static_cast<float>(proj_gravity[0]);
      obs_[4] = static_cast<float>(proj_gravity[1]);
      obs_[5] = static_cast<float>(proj_gravity[2]);
      // [6:9]   command
      obs_[6] = cmd_vx;
      obs_[7] = cmd_vy;
      obs_[8] = cmd_wz;
      // [9:32]  joint_pos - default_pos, in AMP_mjlab physical joint order
      for (size_t i = 0; i < n; ++i) {
        obs_[9 + i] = static_cast<float>(jointState(i).position - default_pos_[i]);
      }
      // [32:55] joint_vel (training default velocities are zero)
      for (size_t i = 0; i < n; ++i) {
        obs_[32 + i] = static_cast<float>(jointState(i).velocity);
      }
      // [55:78] prev_action
      for (int i = 0; i < action_size_; ++i) {
        obs_[55 + i] = actions_[i];
      }

      // Clip observations
      if (clip_obs_ > 0.0) {
        for (auto & v : obs_) {
          v = std::clamp(v, static_cast<float>(-clip_obs_), static_cast<float>(clip_obs_));
        }
      }

      // Update history buffer
      if (!history_filled_) {
        // First frame: fill entire buffer with current observation
        for (int h = 0; h < num_history_; ++h) {
          std::memcpy(obs_history_.data() + h * obs_size_,
                       obs_.data(), obs_size_ * sizeof(float));
        }
        history_filled_ = true;
      } else {
        // Normal: shift left and append latest
        std::memmove(obs_history_.data(),
                     obs_history_.data() + obs_size_,
          (num_history_ - 1) * obs_size_ * sizeof(float));
        std::memcpy(obs_history_.data() + (num_history_ - 1) * obs_size_,
                     obs_.data(), obs_size_ * sizeof(float));
      }

      // Run ONNX inference with history as input
      try {
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
          mem_info_, obs_history_.data(), obs_history_.size(),
          input_shape_.data(), input_shape_.size());

        auto output_tensors = ort_session_->Run(
          Ort::RunOptions{nullptr},
          input_names_.data(), &input_tensor, 1,
          output_names_.data(), 1);

        const float *raw_actions = output_tensors[0].GetTensorData<float>();

        for (int i = 0; i < action_size_; ++i) {
          actions_[i] = clip_actions_ > 0.0 ? std::clamp(raw_actions[i],
            static_cast<float>(-clip_actions_), static_cast<float>(clip_actions_)) : raw_actions[i];
        }

        infer_count_++;
      } catch (const Ort::Exception & e) {
        RCLCPP_ERROR_THROTTLE(logger(), *get_node()->get_clock(), 1000,
          "ONNX inference failed: %s", e.what());
      }
    }

    // Apply actions: position = default_pos + action_scale * raw_action
    // Reference positions remain unclipped so PD position errors can produce torque.
    for (size_t i = 0; i < n; ++i) {
      auto & cmd = jointCommand(i);
      const double scaled_action = action_scale_[i] * static_cast<double>(actions_[i]);
      cmd.position = default_pos_[i] + scaled_action;
      cmd.velocity = 0.0;
      cmd.effort = 0.0;
      cmd.kp = walk_kp_[i];
      cmd.kd = walk_kd_[i];
      // Coupled joints (DoubleCoupleJoint) use HostKpMotorKd mode
      cmd.torque_mode = kCoupledJoint[i] ? 2.0 : 0.0;
    }
    writeLogRow(time);
  }

private:
  void validatePolicyModel()
  {
    if (ort_session_->GetInputCount() != 1 || ort_session_->GetOutputCount() != 1) {
      throw std::runtime_error("AMP policy must have one input and one output");
    }
    const auto input_type = ort_session_->GetInputTypeInfo(0);
    const auto output_type = ort_session_->GetOutputTypeInfo(0);
    const auto input = input_type.GetTensorTypeAndShapeInfo();
    const auto output = output_type.GetTensorTypeAndShapeInfo();
    const auto input_dims = input.GetShape();
    const auto output_dims = output.GetShape();
    if (input.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
        output.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
        input_dims.size() != 2 || output_dims.size() != 2 ||
        (input_dims[0] != -1 && input_dims[0] != 1) ||
        (output_dims[0] != -1 && output_dims[0] != 1) ||
        input_dims[1] != static_cast<int64_t>(num_history_) * obs_size_ ||
        output_dims[1] != action_size_) {
      throw std::runtime_error("ONNX dimensions/types do not match AMP controller parameters");
    }
    Ort::AllocatorWithDefaultOptions allocator;
    const auto metadata = ort_session_->GetModelMetadata();
    const auto read = [&](const char * key) {
      auto value = metadata.LookupCustomMetadataMapAllocated(key, allocator);
      if (!value) {throw std::runtime_error(std::string("Missing AMP metadata: ") + key);}
      return std::string(value.get());
    };
    std::ostringstream names;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      if (i) {names << ',';}
      names << joint_names_[i];
    }
    if (read("joint_names") != names.str() ||
        read("observation_names") !=
        "base_ang_vel,projected_gravity,command,joint_pos,joint_vel,actions" ||
        read("history_ordering") != "oldest_to_newest" ||
        read("normalization") != "embedded" ||
        std::stoi(read("history_length")) != num_history_ ||
        std::stoi(read("observation_frame_size")) != obs_size_ ||
        std::abs(std::stod(read("policy_frequency")) * inference_period_ - 1.0) > 1e-6) {
      throw std::runtime_error("AMP observation/action metadata does not match configuration");
    }
    const auto check_values = [&](const char * key, const std::vector<double> & configured) {
      std::istringstream stream(read(key));
      std::string token;
      size_t i = 0;
      while (std::getline(stream, token, ',')) {
        const double value = std::stod(token);
        if (i >= configured.size() || !std::isfinite(value) ||
            !std::isfinite(configured[i]) || std::abs(value - configured[i]) > 1e-5) {
          throw std::runtime_error(std::string("AMP metadata mismatch: ") + key);
        }
        ++i;
      }
      if (i != configured.size()) {
        throw std::runtime_error(std::string("AMP metadata length mismatch: ") + key);
      }
    };
    check_values("default_joint_pos", default_pos_);
    check_values("joint_stiffness", walk_kp_);
    check_values("joint_damping", walk_kd_);
    check_values("action_scale", action_scale_);
  }

  // One file per activation, with columns in the configured controller joint order.
  void openLogFile()
  {
    closeLogFile();
    log_file_.clear();
    namespace fs = std::filesystem;
    const fs::path dir = log_dir_.empty() ? fs::path("/home/milkyway/log/hunter180") : fs::path(log_dir_);
    std::error_code error;
    fs::create_directories(dir, error);
    if (error) {
      RCLCPP_WARN(logger(), "Failed to create walk log directory %s: %s",
        dir.c_str(), error.message().c_str());
      return;
    }
    size_t run_id = 1;
    fs::path path;
    do {
      path = dir / ("walk_log_" + std::to_string(run_id++) + ".csv");
    } while (fs::exists(path, error) && !error);
    if (error) {
      RCLCPP_WARN(logger(), "Failed to select walk log file: %s", error.message().c_str());
      return;
    }
    log_path_ = path.string();
    log_file_.open(log_path_, std::ios::out | std::ios::trunc);
    if (!log_file_.is_open()) {
      RCLCPP_WARN(logger(), "Failed to open walk log file: %s", log_path_.c_str());
      return;
    }
    log_file_ << std::setprecision(17) << "time_ns";
    for (const char * field : {"mpos", "mvel", "meff", "cpos", "ckp", "ckd"}) {
      for (size_t i = 0; i < jointCount(); ++i) {
        log_file_ << "," << field << i;
      }
    }
    log_file_ << "\n";
    RCLCPP_INFO(logger(), "Walk log file: %s", log_path_.c_str());
  }

  void closeLogFile()
  {
    if (log_file_.is_open()) {
      log_file_.close();
      RCLCPP_INFO(logger(), "Walk log file closed: %s", log_path_.c_str());
    }
  }

  void writeLogRow(const rclcpp::Time & time)
  {
    if (!log_file_.is_open()) {return;}
    const size_t n = jointCount();
    log_file_ << time.nanoseconds();
    for (size_t i = 0; i < n; ++i) {log_file_ << "," << jointState(i).position;}
    for (size_t i = 0; i < n; ++i) {log_file_ << "," << jointState(i).velocity;}
    for (size_t i = 0; i < n; ++i) {log_file_ << "," << jointState(i).effort;}
    for (size_t i = 0; i < n; ++i) {log_file_ << "," << jointCommand(i).position;}
    for (size_t i = 0; i < n; ++i) {log_file_ << "," << jointCommand(i).kp;}
    for (size_t i = 0; i < n; ++i) {log_file_ << "," << jointCommand(i).kd;}
    // Buffered output; deactivation closes the stream and flushes the remaining rows.
    log_file_ << "\n";
    if (!log_file_) {
      RCLCPP_WARN(logger(), "Walk CSV write failed; logging disabled for this activation: %s",
        log_path_.c_str());
      log_file_.close();
    }
  }

  // Joint parameters
  std::vector<double> default_pos_;
  std::vector<double> walk_kp_;
  std::vector<double> walk_kd_;

  // Action parameters
  int action_size_{23};
  int obs_size_{78};
  int num_history_{4};
  double inference_period_{0.02};
  double inference_accumulator_{0.0};
  std::vector<double> action_scale_;
  double clip_obs_{0.0};
  double clip_actions_{0.0};

  // Inference buffers
  std::vector<float> actions_;
  std::vector<float> obs_;
  std::vector<float> obs_history_;
  int infer_count_{0};
  bool history_filled_{false};

  // ONNX Runtime
  std::ofstream log_file_;
  std::string log_path_;
  std::string log_dir_;
  std::unique_ptr<Ort::Env> ort_env_;
  std::unique_ptr<Ort::Session> ort_session_;
  std::string input_name_;
  std::string output_name_;
  std::vector<const char *> input_names_;
  std::vector<const char *> output_names_;
  Ort::MemoryInfo mem_info_{nullptr};
  std::vector<int64_t> input_shape_;

  // Radio command (lock-free)
  rclcpp::Subscription<ec_radio::msg::Radio>::SharedPtr radio_sub_;
  std::atomic<float> radio_vx_{0.0f};
  std::atomic<float> radio_vy_{0.0f};
  std::atomic<float> radio_wz_{0.0f};
};

} // namespace ec

PLUGINLIB_EXPORT_CLASS(ec::WalkController, controller_interface::ControllerInterface)

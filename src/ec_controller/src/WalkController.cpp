// =============================================================================
//  Copyright (c) 2026 Phanerox. All rights reserved.
// =============================================================================

#include <pluginlib/class_list_macros.hpp>
#include "ec_controller/ControllerBase.h"

#include <ec_radio/msg/radio.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <onnxruntime_cxx_api.h>

#include <cmath>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <string>
#include <vector>

namespace ec
{

// YAML joint order (controllers.yaml): L_leg(6), R_leg(6), waist(3), L_arm(4), R_arm(4)
//   [ 0] left_hip_pitch_joint
//   [ 1] left_hip_roll_joint
//   [ 2] left_hip_yaw_joint
//   [ 3] left_knee_joint
//   [ 4] left_ankle_pitch_joint
//   [ 5] left_ankle_roll_joint
//   [ 6] right_hip_pitch_joint
//   [ 7] right_hip_roll_joint
//   [ 8] right_hip_yaw_joint
//   [ 9] right_knee_joint
//   [10] right_ankle_pitch_joint
//   [11] right_ankle_roll_joint
//   [12] waist_yaw_joint
//   [13] waist_roll_joint
//   [14] waist_pitch_joint
//   [15] left_shoulder_pitch_joint
//   [16] left_shoulder_roll_joint
//   [17] left_shoulder_yaw_joint
//   [18] left_elbow_joint
//   [19] right_shoulder_pitch_joint
//   [20] right_shoulder_roll_joint
//   [21] right_shoulder_yaw_joint
//   [22] right_elbow_joint
//
// NN joint order (from readme.md): L/R interleaved
//   [ 0] left_hip_pitch_joint
//   [ 1] right_hip_pitch_joint
//   [ 2] waist_yaw_joint
//   [ 3] left_hip_roll_joint
//   [ 4] right_hip_roll_joint
//   [ 5] waist_roll_joint
//   [ 6] left_hip_yaw_joint
//   [ 7] right_hip_yaw_joint
//   [ 8] waist_pitch_joint
//   [ 9] left_knee_joint
//   [10] right_knee_joint
//   [11] left_shoulder_pitch_joint
//   [12] right_shoulder_pitch_joint
//   [13] left_ankle_pitch_joint
//   [14] right_ankle_pitch_joint
//   [15] left_shoulder_roll_joint
//   [16] right_shoulder_roll_joint
//   [17] left_ankle_roll_joint
//   [18] right_ankle_roll_joint
//   [19] left_shoulder_yaw_joint
//   [20] right_shoulder_yaw_joint
//   [21] left_elbow_joint
//   [22] right_elbow_joint

// kYamlToNn[yaml_idx] = nn_idx
static constexpr int kYamlToNn[23] = {
//  L_hip_p  L_hip_r  L_hip_y  L_knee   L_ank_p  L_ank_r
  0, 3, 6, 9, 13, 17,
//  R_hip_p  R_hip_r  R_hip_y  R_knee   R_ank_p  R_ank_r
  1, 4, 7, 10, 14, 18,
//  waist_y  waist_r  waist_p
  2, 5, 8,
//  L_sho_p  L_sho_r  L_sho_y  L_elbow
  11, 15, 19, 21,
//  R_sho_p  R_sho_r  R_sho_y  R_elbow
  12, 16, 20, 22
};

// kNnToYaml[nn_idx] = yaml_idx
static constexpr int kNnToYaml[23] = {
//  L_hip_p  R_hip_p  waist_y  L_hip_r  R_hip_r  waist_r
  0, 6, 12, 1, 7, 13,
//  L_hip_y  R_hip_y  waist_p  L_knee   R_knee
  2, 8, 14, 3, 9,
//  L_sho_p  R_sho_p  L_ank_p  R_ank_p  L_sho_r  R_sho_r
  15, 19, 4, 10, 16, 20,
//  L_ank_r  R_ank_r  L_sho_y  R_sho_y  L_elbow  R_elbow
  5, 11, 17, 21, 18, 22
};

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
    if (default_pos_.size() < n || walk_kp_.size() < n || walk_kd_.size() < n) {
      RCLCPP_ERROR(logger(),
          "Parameter array size mismatch: expected %zu, got pos=%zu kp=%zu kd=%zu",
                   n, default_pos_.size(), walk_kp_.size(), walk_kd_.size());
      return controller_interface::CallbackReturn::ERROR;
    }

    // Action parameters
    node->declare_parameter<int>("action.action_size", 23);
    node->declare_parameter<int>("action.observation_size", 78);
    node->declare_parameter<int>("action.num_history", 10);
    node->declare_parameter<int>("action.policy_freq", 50);
    node->declare_parameter<std::vector<double>>("action.action_scale",
        std::vector<double>(23, 0.25));

    action_size_ = node->get_parameter("action.action_size").as_int();
    obs_size_ = node->get_parameter("action.observation_size").as_int();
    num_history_ = node->get_parameter("action.num_history").as_int();
    int policy_freq = node->get_parameter("action.policy_freq").as_int();
    action_scale_ = node->get_parameter("action.action_scale").as_double_array();

    inference_period_ = 1.0 / policy_freq;

    // Validate observation size: 9 (ang_vel + gravity + cmd) + 2*n (pos + vel) + action_size
    int expected_obs = 9 + 2 * static_cast<int>(n) + action_size_;
    if (obs_size_ != expected_obs) {
      RCLCPP_ERROR(logger(), "observation_size mismatch: expected %d (9 + 2*%zu + %d), got %d",
                   expected_obs, n, action_size_, obs_size_);
      return controller_interface::CallbackReturn::ERROR;
    }
    if (static_cast<int>(action_scale_.size()) < action_size_) {
      RCLCPP_ERROR(logger(), "action_scale size %zu < action_size %d",
                   action_scale_.size(), action_size_);
      return controller_interface::CallbackReturn::ERROR;
    }

    // Normalization
    node->declare_parameter<double>("normalization.clip_scales.clip_observations", 100.0);
    node->declare_parameter<double>("normalization.clip_scales.clip_actions", 100.0);
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
      } catch (const Ort::Exception & e) {
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

    // 日志目录：留空则使用 /tmp/walk_log；每次 activate 新建一个 walk_log_N.csv
    node->declare_parameter<std::string>("log_dir", "");
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
      obs_[9 + kYamlToNn[i]] = static_cast<float>(pos - default_pos_[i]);
      obs_[32 + kYamlToNn[i]] = static_cast<float>(vel);
    }
    // No previous action at activation
    for (int i = 0; i < action_size_; ++i) {
      obs_[55 + i] = 0.0f;
    }

    // Clip
    for (auto & v : obs_) {
      v = std::clamp(v, static_cast<float>(-clip_obs_), static_cast<float>(clip_obs_));
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
    if (log_file_.is_open()) {
      log_file_.close();
      RCLCPP_INFO(logger(), "Walk log file closed");
    }
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
  void onUpdate(const rclcpp::Time & /*time*/, const rclcpp::Duration & period) override
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
      inference_accumulator_ -= inference_period_;

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
      // [9:32]  joint_pos - default_pos, remapped to NN order
      for (size_t i = 0; i < n; ++i) {
        obs_[9 + kYamlToNn[i]] = static_cast<float>(jointState(i).position - default_pos_[i]);
      }
      // [32:55] joint_vel, remapped to NN order
      for (size_t i = 0; i < n; ++i) {
        obs_[32 + kYamlToNn[i]] = static_cast<float>(jointState(i).velocity);
      }
      // [55:78] prev_action
      for (int i = 0; i < action_size_; ++i) {
        obs_[55 + i] = actions_[i];
      }

      // Clip observations
      for (auto & v : obs_) {
        v = std::clamp(v, static_cast<float>(-clip_obs_), static_cast<float>(clip_obs_));
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
          actions_[i] = std::clamp(raw_actions[i],
            static_cast<float>(-clip_actions_), static_cast<float>(clip_actions_));
        }

        infer_count_++;
      } catch (const Ort::Exception & e) {
        RCLCPP_ERROR_THROTTLE(logger(), *get_node()->get_clock(), 1000,
          "ONNX inference failed: %s", e.what());
      }
    }

    // 1Hz 日志打印
    // {
    //   static auto last_print = std::chrono::steady_clock::now();
    //   auto now = std::chrono::steady_clock::now();
    //   if (now - last_print >= std::chrono::seconds(1)) {
    //     last_print = now;
    //     RCLCPP_INFO(logger(), "imu_gyro: [%.4f, %.4f, %.4f]",
    //                 ang_vel[0], ang_vel[1], ang_vel[2]);
    //     RCLCPP_INFO(logger(), "proj_gravity: [%.4f, %.4f, %.4f]",
    //                 proj_gravity[0], proj_gravity[1], proj_gravity[2]);
    //   }
    // }

    // Apply actions: position = default_pos + action_scale * raw_action
    // Actions are in NN order, remap to YAML order
    for (size_t i = 0; i < n; ++i) {
      auto & cmd = jointCommand(i);
      int nn_idx = kYamlToNn[i];
      float scaled_action = (nn_idx < action_size_) ?
        action_scale_[i] * actions_[nn_idx] :
        0.0f;
      cmd.position = default_pos_[i] + scaled_action;
      cmd.velocity = 0.0;
      cmd.effort = 0.0;
      cmd.kp = walk_kp_[i];
      cmd.kd = walk_kd_[i];
      // Coupled joints (DoubleCoupleJoint) use HostKpMotorKd mode
      cmd.torque_mode = kCoupledJoint[i] ? 2.0 : 0.0;
    }

    // 每个控制周期写入一行 CSV（controller_manager.update_rate = 1000 Hz）。
    // 仅记录当前周期的电机反馈和位置指令（YAML 关节顺序）。
    if (log_file_.is_open()) {
      writeLogRow();
    }

    // // 每次推理打印 action 输入输出
    // if (ort_session_) {
    //   // raw action (YAML顺序, 与 joint_cmd 一一对应)
    //   std::string raw_str;
    //   for (size_t i = 0; i < n; ++i) {
    //     if (i > 0) raw_str += ", ";
    //     raw_str += std::to_string(actions_[kYamlToNn[i]]);
    //   }
    //   // scaled action (YAML顺序, 与 joint_cmd 一一对应)
    //   std::string scaled_str;
    //   for (size_t i = 0; i < n; ++i) {
    //     if (i > 0) scaled_str += ", ";
    //     int nn_idx = kYamlToNn[i];
    //     scaled_str += std::to_string(action_scale_[i] * actions_[nn_idx]);
    //   }
    //   // 最终关节命令 (YAML顺序)
    //   std::string cmd_str;
    //   for (size_t i = 0; i < n; ++i) {
    //     if (i > 0) cmd_str += ", ";
    //     cmd_str += std::to_string(jointCommand(i).position);
    //   }
    //   RCLCPP_INFO(logger(),
    //     "raw_action:         [%s]\n"
    //     "action_scaled:      [%s]\n"
    //     "joint_cmd:          [%s]",
    //     raw_str.c_str(), scaled_str.c_str(), cmd_str.c_str());
    // }
  }

private:
  // 每次 activate 新建一个 CSV：时间戳 + 电机反馈/位置指令
  void openLogFile()
  {
    namespace fs = std::filesystem;
    // log_dir 为空时落到 /tmp/walk_log（不依赖 HOME，sudo 下也一致）
    fs::path dir = log_dir_.empty() ? fs::path("/tmp/walk_log") : fs::path(log_dir_);

    std::error_code ec;
    fs::create_directories(dir, ec);
    if (ec) {
      RCLCPP_WARN(logger(), "Failed to create log dir %s: %s", dir.c_str(), ec.message().c_str());
    }

    int run_id = 1;
    while (fs::exists(dir / ("walk_log_" + std::to_string(run_id) + ".csv"))) {
      ++run_id;
    }
    log_path_ = (dir / ("walk_log_" + std::to_string(run_id) + ".csv")).string();
    log_file_.open(log_path_, std::ios::out | std::ios::trunc);
    if (!log_file_.is_open()) {
      RCLCPP_WARN(logger(), "Failed to open walk log file: %s", log_path_.c_str());
      return;
    }

    // 列布局见 writeLogRow：时间戳 + 电机反馈/位置指令（YAML 顺序）
    log_file_ << std::setprecision(8);
    log_file_ << "time_ns,";
    const size_t n = jointCount();
    for (size_t i = 0; i < n; ++i) {log_file_ << "mpos" << i << ",";}
    for (size_t i = 0; i < n; ++i) {log_file_ << "mvel" << i << ",";}
    for (size_t i = 0; i < n; ++i) {log_file_ << "meff" << i << ",";}
    for (size_t i = 0; i < n; ++i) {log_file_ << "cpos" << i << (i + 1 < n ? "," : "\n");}

    RCLCPP_INFO(logger(), "Walk log file: %s", log_path_.c_str());
  }

  // 每个控制周期写一行电机反馈和位置指令。
  void writeLogRow()
  {
    const size_t n = jointCount();

    log_file_ << get_node()->now().nanoseconds();
    // 电机反馈 + 位置指令（YAML 顺序）
    for (size_t i = 0; i < n; ++i) {log_file_ << "," << jointState(i).position;}
    for (size_t i = 0; i < n; ++i) {log_file_ << "," << jointState(i).velocity;}
    for (size_t i = 0; i < n; ++i) {log_file_ << "," << jointState(i).effort;}
    for (size_t i = 0; i < n; ++i) {log_file_ << "," << jointCommand(i).position;}
    log_file_ << "\n";
    // 使用流缓冲，避免 1000 Hz 下逐行强制刷新；关闭文件时刷新剩余数据。
  }

  // Joint parameters
  std::vector<double> default_pos_;
  std::vector<double> walk_kp_;
  std::vector<double> walk_kd_;

  // Action parameters
  int action_size_{23};
  int obs_size_{78};
  int num_history_{10};
  double inference_period_{0.02};
  double inference_accumulator_{0.0};
  std::vector<double> action_scale_;
  double clip_obs_{100.0};
  double clip_actions_{100.0};

  // Inference buffers
  std::vector<float> actions_;
  std::vector<float> obs_;
  std::vector<float> obs_history_;
  int infer_count_{0};
  bool history_filled_{false};

  // CSV 日志文件
  std::ofstream log_file_;
  std::string log_path_;
  std::string log_dir_;

  // ONNX Runtime
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

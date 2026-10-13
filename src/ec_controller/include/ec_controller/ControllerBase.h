// =============================================================================
//  Copyright (c) 2026 Phanerox. All rights reserved.
// =============================================================================
#ifndef EC_CONTROLLER_CONTROLLER_BASE_H_
#define EC_CONTROLLER_CONTROLLER_BASE_H_

#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ec
{

// Coupled joints (DoubleCoupleJoint) in YAML order — shared by all controllers
inline constexpr bool kCoupledJoint[23] = {
//  L_hip_p  L_hip_r  L_hip_y  L_knee   L_ank_p  L_ank_r
  false, false, false, false, true, true,
//  R_hip_p  R_hip_r  R_hip_y  R_knee   R_ank_p  R_ank_r
  false, false, false, false, true, true,
//  waist_y  waist_r  waist_p
  false, true, true,
//  L_sho_p  L_sho_r  L_sho_y  L_elbow
  false, false, false, false,
//  R_sho_p  R_sho_r  R_sho_y  R_elbow
  false, false, false, false
};

// Lightweight joint data structure
struct JointState
{
  double position {0.0};
  double velocity {0.0};
  double effort {0.0};
};

struct JointCommand
{
  double position {0.0};
  double velocity {0.0};
  double effort {0.0};
  double kp {0.0};
  double kd {0.0};
  double torque_mode {0.0};
};

/**
 * @brief Optimized base class for controllers
 */
class ControllerBase : public controller_interface::ControllerInterface {
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

protected:
  virtual void onUpdate(const rclcpp::Time & time, const rclcpp::Duration & period) = 0;

  // Direct access to joint states
  const JointState & jointState(size_t index) const {return joint_states_[index];}
  size_t jointCount() const {return joint_names_.size();}

  // Direct access to joint commands
  JointCommand & jointCommand(size_t index) {return joint_commands_[index];}

  inline rclcpp::Logger logger() const {return get_node()->get_logger();}

  std::vector<std::string> joint_names_;

private:
  void buildIndexMap();

  std::vector<JointState> joint_states_;
  std::vector<JointCommand> joint_commands_;

  // Cached interface indices (filled in on_activate)
  std::vector<size_t> state_pos_idx_;
  std::vector<size_t> state_vel_idx_;
  std::vector<size_t> state_eff_idx_;
  std::vector<size_t> cmd_pos_idx_;
  std::vector<size_t> cmd_vel_idx_;
  std::vector<size_t> cmd_eff_idx_;
  std::vector<size_t> cmd_kp_idx_;
  std::vector<size_t> cmd_kd_idx_;
  std::vector<size_t> cmd_torque_mode_idx_;
};

} // namespace ec

#endif  // EC_CONTROLLER_CONTROLLER_BASE_H_

// =============================================================================
//  Copyright (c) 2026 Phanerox. All rights reserved.
// =============================================================================

#include <pluginlib/class_list_macros.hpp>
#include "ec_controller/ControllerBase.h"

namespace ec
{

class StandController : public ControllerBase {
public:
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override
  {
    auto ret = ControllerBase::on_configure(previous_state);
    if (ret != controller_interface::CallbackReturn::SUCCESS) {return ret;}

    auto node = get_node();
    node->declare_parameter<std::vector<double>>("stand_pos", std::vector<double>{});
    node->declare_parameter<std::vector<double>>("stand_kp", std::vector<double>{});
    node->declare_parameter<std::vector<double>>("stand_kd", std::vector<double>{});

    stand_pos_ = node->get_parameter("stand_pos").as_double_array();
    stand_kp_ = node->get_parameter("stand_kp").as_double_array();
    stand_kd_ = node->get_parameter("stand_kd").as_double_array();

    const size_t n = jointCount();
    if (stand_pos_.size() < n || stand_kp_.size() < n || stand_kd_.size() < n) {
      RCLCPP_ERROR(logger(),
          "Parameter array size mismatch: expected %zu, got pos=%zu kp=%zu kd=%zu",
                   n, stand_pos_.size(), stand_kp_.size(), stand_kd_.size());
      return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

protected:
  void onUpdate(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    const size_t n = jointCount();
    for (size_t i = 0; i < n; ++i) {
      auto & cmd = jointCommand(i);
      cmd.position = stand_pos_[i];
      cmd.velocity = 0.0;
      cmd.effort = 0.0;
      cmd.kp = stand_kp_[i];
      cmd.kd = stand_kd_[i];
      cmd.torque_mode = kCoupledJoint[i] ? 2.0 : 0.0;
    }
  }

private:
  std::vector<double> stand_pos_;
  std::vector<double> stand_kp_;
  std::vector<double> stand_kd_;
};

} // namespace ec

PLUGINLIB_EXPORT_CLASS(ec::StandController, controller_interface::ControllerInterface)

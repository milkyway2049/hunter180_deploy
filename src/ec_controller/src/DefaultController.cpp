// =============================================================================
//  Copyright (c) 2026 Phanerox. All rights reserved.
// =============================================================================

#include <pluginlib/class_list_macros.hpp>
#include "ec_controller/ControllerBase.h"

namespace ec
{

class DefaultController : public ControllerBase {
public:
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override
  {
    auto ret = ControllerBase::on_configure(previous_state);
    if (ret != controller_interface::CallbackReturn::SUCCESS) {return ret;}

    auto node = get_node();
    node->declare_parameter<std::vector<double>>("default_pos", std::vector<double>{});
    node->declare_parameter<std::vector<double>>("default_kp", std::vector<double>{});
    node->declare_parameter<std::vector<double>>("default_kd", std::vector<double>{});

    default_pos_ = node->get_parameter("default_pos").as_double_array();
    default_kp_ = node->get_parameter("default_kp").as_double_array();
    default_kd_ = node->get_parameter("default_kd").as_double_array();

    const size_t n = jointCount();
    if (default_pos_.size() < n || default_kp_.size() < n || default_kd_.size() < n) {
      RCLCPP_ERROR(logger(),
          "Parameter array size mismatch: expected %zu, got pos=%zu kp=%zu kd=%zu",
                   n, default_pos_.size(), default_kp_.size(), default_kd_.size());
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
      cmd.position = default_pos_[i];
      cmd.velocity = 0.0;
      cmd.effort = 0.0;
      cmd.kp = default_kp_[i];
      cmd.kd = default_kd_[i];
      cmd.torque_mode = kCoupledJoint[i] ? 2.0 : 0.0;
    }
  }

private:
  std::vector<double> default_pos_;
  std::vector<double> default_kp_;
  std::vector<double> default_kd_;
};

} // namespace ec

PLUGINLIB_EXPORT_CLASS(ec::DefaultController, controller_interface::ControllerInterface)

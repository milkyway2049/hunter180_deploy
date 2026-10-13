// =============================================================================
//  Copyright (c) 2026 Phanerox. All rights reserved.
// =============================================================================

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <ec_radio/msg/radio.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>

#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace ec
{

/**
 * @brief Controller Switcher Node
 *
 * 监听遥控器输出，自动切换控制器
 * 转换顺序: 无力(default) -> 站立(stand) -> 强化学习(walk) -> 无力(default)
 */
class ControllerSwitcherNode : public rclcpp::Node {
public:
  ControllerSwitcherNode()
  : Node("controller_switcher")
  {
    // 声明参数
    this->declare_parameter<std::string>("controller_manager_ns", "/controller_manager");
    this->declare_parameter<std::string>("initial_controller", "default_controller");

    // 获取参数
    controller_manager_ns_ = this->get_parameter("controller_manager_ns").as_string();
    initial_controller_ = this->get_parameter("initial_controller").as_string();

    // 创建服务客户端
    switch_controller_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>(
      controller_manager_ns_ + "/switch_controller");

    // 订阅遥控器话题
    radio_sub_ = this->create_subscription<ec_radio::msg::Radio>(
      "/radio", 10,
      std::bind(&ControllerSwitcherNode::radioCallback, this, std::placeholders::_1));

    // 订阅控制器切换话题（手动切换）
    default_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/controller_switch/default_controller", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        if (msg->data > 0.5f) {switchController("default_controller");}
      });

    stand_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/controller_switch/stand_controller", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        if (msg->data > 0.5f) {switchController("stand_controller");}
      });

    walk_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/controller_switch/walk_controller", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        if (msg->data > 0.5f) {switchController("walk_controller");}
      });

    // 等待服务可用
    RCLCPP_INFO(this->get_logger(), "Waiting for switch_controller service...");
    switch_controller_client_->wait_for_service(30s);

    // 使用定时器延迟激活初始控制器
    init_timer_ = this->create_wall_timer(2s, [this]() {
          init_timer_->cancel();
          switchController(initial_controller_);
    });

    RCLCPP_INFO(this->get_logger(), "Controller Switcher Node started");
    RCLCPP_INFO(this->get_logger(), "Transition: default -> stand -> walk -> default");
  }

private:
  void radioCallback(const ec_radio::msg::Radio::SharedPtr msg)
  {
    if (msg->mode != last_mode_) {
      last_mode_ = msg->mode;

      switch (msg->mode) {
        case 1:
          switchController("default_controller");
          break;
        case 2:
          switchController("stand_controller");
          break;
        case 3:
          switchController("walk_controller");
          break;
        default:
          switchController("default_controller");
          break;
      }
    }
  }

  bool isTransitionAllowed(const std::string & from, const std::string & to)
  {
    if (from == "default_controller" && to == "stand_controller") {return true;}
    if (from == "stand_controller" && to == "walk_controller") {return true;}
    if (from == "stand_controller" && to == "default_controller") {return true;}
    if (from == "walk_controller" && to == "default_controller") {return true;}
    return false;
  }

  void switchController(const std::string & controller_name)
  {
    if (active_controller_ == controller_name) {return;}

    // 如果还没有记录活跃控制器，直接设置（可能是spawner已经激活了）
    if (active_controller_.empty()) {
      active_controller_ = controller_name;
      RCLCPP_INFO(this->get_logger(), "Assuming '%s' is already active.", controller_name.c_str());
      return;
    }

    // 检查转换是否合法
    if (!isTransitionAllowed(active_controller_, controller_name)) {
      RCLCPP_WARN(this->get_logger(), "Transition not allowed: %s -> %s",
        active_controller_.c_str(), controller_name.c_str());
      return;
    }

    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();

    // 停用当前控制器
    request->deactivate_controllers.push_back(active_controller_);

    // 激活新控制器
    request->activate_controllers.push_back(controller_name);
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
    request->timeout = rclcpp::Duration::from_seconds(5.0);

    RCLCPP_INFO(this->get_logger(), "Switching to: %s", controller_name.c_str());

    auto future = switch_controller_client_->async_send_request(request,
        [this,
        controller_name](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::
        SharedFuture result) {
          try {
            if (result.get()->ok) {
              active_controller_ = controller_name;
              RCLCPP_INFO(this->get_logger(), "Switched to: %s", controller_name.c_str());
            } else {
              RCLCPP_ERROR(this->get_logger(), "Failed to switch to: %s", controller_name.c_str());
            }
          } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
          }
      });
  }

  std::string controller_manager_ns_;
  std::string initial_controller_;

  std::string active_controller_;
  uint8_t last_mode_{0};

  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr
    switch_controller_client_;

  rclcpp::Subscription<ec_radio::msg::Radio>::SharedPtr radio_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr default_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr stand_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr walk_sub_;

  rclcpp::TimerBase::SharedPtr init_timer_;
};

} // namespace ec

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ec::ControllerSwitcherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

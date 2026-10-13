#include <rclcpp/rclcpp.hpp>
#include <serial_driver/serial_driver.hpp>
#include <ec_radio/msg/radio.hpp>

using namespace std::chrono_literals;

class RadioBridgeNode : public rclcpp::Node {
public:
  RadioBridgeNode()
  : Node("radio_node"),
    owned_ctx_{new IoContext(2)},
    serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
  {
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baud_rate", 115200);

    pub_ = this->create_publisher<ec_radio::msg::Radio>("radio", 10);

    reconnect_timer_ = this->create_wall_timer(1s, [this]() {
          if (!serial_driver_->port()->is_open()) {
            try {
              serial_driver_->port()->open();
              RCLCPP_INFO(this->get_logger(), "Reconnected");
              start_receive();
            } catch (const std::exception & e) {
              RCLCPP_WARN(this->get_logger(), "Reconnect failed: %s", e.what());
            }
          }
    });

    open_port();
  }

private:
  void open_port()
  {
    std::string port = this->get_parameter("serial_port").as_string();
    int baud = this->get_parameter("baud_rate").as_int();

    drivers::serial_driver::SerialPortConfig config(baud,
      drivers::serial_driver::FlowControl::NONE,
      drivers::serial_driver::Parity::NONE,
      drivers::serial_driver::StopBits::ONE);

    try {
      serial_driver_->init_port(port, config);
      serial_driver_->port()->open();
      RCLCPP_INFO(this->get_logger(), "Opened %s @ %d baud", port.c_str(), baud);
      start_receive();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Open failed: %s", e.what());
    }
  }

  void start_receive()
  {
    try {
      serial_driver_->port()->async_receive(
        std::bind(&RadioBridgeNode::on_receive, this, std::placeholders::_1,
        std::placeholders::_2));
    } catch (...) {
      try {serial_driver_->port()->close();} catch (...) {}
    }
  }

  void on_receive(const std::vector<uint8_t> & data, const size_t & len)
  {
    rx_buf_.insert(rx_buf_.end(), data.begin(), data.begin() + len);

    while (rx_buf_.size() >= 32) {
      if (rx_buf_[0] != 0x20 || rx_buf_[1] != 0x40) {
        rx_buf_.erase(rx_buf_.begin());
        continue;
      }

      std::vector<uint8_t> f(rx_buf_.begin(), rx_buf_.begin() + 32);
      rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + 32);

      uint16_t ch[18];
      for (int i = 0; i < 14; ++i) {
        ch[i] = ((f[3 + 2 * i] & 0x0F) << 8) | f[2 + 2 * i];
      }
      for (int i = 0; i < 4; ++i) {
        ch[14 + i] = ((f[7 + 6 * i] & 0xF0) <<
          4) | (f[5 + 6 * i] & 0xF0) | ((f[3 + 6 * i] & 0xF0) >> 4);
      }

      for (auto & v : ch) {v = std::clamp(v, (uint16_t)1000, (uint16_t)2000);}

      // ±10% 死区（信号范围1000-2000，10% = ±50，即1450-1550归零）
      auto dz = [](uint16_t v) {return (v >= 1450 && v <= 1550) ? 1500 : v;};
      ch[0] = dz(ch[0]);
      ch[2] = dz(ch[2]);
      ch[3] = dz(ch[3]);

      auto msg = ec_radio::msg::Radio();
      msg.vx = (ch[2] - 1500) / 500.0f;
      msg.vy = -(ch[3] - 1500) / 500.0f;
      msg.wz = -(ch[0] - 1500) / 500.0f;

      // CH4-CH7 按键模式切换：按下>1500，未按下<1500，同时只有一个按键
      msg.mode = 1;
      for (int i = 4; i <= 7; ++i) {
        if (ch[i] > 1500) {
          msg.mode = i - 2;
          break;
        }
      }

      pub_->publish(msg);
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "vx=%.2f vy=%.2f wz=%.2f mode=%d",
                           msg.vx, msg.vy, msg.wz, msg.mode);
    }

    start_receive();
  }

  std::unique_ptr<IoContext> owned_ctx_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
  rclcpp::Publisher<ec_radio::msg::Radio>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;
  std::vector<uint8_t> rx_buf_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RadioBridgeNode>());
  rclcpp::shutdown();
  return 0;
}

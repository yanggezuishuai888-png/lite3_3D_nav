#include "lite3_sensor_gateway/codecs.hpp"
#include "lite3_sensor_gateway/command.hpp"
#include "lite3_sensor_gateway/net.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
using namespace lite3_gateway;

class CommandSender : public rclcpp::Node {
  Socket socket_;
  sockaddr_in destination_;
  uint64_t session_, sequence_ = 0;
  bool time_valid_ = false, active_ = false;
  double last_status_ = 0;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_;

  void send(const geometry_msgs::msg::Twist &command) {
    Envelope e;
    e.type = Type::CmdVel;
    e.clock = ClockDomain::System;
    e.session = session_;
    e.sequence = ++sequence_;
    e.source_ns = e.sent_ns = system_ns();
    if (!socket_.send(fragment(e, encode(command)).front(), destination_))
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "command UDP send failed: %s",
                            strerror(errno));
  }

public:
  CommandSender()
      : Node("lite3_command_sender"),
        socket_(declare_parameter<std::string>("bind_ip", "10.42.0.1"),
                declare_parameter<int>("command_port", 45005)),
        destination_(address(declare_parameter<std::string>("dog_ip", "10.42.0.2"),
                             get_parameter("command_port").as_int())) {
    std::random_device random;
    session_ = (uint64_t(random()) << 32) ^ random() ^ system_ns();
    if (!session_)
      session_ = 1;
    const auto input = declare_parameter<std::string>("input_topic", "/cmd_vel");
    const auto status = declare_parameter<std::string>("time_valid_topic", "/lite3/gateway/time_valid");
    status_sub_ = create_subscription<std_msgs::msg::Bool>(
        status, rclcpp::QoS(1).reliable(),
        [this](std_msgs::msg::Bool::ConstSharedPtr m) {
          time_valid_ = m->data;
          last_status_ = steady_seconds();
        });
    command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        input, rclcpp::SensorDataQoS().keep_last(1),
        [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
          if (!valid_command(*m)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "invalid or out-of-range cmd_vel");
            return;
          }
          if (!time_valid_ || steady_seconds() - last_status_ > 1.5) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "cmd_vel blocked: gateway time status unavailable");
            return;
          }
          send(*m);
          active_ = true;
        });
    watchdog_ = create_wall_timer(std::chrono::milliseconds(50), [this] {
      if (active_ && (!time_valid_ || steady_seconds() - last_status_ > 1.5)) {
        send(geometry_msgs::msg::Twist{});
        active_ = false;
      }
    });
    RCLCPP_INFO(get_logger(), "PC %s -> dog UDP %s:%d; input %s; clock gate %s",
                get_parameter("bind_ip").as_string().c_str(),
                get_parameter("dog_ip").as_string().c_str(),
                static_cast<int>(get_parameter("command_port").as_int()), input.c_str(),
                status.c_str());
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<CommandSender>());
  } catch (const std::exception &e) {
    fprintf(stderr, "command_sender: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

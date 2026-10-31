#include "lite3_sensor_gateway/codecs.hpp"
#include "lite3_sensor_gateway/command.hpp"
#include "lite3_sensor_gateway/net.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <set>
using namespace lite3_gateway;

class CommandReceiver : public rclcpp::Node {
  Socket socket_;
  sockaddr_in peer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr poll_, watchdog_;
  uint64_t session_ = 0, sequence_ = 0;
  std::set<uint64_t> retired_;
  double last_command_ = 0, stop_until_ = 0;
  bool active_ = false;

  void receive() {
    for (int i = 0; i < 32; ++i) {
      uint8_t buffer[kDatagramSize + 1];
      sockaddr_in from{};
      socklen_t len = sizeof(from);
      auto n = recvfrom(socket_.fd(), buffer, sizeof(buffer), 0,
                        reinterpret_cast<sockaddr *>(&from), &len);
      if (n < 0)
        break;
      if (from.sin_addr.s_addr != peer_.sin_addr.s_addr ||
          from.sin_port != peer_.sin_port)
        continue;
      try {
        auto f = parse_fragment(Bytes(buffer, buffer + n));
        if (f.env.type != Type::CmdVel || f.env.clock != ClockDomain::System ||
            f.env.flags || f.count != 1 || f.total != f.data.size() ||
            checksum(f.data) != f.crc || !fresh_command(f.env.source_ns, system_ns()) ||
            retired_.count(f.env.session))
          continue;
        if (f.env.session != session_) {
          if (session_)
            retired_.insert(session_);
          session_ = f.env.session;
          sequence_ = 0;
        }
        if (f.env.sequence <= sequence_)
          continue;
        auto command = decode_twist(f.data);
        if (!valid_command(command))
          continue;
        sequence_ = f.env.sequence;
        publisher_->publish(command);
        active_ = true;
        last_command_ = steady_seconds();
        stop_until_ = 0;
      } catch (const std::exception &) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "invalid command datagram");
      }
    }
  }

public:
  CommandReceiver()
      : Node("lite3_command_receiver"),
        socket_(declare_parameter<std::string>("bind_ip", "10.42.0.2"),
                declare_parameter<int>("command_port", 45005)),
        peer_(address(declare_parameter<std::string>("pc_ip", "10.42.0.1"),
                      get_parameter("command_port").as_int())) {
    const auto output = declare_parameter<std::string>("output_topic", "/cmd_vel");
    publisher_ = create_publisher<geometry_msgs::msg::Twist>(
        output, rclcpp::SensorDataQoS().keep_last(1));
    poll_ = create_wall_timer(std::chrono::milliseconds(5), [this] { receive(); });
    watchdog_ = create_wall_timer(std::chrono::milliseconds(50), [this] {
      const auto now = steady_seconds();
      if (active_ && now - last_command_ > 0.25) {
        active_ = false;
        stop_until_ = now + 1.0;
        RCLCPP_WARN(get_logger(), "cmd_vel timed out; publishing zero velocity");
      }
      if (stop_until_ > now)
        publisher_->publish(geometry_msgs::msg::Twist{});
    });
    RCLCPP_INFO(get_logger(), "dog command UDP %s:%d <- PC %s; output %s",
                get_parameter("bind_ip").as_string().c_str(),
                static_cast<int>(get_parameter("command_port").as_int()),
                get_parameter("pc_ip").as_string().c_str(), output.c_str());
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<CommandReceiver>());
  } catch (const std::exception &e) {
    fprintf(stderr, "command_receiver: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>

namespace
{
double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

diagnostic_msgs::msg::KeyValue key_value(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

bool finite_pose(const geometry_msgs::msg::Pose & pose)
{
  const auto & p = pose.position;
  const auto & q = pose.orientation;
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
         std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
         std::isfinite(q.w);
}
}  // namespace

class LegOdomAdapter : public rclcpp::Node
{
public:
  LegOdomAdapter()
  : Node("leg_odom_adapter"), tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  {
    input_topic_ = declare_parameter<std::string>("input_odom_topic", "/lite3/leg_odom2");
    time_valid_topic_ =
      declare_parameter<std::string>("time_valid_topic", "/lite3/gateway/time_valid");
    output_topic_ =
      declare_parameter<std::string>("output_odom_topic", "/lite3/leg_odom_validated");
    expected_input_frame_ = declare_parameter<std::string>("expected_input_frame", "odom");
    output_frame_ = declare_parameter<std::string>("output_frame", "leg_odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    require_time_valid_ = declare_parameter<bool>("require_time_valid", true);
    planarize_ = declare_parameter<bool>("planarize", true);
    max_past_age_ = declare_parameter<double>("max_past_age_seconds", 0.25);
    max_future_age_ = declare_parameter<double>("max_future_age_seconds", 0.05);
    max_position_step_ = declare_parameter<double>("max_position_step_m", 0.50);
    max_yaw_step_ = declare_parameter<double>("max_yaw_step_rad", 1.00);
    max_linear_speed_ = declare_parameter<double>("max_linear_speed_mps", 3.00);
    max_angular_speed_ = declare_parameter<double>("max_angular_speed_radps", 5.00);
    const double diagnostic_period =
      declare_parameter<double>("diagnostic_period_seconds", 1.0);
    recent_sample_seconds_ = declare_parameter<double>("recent_sample_seconds", 0.50);

    if (output_frame_.empty() || base_frame_.empty() || output_frame_ == base_frame_) {
      throw std::runtime_error("output_frame and base_frame must be distinct, non-empty frames");
    }
    if (max_past_age_ <= 0.0 || max_future_age_ < 0.0 || max_position_step_ <= 0.0 ||
      max_yaw_step_ <= 0.0 || max_linear_speed_ <= 0.0 || max_angular_speed_ <= 0.0 ||
      diagnostic_period <= 0.0 || recent_sample_seconds_ <= 0.0)
    {
      throw std::runtime_error("adapter timing and motion limits must be positive");
    }

    validated_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      output_topic_, rclcpp::SensorDataQoS().keep_last(5));
    diagnostic_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic_, rclcpp::SensorDataQoS().keep_last(5),
      std::bind(&LegOdomAdapter::on_odom, this, std::placeholders::_1));
    time_valid_sub_ = create_subscription<std_msgs::msg::Bool>(
      time_valid_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&LegOdomAdapter::on_time_valid, this, std::placeholders::_1));
    clear_fault_service_ = create_service<std_srvs::srv::Trigger>(
      "~/clear_fault",
      std::bind(
        &LegOdomAdapter::clear_fault, this, std::placeholders::_1, std::placeholders::_2));

    diagnostic_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(diagnostic_period)),
      std::bind(&LegOdomAdapter::publish_diagnostics, this));

    RCLCPP_INFO(
      get_logger(),
      "Leg odometry adapter: %s -> TF %s -> %s; time gate=%s, planar=%s",
      input_topic_.c_str(), output_frame_.c_str(), base_frame_.c_str(),
      require_time_valid_ ? "required" : "disabled", planarize_ ? "true" : "false");
  }

private:
  using SteadyClock = std::chrono::steady_clock;

  void on_time_valid(const std_msgs::msg::Bool::ConstSharedPtr message)
  {
    time_valid_ = message->data;
    time_valid_seen_ = true;
  }

  void reject(const std::string & reason)
  {
    ++rejected_;
    last_reject_reason_ = reason;
    if (reason == "time_invalid") {
      ++rejected_time_invalid_;
    } else if (reason == "stale_or_future") {
      ++rejected_age_;
    } else if (reason == "non_monotonic") {
      ++rejected_non_monotonic_;
    } else if (reason == "frame_mismatch") {
      ++rejected_frame_;
    } else if (reason == "invalid_pose") {
      ++rejected_invalid_pose_;
    } else if (reason == "fault_latched" || reason == "motion_discontinuity") {
      ++rejected_discontinuity_;
    }
  }

  void on_odom(const nav_msgs::msg::Odometry::ConstSharedPtr message)
  {
    ++received_;
    if (fault_latched_) {
      reject("fault_latched");
      return;
    }
    if (require_time_valid_ && (!time_valid_seen_ || !time_valid_)) {
      reject("time_invalid");
      return;
    }
    if (message->header.frame_id != expected_input_frame_ ||
      message->child_frame_id != base_frame_)
    {
      reject("frame_mismatch");
      return;
    }
    if (!finite_pose(message->pose.pose)) {
      reject("invalid_pose");
      return;
    }

    const auto & orientation = message->pose.pose.orientation;
    const double quaternion_norm = std::sqrt(
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w);
    if (!std::isfinite(quaternion_norm) || quaternion_norm < 0.5) {
      reject("invalid_pose");
      return;
    }

    const rclcpp::Time source_stamp(message->header.stamp);
    if (source_stamp.nanoseconds() <= 0) {
      reject("stale_or_future");
      return;
    }
    const double age = (get_clock()->now() - source_stamp).seconds();
    last_source_age_ = age;
    if (age > max_past_age_ || age < -max_future_age_) {
      reject("stale_or_future");
      return;
    }
    if (have_previous_ && source_stamp <= previous_stamp_) {
      reject("non_monotonic");
      return;
    }

    nav_msgs::msg::Odometry output = *message;
    output.header.frame_id = output_frame_;
    output.child_frame_id = base_frame_;

    tf2::Quaternion normalized(
      orientation.x / quaternion_norm, orientation.y / quaternion_norm,
      orientation.z / quaternion_norm, orientation.w / quaternion_norm);
    const double yaw = tf2::getYaw(normalized);
    if (planarize_) {
      output.pose.pose.position.z = 0.0;
      tf2::Quaternion planar;
      planar.setRPY(0.0, 0.0, yaw);
      output.pose.pose.orientation = tf2::toMsg(planar);
      output.twist.twist.linear.z = 0.0;
      output.twist.twist.angular.x = 0.0;
      output.twist.twist.angular.y = 0.0;
    } else {
      output.pose.pose.orientation = tf2::toMsg(normalized);
    }

    if (have_previous_) {
      const double dt = (source_stamp - previous_stamp_).seconds();
      const double dx = output.pose.pose.position.x - previous_x_;
      const double dy = output.pose.pose.position.y - previous_y_;
      const double position_step = std::hypot(dx, dy);
      const double yaw_step = std::abs(normalize_angle(yaw - previous_yaw_));
      const double linear_speed = position_step / dt;
      const double angular_speed = yaw_step / dt;
      if (position_step > max_position_step_ || yaw_step > max_yaw_step_ ||
        linear_speed > max_linear_speed_ || angular_speed > max_angular_speed_)
      {
        fault_latched_ = true;
        last_reject_reason_ = "motion_discontinuity";
        reject("motion_discontinuity");
        RCLCPP_ERROR(
          get_logger(),
          "Leg odometry discontinuity latched: dt=%.6f s, step=%.3f m, yaw=%.3f rad, "
          "speed=%.3f m/s, yaw_rate=%.3f rad/s. Restart mapping before clearing the fault.",
          dt, position_step, yaw_step, linear_speed, angular_speed);
        return;
      }
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header = output.header;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = output.pose.pose.position.x;
    transform.transform.translation.y = output.pose.pose.position.y;
    transform.transform.translation.z = output.pose.pose.position.z;
    transform.transform.rotation = output.pose.pose.orientation;

    validated_pub_->publish(output);
    tf_broadcaster_->sendTransform(transform);

    previous_stamp_ = source_stamp;
    previous_x_ = output.pose.pose.position.x;
    previous_y_ = output.pose.pose.position.y;
    previous_yaw_ = yaw;
    have_previous_ = true;
    last_accept_steady_ = SteadyClock::now();
    ++accepted_;
  }

  void clear_fault(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    fault_latched_ = false;
    have_previous_ = false;
    last_reject_reason_ = "none";
    response->success = true;
    response->message =
      "Fault cleared. Restart RTAB-Map first if the source odometry origin changed.";
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = get_clock()->now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "/lite3/leg_odom_adapter";
    status.hardware_id = "lite3_leg_odom";

    const double seconds_since_accept = accepted_ == 0 ?
      std::numeric_limits<double>::infinity() :
      std::chrono::duration<double>(SteadyClock::now() - last_accept_steady_).count();
    if (fault_latched_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "Odometry discontinuity fault latched";
    } else if (require_time_valid_ && (!time_valid_seen_ || !time_valid_)) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "Waiting for valid cross-machine time";
    } else if (seconds_since_accept > recent_sample_seconds_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "No recent accepted leg odometry";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "Publishing validated leg odometry TF";
    }

    status.values.push_back(key_value("time_valid", time_valid_ ? "true" : "false"));
    status.values.push_back(key_value("fault_latched", fault_latched_ ? "true" : "false"));
    status.values.push_back(key_value("received", std::to_string(received_)));
    status.values.push_back(key_value("accepted", std::to_string(accepted_)));
    status.values.push_back(key_value("rejected", std::to_string(rejected_)));
    status.values.push_back(key_value("rejected_time_invalid", std::to_string(rejected_time_invalid_)));
    status.values.push_back(key_value("rejected_age", std::to_string(rejected_age_)));
    status.values.push_back(
      key_value("rejected_non_monotonic", std::to_string(rejected_non_monotonic_)));
    status.values.push_back(key_value("rejected_frame", std::to_string(rejected_frame_)));
    status.values.push_back(
      key_value("rejected_invalid_pose", std::to_string(rejected_invalid_pose_)));
    status.values.push_back(
      key_value("rejected_discontinuity", std::to_string(rejected_discontinuity_)));
    status.values.push_back(key_value("last_reject_reason", last_reject_reason_));
    status.values.push_back(key_value("last_source_age_seconds", std::to_string(last_source_age_)));
    status.values.push_back(key_value("output_frame", output_frame_));
    status.values.push_back(key_value("base_frame", base_frame_));
    array.status.push_back(status);
    diagnostic_pub_->publish(array);
  }

  std::string input_topic_;
  std::string time_valid_topic_;
  std::string output_topic_;
  std::string expected_input_frame_;
  std::string output_frame_;
  std::string base_frame_;
  bool require_time_valid_{true};
  bool planarize_{true};
  double max_past_age_{0.25};
  double max_future_age_{0.05};
  double max_position_step_{0.50};
  double max_yaw_step_{1.00};
  double max_linear_speed_{3.00};
  double max_angular_speed_{5.00};
  double recent_sample_seconds_{0.50};

  bool time_valid_{false};
  bool time_valid_seen_{false};
  bool fault_latched_{false};
  bool have_previous_{false};
  rclcpp::Time previous_stamp_{0, 0, RCL_ROS_TIME};
  double previous_x_{0.0};
  double previous_y_{0.0};
  double previous_yaw_{0.0};
  double last_source_age_{std::numeric_limits<double>::quiet_NaN()};
  SteadyClock::time_point last_accept_steady_{SteadyClock::now()};
  std::string last_reject_reason_{"none"};

  uint64_t received_{0};
  uint64_t accepted_{0};
  uint64_t rejected_{0};
  uint64_t rejected_time_invalid_{0};
  uint64_t rejected_age_{0};
  uint64_t rejected_non_monotonic_{0};
  uint64_t rejected_frame_{0};
  uint64_t rejected_invalid_pose_{0};
  uint64_t rejected_discontinuity_{0};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr time_valid_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr validated_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_fault_service_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<LegOdomAdapter>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("leg_odom_adapter"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

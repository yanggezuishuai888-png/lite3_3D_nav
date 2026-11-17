#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

namespace
{
bool finite_value(double value)
{
  return std::isfinite(value);
}

bool finite_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  return finite_value(q.x) && finite_value(q.y) && finite_value(q.z) && finite_value(q.w);
}
}  // namespace

class EkfInputAdapter : public rclcpp::Node
{
public:
  EkfInputAdapter()
  : Node("ekf_input_adapter")
  {
    leg_odom_input_topic_ = declare_parameter<std::string>(
      "leg_odom_input_topic", "/lite3/leg_odom2");
    imu_input_topic_ = declare_parameter<std::string>("imu_input_topic", "/lite3/imu/data");
    leg_odom_output_topic_ = declare_parameter<std::string>(
      "leg_odom_output_topic", "/ekf/leg_odom");
    imu_output_topic_ = declare_parameter<std::string>("imu_output_topic", "/ekf/imu");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "ekf_odom");
    input_odom_frame_ = declare_parameter<std::string>("input_odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    max_past_age_seconds_ = declare_parameter<double>("max_past_age_seconds", 0.25);
    max_future_age_seconds_ = declare_parameter<double>("max_future_age_seconds", 0.05);
    leg_position_variance_ = declare_parameter<double>("leg_position_variance", 0.01);
    leg_z_constraint_variance_ = declare_parameter<double>(
      "leg_z_constraint_variance", 0.01);
    imu_orientation_variance_ = declare_parameter<double>(
      "imu_orientation_variance", 0.0025);
    imu_angular_velocity_variance_ = declare_parameter<double>(
      "imu_angular_velocity_variance", 0.0004);

    if (odom_frame_.empty() || base_frame_.empty() || odom_frame_ == base_frame_) {
      throw std::runtime_error("odom_frame and base_frame must be distinct, non-empty frames");
    }
    if (max_past_age_seconds_ <= 0.0 || max_future_age_seconds_ < 0.0 ||
      leg_position_variance_ <= 0.0 || leg_z_constraint_variance_ <= 0.0 ||
      imu_orientation_variance_ <= 0.0 ||
      imu_angular_velocity_variance_ <= 0.0)
    {
      throw std::runtime_error("age limits and covariance parameters must be positive");
    }

    leg_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      leg_odom_output_topic_, rclcpp::SensorDataQoS().keep_last(10));
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      imu_output_topic_, rclcpp::SensorDataQoS().keep_last(20));

    leg_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      leg_odom_input_topic_, rclcpp::SensorDataQoS().keep_last(10),
      std::bind(&EkfInputAdapter::on_leg_odom, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_input_topic_, rclcpp::SensorDataQoS().keep_last(20),
      std::bind(&EkfInputAdapter::on_imu, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "EKF input adapter: leg %s -> %s, IMU %s -> %s, frame %s -> %s",
      leg_odom_input_topic_.c_str(), leg_odom_output_topic_.c_str(),
      imu_input_topic_.c_str(), imu_output_topic_.c_str(),
      odom_frame_.c_str(), base_frame_.c_str());
  }

private:
  bool valid_stamp(const builtin_interfaces::msg::Time & stamp, const char * sensor_name)
  {
    const rclcpp::Time source_stamp(stamp);
    if (source_stamp.nanoseconds() <= 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "%s has a zero timestamp", sensor_name);
      return false;
    }
    const double age = (get_clock()->now() - source_stamp).seconds();
    if (age > max_past_age_seconds_ || age < -max_future_age_seconds_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "%s timestamp age %.6f s is outside [%.3f, %.3f] s",
        sensor_name, age, -max_future_age_seconds_, max_past_age_seconds_);
      return false;
    }
    return true;
  }

  void on_leg_odom(const nav_msgs::msg::Odometry::ConstSharedPtr input)
  {
    if (!valid_stamp(input->header.stamp, "leg odometry")) {
      return;
    }
    if (input->header.frame_id != input_odom_frame_ || input->child_frame_id != base_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting leg odometry frames '%s' -> '%s'; expected '%s' -> '%s'",
        input->header.frame_id.c_str(), input->child_frame_id.c_str(),
        input_odom_frame_.c_str(), base_frame_.c_str());
      return;
    }
    const auto & position = input->pose.pose.position;
    if (!finite_value(position.x) || !finite_value(position.y)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Rejecting non-finite leg odometry position");
      return;
    }

    nav_msgs::msg::Odometry output = *input;
    output.header.frame_id = odom_frame_;
    output.child_frame_id = base_frame_;
    output.pose.pose.position.z = 0.0;
    output.pose.pose.orientation.x = 0.0;
    output.pose.pose.orientation.y = 0.0;
    output.pose.pose.orientation.z = 0.0;
    output.pose.pose.orientation.w = 1.0;
    output.twist.twist.linear.z = 0.0;
    output.twist.twist.angular.x = 0.0;
    output.twist.twist.angular.y = 0.0;
    output.twist.twist.angular.z = 0.0;
    output.pose.covariance.fill(0.0);
    output.twist.covariance.fill(0.0);
    output.pose.covariance[0] = leg_position_variance_;
    output.pose.covariance[7] = leg_position_variance_;
    // z=0 is a local planar pseudo-measurement used only to keep the EKF's
    // otherwise unobservable vertical states bounded. The raw vendor z is a
    // nearly constant nominal height and is intentionally not forwarded.
    output.pose.covariance[14] = leg_z_constraint_variance_;
    output.pose.covariance[21] = 1.0e6;
    output.pose.covariance[28] = 1.0e6;
    output.pose.covariance[35] = 1.0e6;
    output.twist.covariance[0] = 1.0e6;
    output.twist.covariance[7] = 1.0e6;
    output.twist.covariance[14] = 1.0e6;
    output.twist.covariance[21] = 1.0e6;
    output.twist.covariance[28] = 1.0e6;
    output.twist.covariance[35] = 1.0e6;
    leg_odom_pub_->publish(output);
  }

  void on_imu(const sensor_msgs::msg::Imu::ConstSharedPtr input)
  {
    if (!valid_stamp(input->header.stamp, "IMU")) {
      return;
    }
    if (input->header.frame_id != base_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting IMU frame '%s'; expected '%s'",
        input->header.frame_id.c_str(), base_frame_.c_str());
      return;
    }
    if (!finite_quaternion(input->orientation) ||
      !finite_value(input->angular_velocity.x) ||
      !finite_value(input->angular_velocity.y) ||
      !finite_value(input->angular_velocity.z))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting non-finite IMU orientation/angular velocity");
      return;
    }

    const auto & q = input->orientation;
    const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (!finite_value(norm) || norm < 0.5) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Rejecting invalid IMU quaternion");
      return;
    }

    sensor_msgs::msg::Imu output = *input;
    output.orientation.x /= norm;
    output.orientation.y /= norm;
    output.orientation.z /= norm;
    output.orientation.w /= norm;
    output.orientation_covariance.fill(0.0);
    output.angular_velocity_covariance.fill(0.0);
    output.linear_acceleration_covariance.fill(0.0);
    output.orientation_covariance[0] = imu_orientation_variance_;
    output.orientation_covariance[4] = imu_orientation_variance_;
    output.orientation_covariance[8] = imu_orientation_variance_;
    output.angular_velocity_covariance[0] = imu_angular_velocity_variance_;
    output.angular_velocity_covariance[4] = imu_angular_velocity_variance_;
    output.angular_velocity_covariance[8] = imu_angular_velocity_variance_;
    output.linear_acceleration_covariance[0] = 1.0e6;
    output.linear_acceleration_covariance[4] = 1.0e6;
    output.linear_acceleration_covariance[8] = 1.0e6;
    imu_pub_->publish(output);
  }

  std::string leg_odom_input_topic_;
  std::string imu_input_topic_;
  std::string leg_odom_output_topic_;
  std::string imu_output_topic_;
  std::string odom_frame_;
  std::string input_odom_frame_;
  std::string base_frame_;
  double max_past_age_seconds_{0.25};
  double max_future_age_seconds_{0.05};
  double leg_position_variance_{0.01};
  double leg_z_constraint_variance_{0.01};
  double imu_orientation_variance_{0.0025};
  double imu_angular_velocity_variance_{0.0004};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr leg_odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr leg_odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<EkfInputAdapter>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("ekf_input_adapter"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

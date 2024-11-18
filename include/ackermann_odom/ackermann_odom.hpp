#ifndef ACKERMANN_ODOM__ACKERMANN_ODOM_HPP_
#define ACKERMANN_ODOM__ACKERMANN_ODOM_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "ackermann_odom/msg/float32_stamped.hpp"

class AckermannOdom : public rclcpp::Node {
 public:
  AckermannOdom();

 private:
  static constexpr double MAX_VELOCITY = 22.0;
  static constexpr double MAX_STEERING_ANGLE = M_PI / 6;
  static constexpr double SMALL_ANGLE_THRESHOLD = 1e-6;
  static constexpr double MAX_POSITION_CHANGE = 1.0;
  static constexpr double MAX_ENCODER_DIFF = 1000.0;
  static constexpr double ALPHA = 0.1;

  void declare_parameters();
  void initialize_parameters();
  void initialize_publishers_and_subscribers();

  void steeringCallback(const std_msgs::msg::Float32::SharedPtr msg);

  void OdomCallback(
      const sensor_msgs::msg::JointState::ConstSharedPtr& left_msg,
      const sensor_msgs::msg::JointState::ConstSharedPtr& right_msg,
      const ackermann_odom::msg::Float32Stamped::ConstSharedPtr& steering_msg);

  void updateOdometry(
      const rclcpp::Time& current_time,
      const sensor_msgs::msg::JointState::ConstSharedPtr& left_msg,
      const sensor_msgs::msg::JointState::ConstSharedPtr& right_msg,
      const ackermann_odom::msg::Float32Stamped::ConstSharedPtr& steering_msg,
      double dt);

  void publishOdometry();

  rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter> & parameters);

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Publisher<ackermann_odom::msg::Float32Stamped>::SharedPtr publisher_steering_stamped_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steering_sub_;
  message_filters::Subscriber<sensor_msgs::msg::JointState> left_encoder_sub_;
  message_filters::Subscriber<sensor_msgs::msg::JointState> right_encoder_sub_;
  message_filters::Subscriber<ackermann_odom::msg::Float32Stamped> steering_stamped_sub_;
  std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::JointState, sensor_msgs::msg::JointState,
      ackermann_odom::msg::Float32Stamped>>> sync_;

  double wheel_radius_, wheelbase_, track_width_, conversion_ratio_, encoder_resolution_;
  double prev_left_encoder_count_, prev_right_encoder_count_;
  double elapsed_time_, x_, y_, th_, previous_v_;
  rclcpp::Time last_time_;

  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

#endif // ACKERMANN_ODOM__ACKERMANN_ODOM_HPP_
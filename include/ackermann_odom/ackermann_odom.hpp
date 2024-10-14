#ifndef ACKERMANN_ODOM__ACKERMANN_ODOM_NODE_HPP_
#define ACKERMANN_ODOM__ACKERMANN_ODOM_NODE_HPP_

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
  void steeringCallback(
      const std_msgs::msg::Float32::SharedPtr msg);

  void OdomCallback(
      const sensor_msgs::msg::JointState::ConstSharedPtr& left_msg,
      const sensor_msgs::msg::JointState::ConstSharedPtr& right_msg,
      const ackermann_odom::msg::Float32Stamped::ConstSharedPtr& steering_msg);

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Publisher<ackermann_odom::msg::Float32Stamped>::SharedPtr publisher_steering_stamped_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steering_sub_;
  message_filters::Subscriber<sensor_msgs::msg::JointState> left_encoder_sub_;
  message_filters::Subscriber<sensor_msgs::msg::JointState> right_encoder_sub_;
  message_filters::Subscriber<ackermann_odom::msg::Float32Stamped> steering_stamped_sub_;
  std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState,ackermann_odom::msg::Float32Stamped>>> sync_;
    
  double prev_left_encoder_count;
  double prev_right_encoder_count;
  double x_;
  double y_;
  double th_;

  double x_0;
  double y_0;
  double th_0;
  double elapsed_time_;
  rclcpp::Time last_time_;
};

#endif // ACKERMANN_ODOM__ACKERMANN_ODOM_NODE_HPP_
#include "ackermann_odom/ackermann_odom.hpp"

#include <math.h>

AckermannOdom::AckermannOdom() : Node("ackermann_odom_node") {
  RCLCPP_INFO(this->get_logger(), "Hi I'm ackermann_odom!");

  // Create a publisher
  publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  publisher_steering_stamped_ =
      this->create_publisher<ackermann_odom::msg::Float32Stamped>(
          "/steering_stamped", 10);

  steering_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/autodrive/f1tenth_1/steering", 10,
      std::bind(&AckermannOdom::steeringCallback, this, std::placeholders::_1));

  // Create subscribers
  left_encoder_sub_.subscribe(this, "/autodrive/f1tenth_1/left_encoder");
  right_encoder_sub_.subscribe(this, "/autodrive/f1tenth_1/right_encoder");
  steering_stamped_sub_.subscribe(this, "/steering_stamped");

  // Synchronize the subscriptions
  sync_ = std::make_shared<message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<
          sensor_msgs::msg::JointState, sensor_msgs::msg::JointState,
          ackermann_odom::msg::Float32Stamped>>>(
      message_filters::sync_policies::ApproximateTime<
          sensor_msgs::msg::JointState, sensor_msgs::msg::JointState,
          ackermann_odom::msg::Float32Stamped>(10),
      left_encoder_sub_, right_encoder_sub_, steering_stamped_sub_);
  sync_->registerCallback(
      std::bind(&AckermannOdom::OdomCallback, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3));

  // Initialize variables
  prev_left_encoder_count = 0.0;
  prev_right_encoder_count = 0.0;
  elapsed_time_ = 0.0;
  x_ = 0.0;
  y_ = 0.0;
  th_ = 0.0;
  last_time_ = this->get_clock()->now();
  RCLCPP_INFO(this->get_logger(), "AckermannOdom node initialized");
}

void AckermannOdom::steeringCallback(
    const std_msgs::msg::Float32::SharedPtr msg) {
  ackermann_odom::msg::Float32Stamped stamped_msg;
  stamped_msg.header.stamp = this->get_clock()->now();
  stamped_msg.data = msg->data;
  publisher_steering_stamped_->publish(stamped_msg);
}

void AckermannOdom::OdomCallback(
    const sensor_msgs::msg::JointState::ConstSharedPtr& left_msg,
    const sensor_msgs::msg::JointState::ConstSharedPtr& right_msg,
    const ackermann_odom::msg::Float32Stamped::ConstSharedPtr& steering_msg) {
  RCLCPP_INFO(this->get_logger(), "Received synchronized messages");

  if (!left_msg || !right_msg || !steering_msg) {
    RCLCPP_ERROR(this->get_logger(), "Received null message");
    return;
  }

  double wheel_radius = 0.0590;
  double wheel_base = 0.3240;   // L
  double track_width = 0.2360;  // w

  // Create a new odometry message
  auto odom = nav_msgs::msg::Odometry();
  odom.header.stamp = this->get_clock()->now();
  odom.header.frame_id = "world";
  odom.child_frame_id = "f1tenth_1";

  // Compute the velocity
  rclcpp::Time current_time = this->get_clock()->now();
  double dt = (current_time - last_time_).seconds();
  if (dt == 0) {
    RCLCPP_ERROR(this->get_logger(), "Delta time is zero");
    return;
  }

  double v_left =
      wheel_radius * (left_msg->position[0] - prev_left_encoder_count) / dt;
  double v_right =
      wheel_radius * (right_msg->position[0] - prev_right_encoder_count) / dt;
  // Compute linear velocity
  double v = (v_left + v_right) / 2.0;

  // Compute angular velocity
  double delta = steering_msg->data;  // Steering angle
  double R = wheel_base / std::tan(delta);
  double theta_dot = v / R;
  double theta = theta_dot * elapsed_time_ + th_0;

  // Update the pose
  double dx = v * std::cos(th_) * dt;
  double dy = v * std::sin(th_) * dt;
  double dth = theta_dot * dt;

  elapsed_time_ += dt;
  x_ += dx;
  y_ += dy;
  th_ += dth;
  while (th_ > M_PI) th_ -= 2 * M_PI;
  while (th_ < -M_PI) th_ += 2 * M_PI;

  // Update the twist
  odom.twist.twist.linear.x = dx;
  odom.twist.twist.linear.y = dy;
  odom.twist.twist.angular.z = theta_dot;

  // Update the pose in the odometry message
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  // Convert the angle to quaternion
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = std::sin(th_ / 2.0);
  odom.pose.pose.orientation.w = std::cos(th_ / 2.0);

  // Publish the message
  publisher_->publish(odom);

  // Update previous encoder counts and time
  prev_left_encoder_count = left_msg->position[0];
  prev_right_encoder_count = right_msg->position[0];
  last_time_ = current_time;

  RCLCPP_INFO(this->get_logger(), "Published odometry message");
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AckermannOdom>());
  rclcpp::shutdown();
  return 0;
}
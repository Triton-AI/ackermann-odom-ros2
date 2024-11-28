#include "ackermann_odom/ackermann_odom.hpp"

#include <cmath>
#include <memory>
#include <limits>
#include <algorithm>

AckermannOdom::AckermannOdom() : Node("ackermann_odom_node")
{
    declare_parameters();
    initialize_parameters();
    initialize_publishers_and_subscribers();

    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&AckermannOdom::parametersCallback, this, std::placeholders::_1));
}

void AckermannOdom::declare_parameters()
{
    this->declare_parameter("wheel_radius", 0.0590);
    this->declare_parameter("wheelbase", 0.3240);
    this->declare_parameter("track_width", 0.2360);
    this->declare_parameter("conversion_ratio", 120.0);
    this->declare_parameter("encoder_resolution", 16.0);
}

void AckermannOdom::initialize_parameters()
{
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    wheelbase_ = this->get_parameter("wheelbase").as_double();
    track_width_ = this->get_parameter("track_width").as_double();
    conversion_ratio_ = this->get_parameter("conversion_ratio").as_double();
    encoder_resolution_ = this->get_parameter("encoder_resolution").as_double();
}

void AckermannOdom::initialize_publishers_and_subscribers()
{
    publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    publisher_steering_stamped_ =
        this->create_publisher<ackermann_odom::msg::Float32Stamped>("/steering_stamped", 10);

    steering_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/autodrive/f1tenth_1/steering", 10,
        std::bind(&AckermannOdom::steeringCallback, this, std::placeholders::_1));

    left_encoder_sub_.subscribe(this, "/autodrive/f1tenth_1/left_encoder");
    right_encoder_sub_.subscribe(this, "/autodrive/f1tenth_1/right_encoder");
    steering_stamped_sub_.subscribe(this, "/steering_stamped");
    imu_sub_.subscribe(this, "/ego_racecar/imu");

    sync_ = std::make_shared<message_filters::Synchronizer<
            message_filters::sync_policies::ApproximateTime<
                sensor_msgs::msg::JointState, 
                sensor_msgs::msg::JointState,
                ackermann_odom::msg::Float32Stamped, 
                sensor_msgs::msg::Imu
            >
        >
    >(
            message_filters::sync_policies::ApproximateTime<
                sensor_msgs::msg::JointState, 
                sensor_msgs::msg::JointState,
                ackermann_odom::msg::Float32Stamped, 
                sensor_msgs::msg::Imu>(10),                              
            left_encoder_sub_, right_encoder_sub_, steering_stamped_sub_, imu_sub_
);
    sync_->registerCallback(
        std::bind(&AckermannOdom::OdomCallback, this, 
                  std::placeholders::_1,
                  std::placeholders::_2,
                  std::placeholders::_3,
                  std::placeholders::_4));

    prev_left_encoder_count_ = 0.0;
    prev_right_encoder_count_ = 0.0;
    elapsed_time_ = 0.0;
    x_ = 0.0;
    y_ = 0.0;
    th_ = 0.0;
    previous_v_ = 0.0;
    last_time_ = this->get_clock()->now();
}

void AckermannOdom::steeringCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    ackermann_odom::msg::Float32Stamped stamped_msg;
    stamped_msg.header.stamp = this->get_clock()->now();
    stamped_msg.data = msg->data;
    publisher_steering_stamped_->publish(stamped_msg);
}

void AckermannOdom::OdomCallback(
    const sensor_msgs::msg::JointState::ConstSharedPtr& left_msg,
    const sensor_msgs::msg::JointState::ConstSharedPtr& right_msg,
    const ackermann_odom::msg::Float32Stamped::ConstSharedPtr& steering_msg,
    const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg)
{
    if (!left_msg || !right_msg || !steering_msg) {
        RCLCPP_ERROR(this->get_logger(), "Received null message");
        return;
    }

    rclcpp::Time current_time = this->get_clock()->now();
    double dt = (current_time - last_time_).seconds();
    
    if (dt < 1e-6) {
        RCLCPP_WARN(this->get_logger(), "Delta time too small: %.5f", dt);
        return;
    }

    updateOdometry(current_time, left_msg, right_msg, steering_msg, dt);
    publishOdometry();

    last_time_ = current_time;
}

void AckermannOdom::updateOdometry(
    const rclcpp::Time& current_time,
    const sensor_msgs::msg::JointState::ConstSharedPtr& left_msg,
    const sensor_msgs::msg::JointState::ConstSharedPtr& right_msg,
    const ackermann_odom::msg::Float32Stamped::ConstSharedPtr& steering_msg,
    double dt)
{
    double left_encoder_diff = left_msg->position[0] - prev_left_encoder_count_;
    double right_encoder_diff = right_msg->position[0] - prev_right_encoder_count_;

    // Set Prev encoder counts to Current encoder counts
    prev_left_encoder_count_ = left_msg->position[0];
    prev_right_encoder_count_ = right_msg->position[0];

    // Debug Line
    // RCLCPP_INFO(this->get_logger(), "left: %.3f, prev_left: %.3f", 
    //             left_msg->position[0], prev_left_encoder_count_);

    if (std::abs(left_encoder_diff) > MAX_ENCODER_DIFF || std::abs(right_encoder_diff) > MAX_ENCODER_DIFF) {
        RCLCPP_WARN(this->get_logger(), "Large encoder difference detected. Left: %.2f, Right: %.2f", left_encoder_diff, right_encoder_diff);
        return;
    }

	double left_encoder_rad = left_encoder_diff;
    double right_encoder_rad = right_encoder_diff;

    // Debug Line
    // RCLCPP_INFO(this->get_logger(), "left: %.3f, right: %.3f", 
    //             left_encoder_rad, right_encoder_rad);

    double v_left = wheel_radius_ * left_encoder_rad / dt;
    double v_right = wheel_radius_ * right_encoder_rad / dt;

    // Debug Line
    // RCLCPP_INFO(this->get_logger(), "v_left: %.3f, v_right: %.3f, dt: %.3f", 
    //             v_left, v_right, dt);
    
    // Smoothing Velocity (might not need it)
    double v = std::clamp((v_left + v_right) / 2.0, -MAX_VELOCITY, MAX_VELOCITY);
    // if (std::abs(v) < VELOCITY_DECAY_THRESHOLD) {
	// 	if (std::abs(v) < std::abs(previous_v_)) {
	// 		v = previous_v_ * VELOCITY_DECAY_RATE;  // Exponential decay
	// 	}
	// }
	// v = ALPHA * v + (1 - ALPHA) * previous_v_;
    
	previous_v_ = v;

    double delta = steering_msg->data * MAX_STEERING_ANGLE;
    double omega = v * std::tan(delta) / wheelbase_;

    double dx, dy, dth;
	
	// Smooth omega using a low-pass filter
    // static double previous_omega = 0.0;
	// double unsmoothed_omega = v * std::tan(delta) / wheelbase_;
    // double ALPHA_OMEGA = 0.3;  // Smoothing factor for angular velocity
    // omega = ALPHA_OMEGA * unsmoothed_omega + (1 - ALPHA_OMEGA) * previous_omega;

    // Update previous value
    // previous_omega = omega;

    if (std::abs(omega * dt) < SMALL_ANGLE_THRESHOLD) {
        dx = v * dt;
        dy = 0;
        dth = 0;
    } else {
        dth = omega * dt;
        dx = (v / omega) * std::sin(dth);
        dy = (v / omega) * (1 - std::cos(dth));
    }
	// Reset velocity if position changes are very small
    if (std::abs(dx) < POSITION_VELOCITY_RESET_THRESHOLD && 
        std::abs(dy) < POSITION_VELOCITY_RESET_THRESHOLD) {
        v = 0.0;
        previous_v_ = 0.0;
        dx = 0.0;
        dy = 0.0;
    }
	// Apply dead-zone filtering
    dx = (std::abs(dx) < POSITION_DEAD_ZONE) ? 0.0 : dx;
    dy = (std::abs(dy) < POSITION_DEAD_ZONE) ? 0.0 : dy;

	// Calculate new position
    double new_x = x_ + dx * std::cos(th_) - dy * std::sin(th_);
    double new_y = y_ + dx * std::sin(th_) + dy * std::cos(th_);

    // dx = std::clamp(dx, -MAX_POSITION_CHANGE, MAX_POSITION_CHANGE);
    // dy = std::clamp(dy, -MAX_POSITION_CHANGE, MAX_POSITION_CHANGE);

    // Apply complementary filter to position
    x_ = BETA * new_x + (1 - BETA) * x_;
    y_ = BETA * new_y + (1 - BETA) * y_;
    
	// Normalize theta to prevent drift in orientation
    th_ += dth;
    th_ = std::atan2(std::sin(th_), std::cos(th_));
    elapsed_time_ += dt;

    th_ = std::atan2(std::sin(th_), std::cos(th_));

	RCLCPP_INFO(this->get_logger(), "v_left: %.5f, v_right: %.5f", v_left, v_right);
	RCLCPP_INFO(this->get_logger(), "dx: %.5f, dy: %.5f, dth: %.5f", dx, dy, dth);
    RCLCPP_INFO(this->get_logger(), 
        "delta: %.5f, omega: %.5f, dth: %.5f, th_: %.5f",
        delta, omega, dth, th_);

    RCLCPP_INFO(this->get_logger(), 
        "Updated pose -> x: %.2f, y: %.2f, th: %.2f, v: %.5f, steering: %.5f", 
        x_, y_, th_, v, delta);
}

void AckermannOdom::publishOdometry()
{
    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = this->get_clock()->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "ego_racecar/base_footprint";

    odom.twist.twist.linear.x = previous_v_ * std::cos(th_);
    odom.twist.twist.linear.y = previous_v_ * std::sin(th_);
    odom.twist.twist.angular.z = previous_v_ * std::tan(th_) / wheelbase_;

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = std::sin(th_ / 2.0);
    odom.pose.pose.orientation.w = std::cos(th_ / 2.0);

    // Set covariances
    for (int i = 0; i < 36; i++) {
        odom.pose.covariance[i] = 0.0;
        odom.twist.covariance[i] = 0.0;
    }
    odom.pose.covariance[0] = odom.pose.covariance[7] = odom.pose.covariance[35] = 0.01;
    odom.twist.covariance[0] = odom.twist.covariance[7] = odom.twist.covariance[35] = 0.01;

    publisher_->publish(odom);
}

rcl_interfaces::msg::SetParametersResult AckermannOdom::parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & param : parameters)
    {
        if (param.get_name() == "wheel_radius")
        {
            wheel_radius_ = param.as_double();
        }
        else if (param.get_name() == "wheelbase")
        {
            wheelbase_ = param.as_double();
        }
        else if (param.get_name() == "track_width")
        {
            track_width_ = param.as_double();
        }
        else if (param.get_name() == "conversion_ratio")
        {
            conversion_ratio_ = param.as_double();
        }
        else if (param.get_name() == "encoder_resolution")
        {
            encoder_resolution_ = param.as_double();
        }
    }
    return result;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AckermannOdom>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
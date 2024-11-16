#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class EncoderNode : public rclcpp::Node
{
public:
  EncoderNode() : Node("encoder_node"), position_(0.0)
  {
    this->declare_parameter("topic_name", "joint_states");
    this->declare_parameter("joint_name", "wheel_joint");

    topic_name_ = this->get_parameter("topic_name").as_string();
    joint_name_ = this->get_parameter("joint_name").as_string();

    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(topic_name_, 10);
    timer_ = this->create_wall_timer(
      100ms, std::bind(&EncoderNode::publish_joint_state, this));
  }

private:
  void publish_joint_state()
  {
    auto message = sensor_msgs::msg::JointState();
    message.header.stamp = this->now();
    message.header.frame_id = "left_encoder";
    message.name.push_back(joint_name_);
    message.position.push_back(position_);
    message.velocity.push_back(1.0);  // Dummy constant velocity
    message.effort.push_back(0.0);    // No effort information

    publisher_->publish(message);

    position_ += 0.1;  // Increment position for dummy data
    if (position_ > 2*M_PI) {  // Reset after one full rotation
      position_ = 0.0;
    }
    position_ = 0.0;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  std::string topic_name_;
  std::string joint_name_;
  double position_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EncoderNode>());
  rclcpp::shutdown();
  return 0;
}
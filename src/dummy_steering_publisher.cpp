#include <chrono>
#include <functional>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

class DummySteeringPublisher : public rclcpp::Node
{
public:
  DummySteeringPublisher()
  : Node("dummy_steering_publisher"), time_(0.0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/steering_dummy", 10);
    timer_ = this->create_wall_timer(
      100ms, std::bind(&DummySteeringPublisher::publish_steering, this));
  }

private:
  void publish_steering()
  {
    auto message = std_msgs::msg::Float32();
    // Generate a sinusoidal steering angle between -0.5 and 0.5 radians
    // message.data = 0.5 * std::sin(time_);
    message.data = -1.0;
    
    publisher_->publish(message);
    
    time_ += 0.1;  // Increment time
    if (time_ > 2 * M_PI) {
      time_ -= 2 * M_PI;  // Reset after one full cycle
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
  double time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  while (true){
    rclcpp::spin(std::make_shared<DummySteeringPublisher>());
  }
  rclcpp::shutdown();
  return 0;
}
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>

using namespace std::chrono_literals;

/*

This file is for testing the subscriber in move_robot_client.cpp

*/

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher")
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::Point>("point_pub", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    geometry_msgs::msg::Point prev;

    void timer_callback()
    {
      geometry_msgs::msg::Point message;
      
      message.x = 0.5;
      message.y = 0.5;
      message.z = 0.25;

      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

// command line publish
// ros2 topic pub -1 /point_pub geometry_msgs/msg/Point "{x: 0.5, y: 0.5, z: 0.25}"
// ros2 topic pub -1 /point_pub geometry_msgs/msg/Point "{x: 0.3, y: 0.5, z: 0.25}"

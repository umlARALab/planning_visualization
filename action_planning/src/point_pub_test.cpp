#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cstdlib>
#include <ctime>

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
      publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("point_pub", 10);
      timer_ = this->create_wall_timer(
      20s, std::bind(&MinimalPublisher::timer_callback, this));

      srand(time(0));
    }

  private:
    void timer_callback()
    {
      geometry_msgs::msg::PointStamped message;
      message.header.frame_id = "panda_link0";
      
      // message.point.x = ((rand() % 10) - 5.0) / 10.0;
      // message.point.y = ((rand() % 10) - 5.0) / 10.0;;
      // message.point.z = 0.25;

      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
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
/*
ros2 topic pub -1 /point_pub geometry_msgs/msg/Point "{x: 0.5, y: 0.5, z: 0.25}"
ros2 topic pub -1 /point_pub geometry_msgs/msg/PointStamped "{
header: {frame_id: "panda_link0"}, point: {x: 0.5, y: 0.5, z: 0.25}
}"

*/ 
// ros2 topic pub -1 /point_pub geometry_msgs/msg/Point "{x: 0.5, y: 0.5, z: 0.25}"

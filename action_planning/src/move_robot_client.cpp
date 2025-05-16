#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <geometry_msgs/msg/pose.hpp>

#include "moveit_action/action/robot_move.hpp"

class ArmActionClient : public rclcpp::Node {
 public:
    using RobotAction = moveit_action::action::RobotMove;
    using GoalHandleTest = rclcpp_action::ServerGoalHandle<RobotAction>;

    explicit ArmActionClient(const rclcpp::NodeOptions &options) : Node("arm_action_client") {
        this->client_ptr_ = rclcpp_action::create_client<RobotAction>(this, "arm_action");
    }

    void send_goal(geometry_msgs::Pose pose) {
        // wait for server
        if (!this->client_ptr_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "action server not available after waiting");
            rclcpp::shutdown();
        }

        // create goal
        auto goal_msg = RobotAction::Goal();
        goal_msg.pose_goal = pose;

        // send goal
        RCLCPP_INFO(this->get_logger(), "Sending goal...");

    }

 private:
    rclcpp_action::Client<RobotAction>::SharedPtr client_ptr_;

}; // Arm Action Client Class

int main (int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto client = std::make_shared<ArmActionClient>();

    return 0;
}
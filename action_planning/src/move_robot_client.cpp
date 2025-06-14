#include <functional>
#include <memory>
#include <thread>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "moveit_action/action/robot_move.hpp"

class ArmActionClient : public rclcpp::Node {
 public:
    using RobotAction = moveit_action::action::RobotMove;
    using GoalHandleTest = rclcpp_action::ClientGoalHandle<RobotAction>;

    explicit ArmActionClient(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("action_client", options) {
        this->client_ptr_ = rclcpp_action::create_client<RobotAction>(this, "arm_action");

        // status = rclcpp_action::GoalStatus::STATUS_EXECUTING;

    }

    void send_goal(std::vector<geometry_msgs::msg::Pose> pose) {
        // wait for server
        if (!this->client_ptr_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "action server not available after waiting");
            rclcpp::shutdown();
        }

        // create goal
        auto goal_msg = RobotAction::Goal();

        // send goals to server one at a time; time out if it takes longer than a minute
        int i = 1;
        for (geometry_msgs::msg::Pose p : pose) {
            RCLCPP_INFO(this->get_logger(), "Sending goal %d...", i);

            // set goal
            goal_msg.pose_goal = p;

            // send goal to server when it is ready
            if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(60))) {
                RCLCPP_INFO(this->get_logger(), "[client] Action server not ready in time");
            }

            RCLCPP_INFO(this->get_logger(), "[client] Action server ready");

            this->client_ptr_->async_send_goal(goal_msg);
            rclcpp::sleep_for(std::chrono::seconds(10));

            i++;
        }

        // auto goal_options = rclcpp_action::Client<RobotAction>::SendGoalOptions();
        // goal_options.goal_response_callback = std::bind(%ArmActionClient::goal)
        // this->client_ptr_->async_send_goal(goal_msg);

    }

 private:
    rclcpp_action::Client<RobotAction>::SharedPtr client_ptr_;
    // rclcpp_action::GoalStatus status;

    void goal_response_callback(std::shared_future<GoalHandleTest::SharedPtr> future) {
        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal rejected by server.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result.");
        }
    }

    // void feedback_callback(GoalHandleTest::SharedPtr ptr, const std::shared_ptr<const RobotAction::Feedback> feedback) {
    //     RCLCPP_INFO(this->get_logger(), feedback->progress);
    // }

    void result_callback(const GoalHandleTest::WrappedResult &result) {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal canceleld");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown goal result");
            break;
        }
    }

}; // Arm Action Client Class

int main (int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // std::chrono::duration<int> sleep_time = (std::chrono::seconds) 30;
    auto client = std::make_shared<ArmActionClient>();

    // create vector of points
    std::vector<geometry_msgs::msg::Pose> pose_list;

    // use test pose to see if it works;
    geometry_msgs::msg::Pose test_pose;
    test_pose.position.x = 0.5;
    test_pose.position.y = 0.5;
    test_pose.position.z = 0.25;

    tf2::Quaternion rot, start, end;
    start.setRPY(0.0, 0.0, 0.0);
    rot.setRPY(M_PI, 0.0, 0.0);
    end = rot * start;
    end.normalize();
    test_pose.orientation = tf2::toMsg(end);

    geometry_msgs::msg::Pose test2;
    test2.position.x = 0.3;
    test2.position.y = 0.5;
    test2.position.z = 0.5;
    test2.orientation = tf2::toMsg(end);

    pose_list.push_back(test_pose);
    pose_list.push_back(test2);

    client->send_goal(pose_list);

    // rclcpp::sleep_for(sleep_time);

    rclcpp::spin(client);
    rclcpp::shutdown();

    return 0;
}
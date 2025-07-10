#include <functional>
#include <memory>
#include <thread>
#include <vector>
#include <cmath>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "moveit_action/action/robot_move.hpp"

std::string topic_name = "point_pub";

/*

This file contains the action client to send a point to the action server, as well as a subscriber that will listen to a topic
that will publish points.  

*/

class ArmActionClient : public rclcpp::Node {
 public:
    using RobotAction = moveit_action::action::RobotMove;
    using GoalHandleTest = rclcpp_action::ClientGoalHandle<RobotAction>;

    explicit ArmActionClient(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("action_client", options) {
        this->client_ptr_ = rclcpp_action::create_client<RobotAction>(this, "arm_action");
        point_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            topic_name, 10, std::bind(&ArmActionClient::topic_callback, this,std::placeholders::_1)
        );

        // status = rclcpp_action::GoalStatus::STATUS_EXECUTING;
        // wait for server
        RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for action server...");
        if (!this->client_ptr_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "action server not available after waiting");
            rclcpp::shutdown();
        }
        RCLCPP_INFO_ONCE(this->get_logger(), "Action server ready");

        prev.x = NULL;
        prev.y = NULL;
        prev.z = NULL;

    }

    void send_goal(geometry_msgs::msg::Pose pose) {
        // create goal
        auto goal_msg = RobotAction::Goal();

        // send goals to server one at a time; time out if it takes longer than a minute
        RCLCPP_INFO(this->get_logger(), "Sending goal point: (%f, %f, %f)", pose.position.x, pose.position.y, pose.position.z);

        // set goal
        goal_msg.pose_goal = pose;

        // send goal to server when it is ready
        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(60))) {
            RCLCPP_INFO(this->get_logger(), "Action server not ready in time");
        }

        // RCLCPP_INFO(this->get_logger(), "[client] Action server ready");

        // this->client_ptr_->async_send_goal(goal_msg);

        auto send_goal_options = rclcpp_action::Client<RobotAction>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&ArmActionClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&ArmActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&ArmActionClient::result_callback, this, std::placeholders::_1);

        client_ptr_->async_send_goal(goal_msg, send_goal_options);
        // RCLCPP_INFO(this->get_logger(), "[client] Action goal done");

        // auto goal_options = rclcpp_action::Client<RobotAction>::SendGoalOptions();
        // goal_options.goal_response_callback = std::bind(%ArmActionClient::goal)
        // this->client_ptr_->async_send_goal(goal_msg);

    }

 private:
    rclcpp_action::Client<RobotAction>::SharedPtr client_ptr_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr point_sub_;
    geometry_msgs::msg::Point prev;
    // rclcpp_action::GoalStatus status;

    void topic_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        geometry_msgs::msg::Pose target;
        std::string w;

        // check if its finding the same point as before
        if (!(prev.x == msg->x && prev.y == msg->y && prev.z == msg->z)) {
            RCLCPP_INFO(this->get_logger(), "\nGot point (%f, %f, %f). Plan and execute trajectory? (y/n)", msg->x, msg->y, msg->z);
            std::getline(std::cin, w);
            if (strcmp(w.c_str(), "y") == 0 || strcmp(w.c_str(), "Y") == 0) {
                target.position = *msg;
            
                tf2::Quaternion rot, start, end;
                start.setRPY(0.0, 0.0, 0.0);
                rot.setRPY(M_PI, 0.0, 0.0);
                end = rot * start;
                end.normalize();
                target.orientation = tf2::toMsg(end);

                send_goal(target);

                rclcpp::sleep_for(std::chrono::seconds(10));
            } else {
                RCLCPP_INFO(this->get_logger(), "Looking for next target...");
            }
        }

        prev.x = msg->x;
        prev.y = msg->y;
        prev.z = msg->z;
    }


    void goal_response_callback(const GoalHandleTest::SharedPtr &future) {
        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal rejected by server.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, planning and executing...");
        }
    }

    void feedback_callback(GoalHandleTest::SharedPtr ptr, const std::shared_ptr<const RobotAction::Feedback> feedback) {
        switch (feedback->is_execution_done) {
            case 1:
                RCLCPP_INFO(this->get_logger(), "Planning and execution completed with success!");
                break;
            default:
                RCLCPP_INFO(this->get_logger(), "Planning and execution failed...");
                break;
        }
    }

    void result_callback(const GoalHandleTest::WrappedResult &result) {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Ready for next target!");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal cancelled");
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

    // rclcpp::sleep_for(sleep_time);

    rclcpp::spin(client);
    rclcpp::shutdown();

    return 0;
}
#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <geometry_msgs/msg/pose.hpp>
// #include <geometry_msgs/msg/twist.hpp>

#include "moveit_action/action/robot_move.hpp"
#include <moveit/move_group_interface/move_group_interface.h>

class ArmActionServer : public rclcpp::Node {
 public:
    using RobotAction = moveit_action::action::RobotMove;
    using GoalHandleTest = rclcpp_action::ServerGoalHandle<RobotAction>;
    using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

    // initialize action server for the arm
    explicit ArmActionServer(const rclcpp::NodeOptions &options =  rclcpp::NodeOptions()) : Node("robot_action_server", options) {
        this->action_server_ = rclcpp_action::create_server<RobotAction>(
            this, 
            "arm_action",
            std::bind(&ArmActionServer::goal_callback, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ArmActionServer::cancel_callback, this, std::placeholders::_1),
            std::bind(&ArmActionServer::accept_goal, this, std::placeholders::_1)
        );

        move_group_node_ = rclcpp::Node::make_shared("robot_action_server", options);
        executor_.add_node(move_group_node_);
        std::thread([this]() {this->executor_.spin(); }).detach();

        move_group_interface_ = std::make_shared<MoveGroupInterface>(move_group_node_, "panda_arm");

        auto const logger = rclcpp::get_logger("arm_action");
    }

 private:
    rclcpp_action::Server<RobotAction>::SharedPtr action_server_;
    // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Node::SharedPtr move_group_node_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::shared_ptr<MoveGroupInterface> move_group_interface_;


    rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const RobotAction::Goal> goal) {
        // if the goal is in a specific range (?), execute
        if (goal->pose_goal.position.x <= 1 || goal->pose_goal.position.x >= -1) {
            // this->get_logger().info("\n[robot server: goal accepted]\n");
            RCLCPP_INFO_ONCE(this->get_logger(), "\n[robot server: goal accepted]\n");
            (void)uuid;
            
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        } else {
            RCLCPP_INFO_ONCE(this->get_logger(), "\n[robot server: goal rejected]\n");

            return rclcpp_action::GoalResponse::REJECT;
        }
    }

    rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<GoalHandleTest> goal_handle) {
            RCLCPP_INFO_ONCE(this->get_logger(), "\n[robot server: action cancelled]\n");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
    }

    void accept_goal(std::shared_ptr<GoalHandleTest> goal_handle) {
        // // needs to return quickly to avoid blocking executor --> spin new thread
        // std::thread{std::bind(&ArmActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();

        // GET REQUEST FROM GOALS
        const auto goal_pose = goal_handle->get_goal();
        auto result = std::make_shared<RobotAction::Result>();

        // EXECUTE ACTION
        // initialize moveit interface group
        // set target pose
        move_group_interface_->setPoseTarget(goal_pose->pose_goal);

        // create plan to goal
        MoveGroupInterface::Plan plan;
        bool success = (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        // auto const [success, plan] = [move_group_interface_]{
        //     moveit::planning_interface::MoveGroupInterface::Plan msg;
        //     auto const ok = static_cast<bool>(move_group_interface_->plan(msg));
        //     return std::make_pair(ok, msg);
        // }();


        // execute plan
        if (success) {
            RCLCPP_INFO_ONCE(this->get_logger(), "\n[robot server: executing]\n");

            move_group_interface_->execute(plan);

            // SEND RESULT
            result->result = "Execution success";
            goal_handle->succeed(result); 
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning failed");
        }     
    }

    // void execute(const const std::shared_ptr<GoalHandleTest> goal_handle) {
    // }
}; // class ArmActionServer

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto server = std::make_shared<ArmActionServer>();

    rclcpp::spin(server);
    rclcpp::shutdown();

    return 0;
}
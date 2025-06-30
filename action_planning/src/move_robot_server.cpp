#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// #include <geometry_msgs/msg/twist.hpp>

#include "moveit_action/action/robot_move.hpp"
#include <moveit/move_group_interface/move_group_interface.h>

// action class
class ArmActionServer : public rclcpp::Node {
 public:
    using RobotAction = moveit_action::action::RobotMove;
    using GoalHandleTest = rclcpp_action::ServerGoalHandle<RobotAction>;
    using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

    // initialize action server for the arm
    explicit ArmActionServer(const rclcpp::NodeOptions &options =  rclcpp::NodeOptions()) : Node("action_server", options) {
        // initialize action server with the name arm_action
        this->action_server_ = rclcpp_action::create_server<RobotAction>(
            this, 
            "arm_action",
            std::bind(&ArmActionServer::goal_callback, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ArmActionServer::cancel_callback, this, std::placeholders::_1),
            std::bind(&ArmActionServer::accept_goal, this, std::placeholders::_1)
        );

        // initialize move group node
        move_group_node_ = rclcpp::Node::make_shared("manipulator", options);
        executor_.add_node(move_group_node_);
        std::thread([this]() {this->executor_.spin(); }).detach();

        // initialize interface; panda arm is the moveit demo robot 
        arm_group_interface_ = std::make_shared<MoveGroupInterface>(move_group_node_, "panda_arm");
        gripper_group_interface_ = std::make_shared<MoveGroupInterface>(move_group_node_, "hand");

        // initialize logger for output
        auto const logger = rclcpp::get_logger("action_server");
        RCLCPP_INFO_ONCE(this->get_logger(), "\n[robot server: starting]\n");
    }

 private:
    rclcpp_action::Server<RobotAction>::SharedPtr action_server_;
    rclcpp::Node::SharedPtr move_group_node_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::shared_ptr<MoveGroupInterface> arm_group_interface_;
    std::shared_ptr<MoveGroupInterface> gripper_group_interface_;

    // accept or reject the goal 
    rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const RobotAction::Goal> goal) {
        // if the goal is within a threshold (?), execute
        // need to experiment with threshold values (1)
        if (goal->pose_goal.position.x <= 1 && goal->pose_goal.position.x >= -1) {
            RCLCPP_INFO_ONCE(this->get_logger(), "\n[robot server: goal accepted]\n");
            (void)uuid;
            
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        } else {
            RCLCPP_INFO_ONCE(this->get_logger(), "\n[robot server: goal rejected]\n");

            return rclcpp_action::GoalResponse::REJECT;
        }
    }

    // cancel the request
    rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<GoalHandleTest> goal_handle) {
            RCLCPP_INFO_ONCE(this->get_logger(), "\n[robot server: action cancelled]\n");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
    }

    // do stuff when the goal is executed
    void accept_goal(std::shared_ptr<GoalHandleTest> goal_handle) {
        // // needs to return quickly to avoid blocking executor --> spin new thread
        // std::thread{std::bind(&ArmActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
        RCLCPP_INFO_ONCE(this->get_logger(), "\n[robot server: planning]\n");

        // initialize container for list of waypoints
        std::vector<geometry_msgs::msg::Pose> waypoints;
        double eef_step = 0.01;
        double jump = 0.0;
        moveit_msgs::msg::RobotTrajectory trajectory_plan;

        tf2::Quaternion rot, start, end;
        start.setRPY(0.0, 0.0, 0.0);
        rot.setRPY(M_PI, 0.0, 0.0);
        end = rot * start;
        end.normalize();

        // GET REQUEST FROM GOALS
        const auto goal_pose = goal_handle->get_goal();
        auto result = std::make_shared<RobotAction::Result>();

        // ADD WAYPOINTS
        // prepare to pick
        geometry_msgs::msg::Pose start_point = goal_pose->pose_goal;
        start_point.position.z += 0.1;
        start_point.orientation = tf2::toMsg(end);
        waypoints.push_back(start_point);

        // lower to pick
        geometry_msgs::msg::Pose obj_point = start_point;
        obj_point.position.z -= 0.1;
        obj_point.orientation = tf2::toMsg(end);
        waypoints.push_back(obj_point);

        // pick up
        geometry_msgs::msg::Pose pick_point = obj_point;
        pick_point.position.z += 0.1;
        pick_point.orientation = tf2::toMsg(end);
        waypoints.push_back(pick_point);

        // place at target -- currently set at a fixed position
        geometry_msgs::msg::Pose target_point;
        target_point.position.x = 0.5;
        target_point.position.y = -0.5;
        target_point.position.z = 0.25;
        target_point.orientation = tf2::toMsg(end);
        waypoints.push_back(target_point);

        // EXECUTE ACTION
        // set target pose
        // move_group_interface_->setPoseTarget(goal_pose->pose_goal);

        double path_result = arm_group_interface_->computeCartesianPath(waypoints, eef_step, jump, trajectory_plan);

        // create plan to goal
        // MoveGroupInterface::Plan plan;
        // bool success = (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        // execute plan
        if (path_result >= 0.0) {
            RCLCPP_INFO_ONCE(this->get_logger(), "\n[robot server: executing]\n");

            arm_group_interface_->execute(trajectory_plan);

            // SEND RESULT
            result->result = "Execution success";
            goal_handle->succeed(result); 
            
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning failed");
        }
        // clear waypoints
        waypoints.clear();
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
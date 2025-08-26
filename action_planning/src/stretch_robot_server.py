#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from moveit_action import RobotMove

from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory
from stretch_body.robot import Robot
import numpy as np

# goal
# geometry_msgs/Pose pose_goal

# result
# string result

# feedback
# int32 is_execution_done

# ros2 topic pub -1 /stretch/joint_states trajectory_msgs/msg/JointTrajectory "{
# header: {frame_id: "base_link"}, joint_names: ["joint_lift"], points: [0.8]}
# }"

class StretchMoveServer(Node):
    def __init__(self):
        super().__init__('stretch_move_server')
        self.action_server = ActionServer(
            self, 
            RobotMove,
            'robot_move',
            self.execute)
        
        self.gripper_pub = self.create_publisher(JointTrajectory, '/stretch/joint_states', 10)
        
        self.robot = Robot()
        self.robot.startup()
    
    def execute(self, goal_handle):
        self.get_logger().info('Executing goal..')
        # do stuff here
        feedback_msg = RobotMove.Feedback()
        feedback_msg,is_execution_done = 0
        
        # break trajectory into [lift --> extend --> grab --> retract]
        lift_pose = Pose()
        lift_pose = goal_handle.request.pose_goal
        lift_pose.position.z = lift_pose.position.z + 0.1 # check if it's within limit

        open_gripper = JointTrajectory()
        open_gripper.header.frame_id = 'base_link'
        open_gripper.joint_names = ['joint_gripper_finger_left']
        open_gripper.points = [0.2]

        self.gripper_pub.publish()

        # lift
        self.robot.lift.move_to(lift_pose.position.z)
        self.robot.push_command()
        time.sleep(2)

        # extend
        self.robot.arm.move_to(lift_pose.position.x)
        self.robot.push_command()
        time.sleep(2)
        



def main(args=None):
    rclpy.init(args=args)
    server = StretchMoveServer()

    rclpy.spin(server)

if __name__ == '__main__':
    main()

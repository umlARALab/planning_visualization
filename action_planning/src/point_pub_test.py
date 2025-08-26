import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from stretch_body.robot import Robot
import numpy as np    
import time


def main(args=None):
    rclpy.init(args=args)

    robot = Robot()
    robot.startup()

    robot.lift.move_to(0.5)
    robot.push_command()

    robot.end_of_arm.move_to('stretch_gripper', 0)    
    time.sleep(2.0)

    robot.stop()

    rclpy.shutdown()


if __name__ == '__main__':
    main()


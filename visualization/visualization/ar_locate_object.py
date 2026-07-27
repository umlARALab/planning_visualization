import rclpy
from rclpy.node import Node
from enum import Enum

from geometry_msgs.msg import PointStamped, Pose
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry

import numpy as np
from scipy.spatial.transform import Rotation as R
from tf_transformations import euler_from_quaternion

import stretch_body.robot
from stretch_ar.msg import RobotStatus

class State(Enum):
    NOT_READY = 0
    IDLE = 1
    SEARCH = 2
    PLAN = 3
    MOVE = 4

# tf from base link to stretch head - Translation: [0.045, -0.003, 1.307]
base_to_head = [0.045, -0.003, 1.307]

class LocateTarget(Node):
    def __init__(self):
        super().__init__('ar_locate_object')

        self.stretch_pose = Pose()

        self.target_rotation = []
        self.robot_state = RobotStatus()

        self.robot = stretch_body.robot.Robot()
        did_startup = self.robot.startup()
        self.get_logger().info(f'Robot connected to hardware: {did_startup}')
        is_homed = self.robot.is_homed()
        self.get_logger().info(f'Robot is homed: {is_homed}')

        # get rough position estimation of target object
        self.obj_estimation_sub = self.create_subscription(
            PointStamped,
            '/object_position',
            self.locate_callback,
            10
        )

        self.runstop_sub = self.create_subscription(
            Bool, 
            '/stop_robot',
            self.runstop_callback,
            10
        )

        # make shift odom publisher and status publisher
        self.odom_pub = self.create_publisher(Pose, '/stretch_odom', 10)
        self.status_pub = self.create_publisher(RobotStatus, '/robot_feedback', 10)

    def runstop_callback(self, msg):
        self.get_logger().info(f'STOPPING AND DISCONNECTING FROM ROBOT')
        self.robot_state.data = "Stopping and disconnecting"
        self.robot_state.state = 0

        if msg.data:
            self.robot.arm.set_velocity(0.0)
            self.robot.base.set_translate_velocity(0.0)
            self.robot.base.set_rotational_velocity(0.0)
            self.robot.head.move_to('head_pan', 0.0, 0.8)
            self.robot.head.move_to('head_tilt', 0.0, 0.8)
            
            self.robot.push_command()
            self.robot.wait_command()

            self.robot.stop()

            self.status_pub.publish(self.status)
            rclpy.shutdown()
                
    # turn stretch camera to look at target object position
    def locate_callback(self, msg):
        obj_pt = msg.point
        self.robot_state.data = 'Searching for target'
        self.robot_state.state = 2

        updateOdom = Pose()
        updateOdom.position.x = 0.0
        updateOdom.position.y = 0.0
        updateOdom.position.z = 0.0

        # measure rotation from stretch to target object
        obj_v = [obj_pt.x, obj_pt.y, 0]
        stretch_v = [1, 0, 0]

        stretch_rot_obj = R.from_matrix(self.get_rotation_matrix(stretch_v, obj_v))
        self.target_rotation = stretch_rot_obj.as_euler('xyz')
        print(stretch_rot_obj.as_euler('xyz', True))
        print(stretch_rot_obj.as_quat())
        print('Base status angle: ' + str(self.robot.base.status['theta']))

        # print(self.robot.base.status['x'])
        # print(self.robot.base.status['y'])
        # print(self.robot.base.status['theta'])
        # print(' ')

        # rotate stretch towards object by z 
        self.robot.base.rotate_by(self.target_rotation[2])
        self.robot.push_command()
        # self.robot.wait_command()

        updateOdom.orientation.x = stretch_rot_obj.as_quat()[0]
        updateOdom.orientation.y = stretch_rot_obj.as_quat()[1]
        updateOdom.orientation.z = stretch_rot_obj.as_quat()[2]
        updateOdom.orientation.w = stretch_rot_obj.as_quat()[3]
        print(str(updateOdom.orientation))
        self.odom_pub.publish(updateOdom)

        # measure rotation from stretch camera to target object
        obj_v = [1, 0, obj_pt.z - base_to_head[2]]
        cam_rot_obj = R.from_matrix(self.get_rotation_matrix(stretch_v, obj_v))
        head_tilt_angle = cam_rot_obj.as_euler('xyz')

        self.robot.head.move_to('head_tilt', -head_tilt_angle[1], 0.8)
        self.robot.push_command()
        # self.robot.wait_command()

    def get_rotation_matrix(self, v1, v2):
        a = (v1 / np.linalg.norm(v1)).reshape(3)
        b = (v2 / np.linalg.norm(v2)).reshape(3)

        v = np.cross(a, b)
        c = np.dot(a, b)
        s = np.linalg.norm(v)

        kmat = np.array([
            [0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0]
        ])

        rot_mat = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
        return rot_mat

def main():
    rclpy.init()

    sub = LocateTarget()
    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



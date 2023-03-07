#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from math import atan2
import time


class Robot:
    def __init__(self, robot_name):
        self.sonar = 0.0  # Sonar distance
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.turning = False

        self.robot_name = robot_name
        self.ns = "/" + self.robot_name

        '''Listener and publisher'''
        rospy.Subscriber(self.ns + "/sensor/sonar_front",
                         Range, self.callbacksonar)
        rospy.Subscriber(self.ns + "/odom",
                         Odometry, self.callback_odom)

        self.cmd_vel_pub = rospy.Publisher(
            self.ns + "/cmd_vel", Twist, queue_size=1)

    def set_robot_turning(self):
        self.set_speed_angle(0, 1)
        self.turning = True

    def set_robot_moving(self):
        self.set_speed_angle(50, 0)
        self.turning = False

    def callbacksonar(self, data):
        # DO NOT TOUCH
        self.sonar = data.range

    def get_sonar(self):
        # DO NOT TOUCH
        return float(self.sonar)

    def constraint(self, val, min=-2.0, max=2.0):
        # DO NOT TOUCH
        if val < min:
            return min
        if val > max:
            return max
        return val

    def set_speed_angle(self, linear_vel, angular_vel):
        # DO NOT TOUCH
        cmd_vel = Twist()
        cmd_vel.linear.x = self.constraint(linear_vel)
        cmd_vel.angular.z = self.constraint(angular_vel, min=-1, max=1)
        self.cmd_vel_pub.publish(cmd_vel)

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = 2.0 * (w * y - z * x)
        t2 = 1.0 if t2 > 1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z  # in radians

    def callback_odom(self, data: Odometry):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y

        rot_q = data.pose.pose.orientation
        _, _, self.theta = self.euler_from_quaternion(
            rot_q.x, rot_q.y, rot_q.z, rot_q.w)


def run_demo():
    '''Main loop'''
    robot_name = rospy.get_param("~robot_name")
    robot = Robot(robot_name)
    print(f"Robot {robot_name} is starting".format())

    goal = Point()
    goal.x = rospy.get_param('/x_goal')
    goal.y = rospy.get_param('/y_goal')

    robot.set_robot_moving()
    wait_timer = None
    sonar_turn = False

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():

        x_diff = goal.x - robot.x
        y_diff = goal.y - robot.y

        angle_to_goal = atan2(y_diff, x_diff)

        # Write your strategy here ...
        sonar = robot.get_sonar()

        # get angle difference from robot orientation to goal
        angle_diff = abs(angle_to_goal - robot.theta)
        print(f"SONAR VALUE : {sonar:.2f}")
        print(f"ANGLE: {angle_diff}")
        print(f'TURNING: {robot.turning}')

        # if the robot was blocked by a wall, we keep advancing for 2 seconds to keep us from getting blocked
        if wait_timer and (time.time() - wait_timer) < 2.0:
            robot.set_robot_moving()
            rate.sleep()
            wait_timer = None
            continue

        if robot.turning:
            if not sonar_turn and angle_diff <= 0.3:
                # if we are turning because of the angle difference, once the angle difference is good, we resume going forward
                robot.set_robot_moving()
            elif sonar_turn and sonar > 4:
                # if we are turning because of the sonar, once there is no longer a obstacle, we continue advancing and set a time so that we can advance for a minimum of 2 seconds before turning again
                sonar_turn = False
                robot.set_robot_moving()
                wait_timer = time.time()
            else:
                robot.set_robot_turning()
        else:
            if sonar <= 1.5:
                # if there is an obstacle close to the robot, start turning
                robot.set_robot_turning()
                sonar_turn = True

            elif angle_diff > 0.4:
                # if the angle difference is too high, start turning
                robot.set_robot_turning()
                sonar_turn = False
            else:
                robot.set_robot_moving()

        # Finishing by publishing the desired speed. DO NOT TOUCH.
        rate.sleep()


if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Strategy", anonymous=True)

    run_demo()

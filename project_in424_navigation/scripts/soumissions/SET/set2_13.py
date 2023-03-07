#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Apr  3 14:27:03 2022

@author: mac20
"""


import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry


class Robot:
    def __init__(self, robot_name):
        self.sonar = 0.0  # Sonar distance
        self.robot_name = robot_name
        self.ns = "/" + self.robot_name

        self.x_pos = 0.0    # initialisation of the 
        self.y_pos = 0.0    # coordinates of the rover position 
                            # and orientation
        self.yaw_z = 0.0    # caution : in rad for the yaw

        self.x_goal = rospy.get_param("/x_goal")    # coordinates of the goal position
        self.y_goal = rospy.get_param("/y_goal")    # from the parameters

        '''Listener and publisher'''
        rospy.Subscriber(self.ns + "/sensor/sonar_front",
                         Range, self.callbacksonar)
        self.cmd_vel_pub = rospy.Publisher(
            self.ns + "/cmd_vel", Twist, queue_size=1)

        rospy.Subscriber(self.ns + "/odom", 
                         Odometry, self.callbackcompass)    # Subscriber for the position and orientation
        self.cmd_vel_pub = rospy.Publisher(
            self.ns + "/cmd_vel", Twist, queue_size=1)

    def callbacksonar(self, data):
        # DO NOT TOUCH
        self.sonar = data.range

    def callbackcompass(self, data):   # Callback function for orientation and position
        self.x_pos = data.pose.pose.position.x
        self.y_pos = data.pose.pose.position.y
        # we transform the orientation informations that we obtain
        # into roll angle, pitch angle and yaw angle, the one taht we want 
        (roll_x, pitch_y, yaw_z) = self.euler_from_quaternion(
            data.pose.pose.orientation.x, data.pose.pose.orientation.y,
            data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        self.yaw_z = yaw_z

    def get_sonar(self):
        # DO NOT TOUCH
        return float(self.sonar)

    def get_compass(self):        # get function to get the coordinates of the position 
                                  # and the orientation at any moment 
        return float(self.x_pos), float(self.y_pos), float(self.yaw_z)

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

def LOS_determination(x_pos, y_pos, x_goal, y_goal):

    # We will use the principle of Line of Sight between the robot and the goal position,
    # placed in goal-centered referential). The angle will be of zero radian when the robot while 
    # looking in the goal direction is aligned with the north. LOS angle is increasing clockwise and
    # goes from 0 to 2*pi rad (indicating the north in both extremas)

    LOS = 0
    x_rel = x_pos - x_goal     # relative position on x
    y_rel = y_pos - y_goal     # relative position on y
    if((x_rel >= 0) and (y_rel > 0)): # first quadrant 
        LOS = math.pi + math.atan(abs(x_rel) / abs(y_rel))
    if((x_rel < 0) and (y_rel >= 0)): # second quadrant 
        LOS = math.pi/2 + math.atan(abs(y_rel) / abs(x_rel))
    if((x_rel <= 0) and (y_rel < 0)): # third quadrant 
        LOS = math.atan(abs(x_rel) / abs(y_rel))
    if((x_rel > 0) and (y_rel <= 0)): # fourth quadrant 
        LOS = 3*math.pi/2 + math.atan(abs(y_rel) / abs(x_rel))
    return LOS

def run_demo():
    '''Main loop'''
    robot_name = rospy.get_param("~robot_name")
    robot = Robot(robot_name)
    print("Robot {} is starting".format(robot_name))

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        # Write your strategy here ...
        linear_velocity = 1
        angular_velocity = 0
        sonar = robot.get_sonar()
        print("SONAR VALUE : {:.2f}".format(sonar))

        # Finishing by publishing the desired speed. DO NOT TOUCH.
        robot.set_speed_angle(linear_velocity, angular_velocity)
        rate.sleep()


if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Strategy", anonymous=True)

    run_demo()

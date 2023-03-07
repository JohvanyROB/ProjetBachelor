#!/usr/bin/env python3

import math
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
import rosparam


class Robot:
    def __init__(self, robot_name):
        self.sonar = 0.0  # Sonar distance
        self.position = [0.0, 0.0]  # position in real time along the x-axis
        self.orientation = [0.0, 0.0, 0.0, 0.0]  # orientation in real time
        self.robot_name = robot_name
        self.ns = "/" + self.robot_name
        self.goal_x = rosparam.get_param("/x_goal")
        self.goal_y = rosparam.get_param("/y_goal")

        robotname = ["robot_1", "robot_2", "robot_3"]

        '''Listener and publisher'''
        for i in range(3):
            if robotname[i] == robot_name:
                rospy.Subscriber(self.ns + "/sensor/sonar_front",
                                 Range, self.callbacksonar)
                rospy.Subscriber(self.ns + "/odom",
                                 Odometry, self.callbackposition)
                rospy.Subscriber(self.ns + "/odom",
                                 Odometry, self.callbackorientation)
                self.cmd_vel_pub = rospy.Publisher(
                    self.ns + "/cmd_vel", Twist, queue_size=1)

    def callbacksonar(self, data):
        # DO NOT TOUCH
        self.sonar = data.range

    def callbackposition(self, data):
        self.position = np.array(
            [data.pose.pose.position.x, data.pose.pose.position.y])

    def callbackorientation(self, data):
        self.orientation = np.array([data.pose.pose.orientation.x, data.pose.pose.orientation.y,
                                     data.pose.pose.orientation.z, data.pose.pose.orientation.w])

    def callbackgoal(self):
        self.goal_x = rosparam.get_param("/x_goal")
        self.goal_y = rosparam.get_param("/y_goal")

    def get_goal(self):
        return(self.goal_x, self.goal_y)

    def get_sonar(self):
        return float(self.sonar)

    def get_position(self):
        return (self.position)

    def get_orientation(self):
        return (self.orientation)

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

    def distance2goal(self):  # distance that separates the initial position to the goal
        goal2x = rosparam.get_param("/x_goal")
        goal2y = rosparam.get_param("/y_goal")
        return math.sqrt(pow((goal2x - self.position[0]), 2) +
                         pow((goal2y - self.position[1]), 2))


def run_demo():
    '''Main loop'''
    robot_name = rospy.get_param("~robot_name")
    robot = Robot(robot_name)
    print("Robot {} is starting".format(robot_name))

    goal2x = rosparam.get_param("/x_goal")
    goal2y = rosparam.get_param("/y_goal")
    print("Robot location in x-axis goal is {}".format(goal2x))
    print("Robot location in y-axis goal is {}".format(goal2y))
    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        sonar = robot.get_sonar()
        print("SONAR VALUE : {:.2f}".format(sonar))

        position = robot.get_position()
        print("Position VALUES: ", position[0], position[1])

        distance = robot.distance2goal()
        print("DISTANCE TILL GOAL: ", distance)

        inc_x = goal2x - position[0]
        inc_y = goal2y - position[1]
        angle2goal = math.atan2(inc_y, inc_x)
        orientation = robot.get_orientation()
        angle = robot.euler_from_quaternion(
            orientation[0], orientation[1], orientation[2], orientation[3])[2]
        print("diff btw 2 angles", abs(angle2goal - angle))
        # initialisation of the velocity
        linear_velocity = 2.0
        angular_velocity = 0.0

        if (abs(angle2goal - angle) > 0.5):
            linear_velocity = 0.0
            angular_velocity = 2.0
            robot.set_speed_angle(linear_velocity, angular_velocity)

        elif (float(sonar) < 3):
            linear_velocity = -0.5
            angular_velocity = 2.0
            robot.set_speed_angle(linear_velocity, angular_velocity)

        elif (abs(distance) < 1):
            linear_velocity = 0.0
            angular_velocity = 0.0

        else:
            linear_velocity = 2.0
            angular_velocity = 0.0
            robot.set_speed_angle(linear_velocity, angular_velocity)

        # Finishing by publishing the desired speed. DO NOT TOUCH.
        robot.set_speed_angle(linear_velocity, angular_velocity)
        rate.sleep()


if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Strategy", anonymous=True)

    run_demo()

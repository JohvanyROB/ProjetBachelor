#!/usr/bin/env python3
import numpy as np
import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
import time


class Robot:
    def __init__(self, robot_name):
        self.sonar = 0.0  # Sonar distance
        self.robot_name = robot_name
        self.ns = "/" + self.robot_name
        # mycode
        # defining attribute for position
        self.position = [0, 0, 0]
        self.orientation = [0, 0, 0, 0]
        self.x_goal = rospy.get_param("/x_goal")
        self.y_goal = rospy.get_param("/y_goal")
        self.rho = 0
        self.argument = 0

        '''Listener and publisher for sonar'''
        rospy.Subscriber(self.ns + "/sensor/sonar_front",
                         Range, self.callbacksonar)
        self.cmd_vel_pub = rospy.Publisher(
            self.ns + "/cmd_vel", Twist, queue_size=1)

        '''Listener for odom'''
        rospy.Subscriber(self.ns + "/odom",
                         Odometry, self.callbackposition)

        rospy.Subscriber(self.ns + "/odom",
                         Odometry, self.callbackorientation)

    def callbacksonar(self, data):
        # DO NOT TOUCH
        self.sonar = data.range

    def get_sonar(self):
        # DO NOT TOUCH
        return float(self.sonar)

    def callbackposition(self, data):
        # DO NOT TOUCH
        self.position[0] = data.pose.pose.position.x
        self.position[1] = data.pose.pose.position.y
        self.position[2] = data.pose.pose.position.z

    def get_position(self):
        # DO NOT TOUCH
        return (self.position)

    def callbackorientation(self, data):
        # DO NOT TOUCH
        self.orientation[0] = data.pose.pose.orientation.x
        self.orientation[1] = data.pose.pose.orientation.y
        self.orientation[2] = data.pose.pose.orientation.z
        self.orientation[3] = data.pose.pose.orientation.w

    def get_orientation(self):
        return (self.orientation)

    def get_polarCoorGoal(self):
        return self.rho, self.argument

    def set_polarCoorGoal(self):
        self.rho = (
            (self.x_goal-self.position[0])**2 + (self.y_goal-self.position[1])**2)**0.5
        self.argument = 2*np.arctan(((self.position[1]-self.y_goal)/self.rho)/(
            1+((self.position[0]-self.x_goal)/self.rho)))
        if -180 < self.argument < 0:
            self.argument = 180 - (2*np.arctan(((self.position[1]-self.y_goal)/self.rho)/(
                1+((self.position[0]-self.x_goal)/self.rho))) * 180/np.pi)
        if 0 < self.argument < 180:
            self.argument = 180 + (abs(2*np.arctan(((self.position[1]-self.y_goal)/self.rho)/(
                1+((self.position[0]-self.x_goal)/self.rho)))) * 180/np.pi)

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
        yaw_z = math.atan2(t3, t4)*180/np.pi

        if -180 < yaw_z < 0:
            yaw_z = 180 + (180-abs(yaw_z))

        return roll_x, pitch_y, yaw_z  # in radian


def printData(robot, x_goal, y_goal, sonar, myZ, rho):

    print(":::::::::goal:::::::::")
    print("x_goal : {:.2f}".format(x_goal))
    print("y_goal : {:.2f}".format(y_goal))
    print("rho : {:.2f}".format(robot.get_polarCoorGoal()[0]))
    print("argument : {:.2f}".format(robot.get_polarCoorGoal()[1]))
    print("---------------My Robot--------------------------")
    print(":::::::::orientation:::::::::")
    print("ORIENTATION VALUE Z : {:.2f}".format(myZ))
    print(":::::::::SONAR:::::::::")
    print("SONAR VALUE : {:.2f}".format(sonar))
    print("rho VALUE : {:.2f}".format(rho))
    print("-----------------------------------------")


def capVerif(argument, myZ):
    sensi = 5
    if argument - sensi < myZ < argument + sensi:
        cap = True
    else:
        cap = False

    difference = argument-myZ

    if difference < 0:
        if difference > - 180:
            turnRight = True
        else:
            turnRight = False
    else:
        if difference > 180:
            turnRight = True
        else:
            turnRight = False

    return cap, turnRight


def findMycap(robot, turnRight, rho):

    if turnRight is True:
        if rho < 20:
            robot.set_speed_angle(0, - 1)
        else:
            robot.set_speed_angle(0, - 0.1)
    else:
        if rho < 20:
            robot.set_speed_angle(0,1)
        else:
            robot.set_speed_angle(0, 0.1)


def eskiv(robot):

    robot.set_speed_angle(1, 0.5)
    time.sleep(2.0)
    robot.set_speed_angle(2, 0)
    time.sleep(3.0)


def run_demo():
    '''Main loop'''
    robot_name = rospy.get_param("~robot_name")
    robot = Robot(robot_name)
    print("Robot {} is starting".format(robot_name))

    rate = rospy.Rate(0.5)

    while not rospy.is_shutdown():
        # Write your strategy here ...
        time.sleep(2.0)
        sonar = robot.get_sonar()
        myOrientaion = robot.get_orientation()
        convertOrientation = robot.euler_from_quaternion(
            myOrientaion[0], myOrientaion[1], myOrientaion[2], myOrientaion[3])
        myZ = convertOrientation[2]
        robot.set_polarCoorGoal()
        rho, myArgument = robot.get_polarCoorGoal()
        printData(robot, robot.x_goal, robot.y_goal, sonar, myZ, rho)
        cap, turnRight = capVerif(myArgument, myZ)

        if cap is False and sonar == 5.0:
            #print("cap  faux sonnar 5")
            findMycap(robot, turnRight, rho)
        elif cap is True and sonar != 5.0:
            #print("cap  vrai sonnar pas 5")
            eskiv(robot)
            findMycap(robot, turnRight, rho)
        elif cap is False and sonar != 5.0:
            #print("cap  faux sonnar pas 5")
            eskiv(robot)
            findMycap(robot, turnRight, rho)
        elif rho < 1:
            robot.set_speed_angle(0, 0)
            print("Bravo vous etes arrivé")
            break
        else:
            #print(" tout droit")
            robot.set_speed_angle(2, 0)

        # Finishing by publishing the desired speed. DO NOT TOUCH.
        rate.sleep()


if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Strategy", anonymous=True)

    run_demo()
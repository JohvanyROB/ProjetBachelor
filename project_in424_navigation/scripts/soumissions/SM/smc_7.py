#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
import numpy as np
import time
from nav_msgs.msg import Odometry


state = 0
avoid_state = 0
t0, t1 = 0.0, 0.0

linear_velocity, angular_velocity = 0, 0
#last_error = 0
m_angle = 0




class Robot:
    def __init__(self, robot_name):
        self.sonar = 0.0  # Sonar distance

        # partie odom
        self.odomx = 0.0  # coordonnée x
        self.odomy = 0.0  # coordonnée y
        self.odomz = 0.0  # coordonnée z
        self.odom = [self.odomx, self.odomy, self.odomz]  # coordonnées x,y,z
        self.goalx = float(rospy.get_param("/x_goal"))  # coordonnée x arrivée
        self.goaly = float(rospy.get_param("/y_goal"))  # coordonnée y arrivée
        self.yaw = 0.0

        self.robot_name = robot_name
        self.ns = "/" + self.robot_name

        '''Listener and publisher'''
        rospy.Subscriber(self.ns + "/sensor/sonar_front",
                         Range, self.callbacksonar)
        self.cmd_vel_pub = rospy.Publisher(
            self.ns + "/cmd_vel", Twist, queue_size=1)

        # partie odom
        rospy.Subscriber(self.ns + "/odom", Odometry, self.callbackodom)

    def callbacksonar(self, data):
        # DO NOT TOUCH
        self.sonar = data.range

    def get_sonar(self):
        # DO NOT TOUCH
        return float(self.sonar)

    # partie odom
    def callbackodom(self, data):
        self.odomx = data.pose.pose.position.x
        self.odomy = data.pose.pose.position.y
        self.odomz = data.pose.pose.position.z
        _, _, self.yaw = self.euler_from_quaternion(data.pose.pose.orientation.x, data.pose.pose.orientation.y,
                                                    data.pose.pose.orientation.z, data.pose.pose.orientation.w)  # récup l'orientation après conversion d'angle

    def get_odomx(self):
        return float(self.odomx)

    def get_odomy(self):
        return float(self.odomy)

    def get_odomz(self):
        return float(self.odomz)

    def get_yaw(self):
        return float(self.yaw)

    #

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


def run_demo():
    '''Main loop'''
    robot_name = rospy.get_param("~robot_name")
    robot = Robot(robot_name)
    print("Robot {} is starting".format(robot_name))

    rate = rospy.Rate(2)
    rate.sleep()
    while not rospy.is_shutdown():
        # Write your strategy here ...
        print('x:', robot.get_odomx(), 'y:',
              robot.get_odomy(), 'yaw', robot.get_yaw())
        if state == 0:
            init(robot)
        if state == 1:
            print("######head_2_goal######")
            head_2_goal(robot)
        if state == 2:
            print("######avoid_obstacles######")
            avoid_obstacles(robot)

        print(check_goal(robot))
        # Finishing by publishing the desired speed. DO NOT TOUCH.
        robot.set_speed_angle(linear_velocity, angular_velocity)
        rate.sleep()


def avoid_obstacles(robot):
    global avoid_state
    global state
    global linear_velocity, angular_velocity
    global t0, t1
    if avoid_state == 0:
        t0 = time.time()
        turn_until_no_obstacle(robot)

    if avoid_state == 1:
        t1 = time.time()
        print('time till continue', t1-t0)
        if t1-t0 < 1 and robot.get_sonar() > 4.5:
            linear_velocity = 2
            angular_velocity = 0

        if robot.get_sonar() < 4.5:
            avoid_state = 0

        else:
            state = 0
            avoid_state = 0
    return


def turn_until_no_obstacle(robot):
    global avoid_state
    global linear_velocity, angular_velocity
    print('Turn around !')
    if robot.get_sonar() < 4.5:
        linear_velocity = 0.0
        angular_velocity = -1.0
    else:
        avoid_state = 1
    return


def head_2_goal(robot):
    global state
    global linear_velocity, angular_velocity
    # global last_error

    if (check_goal(robot) == False):
        print('sonar:', robot.get_sonar())
        if (robot.get_sonar() > 4.5):
            yaw = robot.get_yaw()
            linear_velocity = 2

            error = m_angle-yaw
            print('error', error)
            # deritative = error-last_error
            # print(deritative)

            Kp = 2
            PID = Kp*error  # +Kd*deritative

            angular_velocity = 0+PID
        else:
            state = 2
        # last_error = error
    else:
        print("The robot has arrived at destination")
        linear_velocity, angular_velocity = 0, 0
        state = -1
    return


def init(robot):
    global state
    global linear_velocity, angular_velocity
    global m_angle
    #linear_velocity, angular_velocity = 0, 0
    if robot.get_sonar() != 0:
        x_robot = robot.get_odomx()
        y_robot = robot.get_odomy()

        x_goal = robot.goalx
        y_goal = robot.goaly

        y = y_goal-y_robot
        x = x_goal-x_robot
        m_angle = math.atan2(y, x)
        print("m_angle: ", m_angle)
        state = 1
    return


def check_goal(robot):
    x_robot = robot.get_odomx()
    y_robot = robot.get_odomy()

    x_goal = robot.goalx
    y_goal = robot.goaly
    return (np.sqrt((x_goal-x_robot)**2+(y_goal-y_robot)**2) < 2)


if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Strategy", anonymous=True)
    run_demo()

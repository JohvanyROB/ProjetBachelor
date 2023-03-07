#!/usr/bin/env python3
import math
import time
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry

# Global variables
angle = 0

linear_velocity = 0
angular_velocity = 0

mode = 0
flag = 0

t_start = 0
t_stop = 0

obstacles_counter = 0
val_time_secu = 4

class Robot:
    def __init__(self, robot_name):
        self.sonar = 0.0  # Sonar distance
        self.odomx = 0.0
        self.odomy = 0.0
        self.odomz = 0.0
        self.yaw = 0.0
        self.odom = [self.odomx, self.odomy, self.odomz]

        self.robot_name = robot_name
        self.goalx = float(rospy.get_param("/x_goal"))
        self.goaly = float(rospy.get_param("/y_goal"))
        self.ns = "/" + self.robot_name

        '''Listener and publisher'''
        rospy.Subscriber(self.ns + "/sensor/sonar_front",
                         Range, self.callbacksonar)
        self.cmd_vel_pub = rospy.Publisher(
            self.ns + "/cmd_vel", Twist, queue_size=1)

        '''Listener and publisher'''
        rospy.Subscriber(self.ns + "/odom", Odometry, self.callbackodom)
        self.odom_pub = rospy.Publisher(
            self.ns + "/odom", Odometry, queue_size=1)

    def callbacksonar(self, data):
        # DO NOT TOUCH
        self.sonar = data.range

    def callbackodom(self, data):
        self.odomx = data.pose.pose.position.x
        self.odomy = data.pose.pose.position.y
        self.odomz = data.pose.pose.position.z
        _, _, self.yaw = self.euler_from_quaternion(
            data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)

    def get_sonar(self):
        # DO NOT TOUCH
        return float(self.sonar)

    def get_odomx(self):
        return float(self.odomx)

    def get_odomy(self):
        return float(self.odomy)

    def get_odomz(self):
        return float(self.odomz)

    def get_yaw(self):
        return float(self.yaw)

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

    global linear_velocity, angular_velocity

    while not rospy.is_shutdown():

        print('X:', robot.get_odomx(), 'Y:', robot.get_odomy())

        if (mode == 0):
            init(robot)

        if (mode == 1):
            go_to_goal(robot)

        if (mode == 2):
            avoid(robot)

        # Finishing by publishing the desired speed.
        robot.set_speed_angle(linear_velocity, angular_velocity)
        rate.sleep()


def init(robot):  # mode=0

    global angle, mode, linear_velocity, angular_velocity

    linear_velocity = 0
    angular_velocity = 0

    odomx = robot.get_odomx()
    odomy = robot.get_odomy()

    if (odomx != 0 and odomy != 0):

        # collecting position data
        odomx = robot.get_odomx()
        odomy = robot.get_odomy()
        goalx = robot.goalx
        goaly = robot.goaly

        # calculate the angle to follow to get to the goal
        angle = math.atan2(goaly-odomy, goalx-odomx)

        # Once the angle is calculated, we move to the next state (1) = go to the goal
        mode = 1

    return


def go_to_goal(robot):  # mode=1

    global linear_velocity, angular_velocity, mode

    # checking if the robot has arrived at destination
    if (at_goal(robot) == False):

        if (robot.get_sonar() > 4.5):

            linear_velocity = 1.5  # go forward

            # calculate the gap between angle and yaw = the cap
            yaw = robot.get_yaw()
            error = angle-yaw

            # Adjusting angular velocity
            P = 1.5  # proportional coefficient
            angular_velocity = P*error
        else:

            mode = 2  # else: use the "avoid obstacles" mode

    else:
        # when the robot arrive at the goal, it stops
        linear_velocity = 0
        angular_velocity = 0
        mode = -1

    return


def at_goal(robot):

    robotx = robot.get_odomx()
    roboty = robot.get_odomy()
    goal_x = robot.goalx
    goal_y = robot.goaly

    # distance between the robot and the goal
    condition = np.sqrt((goal_x-robotx)**2+(goal_y-roboty)**2)

    return (condition < 2)  # comparison with a threshold


def avoid(robot):  # mode=2

    global flag, linear_velocity, angular_velocity, mode, t_start, t_stop, val_temps_secu

    # when the robot arrives at the goal, the obstacles' avoidance is desactivated (in case of several robots, so that all robots gets to the goal without turning around because of the others)
    if (at_goal(robot) == True):

        if(robot.get_sonar < 4.5):  # even if obstacle/other robot is detected

            flag = 1  # no need to avoid it

    # Conditions for avoiding obstacles on the path
    if (flag == 0):  # the robot sees an obstacle

        t_start = time.time()  # get the time
        turn(robot)  # apply the suitable function

    if (flag == 1):  # no more obstacle to avoid

        t_stop = time.time()  # get the time after 4 seconds

        if (t_stop-t_start < val_time_secu):

            linear_velocity = 1  # go forward
            angular_velocity = 0
            turn(robot)  # turns when encounters another obstacle to avoid hitting it

        # time duration reached
        else:

            mode = 0  # return to initialization state to compute "angle" again
            flag = 0

    return


def turn(robot):  # function allowing the robot to turn till it detects no more obstacles thanks to a security time and a counter

    global linear_velocity, angular_velocity, flag, obstacles_counter, val_time_secu

    odomx = robot.get_odomx()
    odomy = robot.get_odomy()
    goalx = robot.goalx
    goaly = robot.goaly

    # If there is an obstacle:
    if (robot.get_sonar() < 4.5):
        linear_velocity = 0
        angular_velocity = -1  # turn right
        val_time_secu = 4

        # if the robot encounters 2 obstacles one near the other (or more):
        # it turns et redefine the time duration of avoidance
        if (obstacles_counter >= 2):
            linear_velocity = 0
            angular_velocity = 1  # turn left

            # increase the time duration
            val_time_secu = 5.5

        obstacles_counter += 1
        print("##Obstacles counter## : ", obstacles_counter)

    else:
        flag = 1  # no more obstacle to avoid
        obstacles_counter = 0  # reset to 0

    return


if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Strategy", anonymous=True)

    run_demo()

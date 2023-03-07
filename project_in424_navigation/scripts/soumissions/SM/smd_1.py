#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
import time
import numpy as np


class Robot:
    def __init__(self, robot_name):
        self.sonar = 0.0  # Sonar distance

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.alpha = 0.0
        self.beta = 0.0
        self.gamma = 0.0
        self.omega = 0.0

        self.x_goal = rospy.get_param("/x_goal")
        self.y_goal = rospy.get_param("/y_goal")

        self.delta_x = 0.0
        self.delta_y = 0.0

        self.robot_name = robot_name
        self.ns = "/" + self.robot_name

        '''Listener and publisher'''
        rospy.Subscriber(self.ns + "/sensor/sonar_front",
                         Range, self.callbacksonar)
        rospy.Subscriber(self.ns + "/odom",
                         Odometry, self.callbackpos)

        self.cmd_vel_pub = rospy.Publisher(
            self.ns + "/cmd_vel", Twist, queue_size=1)

    def callbacksonar(self, data):
        # DO NOT TOUCH
        self.sonar = data.range

    def callbackpos(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.z = data.pose.pose.position.z
        self.alpha = data.pose.pose.orientation.x
        self.beta = data.pose.pose.orientation.y
        self.gamma = data.pose.pose.orientation.z
        self.omega = data.pose.pose.orientation.w

    def get_sonar(self):
        # DO NOT TOUCH
        return float(self.sonar)

    def get_pos(self):
        return (self.x,
                self.y,
                self.z,
                self.alpha,
                self.beta,
                self.gamma,
                self.omega
                )

    def get_goal(self):
        return (self.x_goal,
                self.y_goal)

    def set_goal_angle(self):

        self.delta_x = self.x_goal - self.x
        self.delta_y = self.y_goal - self.y
        hyp_angle_inf = math.atan(abs(self.delta_x/self.delta_y))
        hyp_angle_sup = math.atan(abs(self.delta_y/self.delta_x))
        goal_angle_inf = hyp_angle_inf - math.pi/2
        goal_angle_sup = hyp_angle_sup
        # un angle 180 - hyp angle pour en haut a gauche
        # un angle - 180 - hyp angle pour en bas a gauche

        hyp_dist = np.sqrt(self.delta_x**2 + self.delta_y**2)

        return goal_angle_inf, goal_angle_sup, hyp_dist

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

    # def ratio_w(self, hyp_dist):
        # angular_velocity = 1/hyp_dist
        # return angular_velocity


def run_demo():
    # This function will be called when the code is run.
    '''Main loop'''
    robot_name = rospy.get_param("~robot_name")
    # This class is used to represent a robot in ROS.
    robot = Robot(robot_name)
    print("Robot {} is starting".format(robot_name))

    # Setting up a rate object. This object is used to control the rate at which the code runs.
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():

        # Our strategy :
        """This loop will run until the ROS system is shut down.
             Inside the while loop, the code gets the robot's current position, goal, and sonar readings.
             Next, the code sets up some variables that will be used to control the robot's speed and direction.
             The linear_velocity variable is used to control the robot's forward speed, and the angular_velocity variable is used to control the robot's turning speed.
             After that, the code gets the robot's current yaw angle.
             This angle is used to determine how fast the robot should turn.
             Next, the code sets up some variables that will be used to control the robot's speed and direction.
             The linear_velocity variable is used to control the robot's forward speed, and the angular_velocity variable is used to control the robot's turning speed.
             After that, the code gets the robot's current yaw angle. This angle is used to determine how fast the robot should turn.
             Next, the code checks if the robot is close to its goal.
             If the robot is close to its goal, the code sets the linear_velocity and angular_velocity variables to 0 so that the robot will stop.
             If the robot is not close to its goal, the code checks if the sonar reading is less than 3.
             If the sonar reading is less than 3, this means that there is an obstacle in front of the robot.
             To avoid the obstacle, the code sets the linear_velocity variable to 0.2. This will cause the robot to slow down.
             Next, the code sets the angular_velocity variable to a value that will cause the robot to turn.
             The value is based on the robot's current yaw angle and the goal angle.
             If the goal angle is positive, the code will set the angular_velocity variable to 0.4. This will cause the robot to turn to the right.
             If the goal angle is negative, the code will set the angular_velocity variable to -0.4. This will cause the robot to turn to the left.
             After that, the code publishes the linear_velocity and angular_velocity variables to the ROS system. This will cause the robot to move.
             Finally, the code sleeps for 0.5 seconds and then repeats.
         """
        linear_velocity = 2
        angular_velocity = 0.0
        current_yaw = 0.0

        pos_x, pos_y, pos_z, pos_alpha, pos_beta, pos_gamma, pos_omega = robot.get_pos()
        x_goal, y_goal = robot.get_goal()
        goal_angle_inf, goal_angle_sup, hyp_dist = robot.set_goal_angle()
        sonar = robot.get_sonar()

        # Printing every useful data about the position and computations done about the robot
        print("SONAR VALUE : {:.2f}".format(sonar))
        print("POSITION X: {:.2f}".format(pos_x))
        print("POSITION Y: {:.2f}".format(pos_y))
        # print("POSITION Z: {:.2f}".format(pos_z))
        # print("POSITION ALPHA: {:.2f}".format(pos_alpha))
        # print("POSITION BETA: {:.2f}".format(pos_beta))
        print("POSITION GAMMA: {:.2f}".format(pos_gamma))
        print("POSITION_OMEGA: {:.2f}".format(pos_omega))
        print("X GOAL :{:.2f}".format(x_goal))
        print("Y GOAL :{:.2f}".format(y_goal))
        print("HYP ANGLE:{:.2f}".format(goal_angle_inf))
        print("HYP DIST:{:.2f}".format(hyp_dist))

        # Finishing by publishing the desired speed. DO NOT TOUCH.

        # converting quaternions to angles with euler
        _, _, current_yaw = robot.euler_from_quaternion(
            pos_alpha, pos_beta, pos_gamma, pos_omega)

        # These are for the error margin of the robot angle calculation
        goal_angle_inf_min = goal_angle_inf - 0.2
        goal_angle_inf_max = goal_angle_inf + 0.2
        goal_angle_sup_min = goal_angle_sup - 0.2
        goal_angle_sup_max = goal_angle_sup + 0.2

        print("CURRENT YAW:{:.2f}".format(current_yaw))

        # if the hypthenus lenght is less than 0.6m, stop the robot.
        if hyp_dist < 0.6:
            print("ARRETE TOI PTN MIMI ENCULE")
            robot.set_speed_angle(0, 0)

        # if the robot is higher than the target platform (on y axis)
        elif pos_y > y_goal:

            # if the sonar sees nothing and the robot is in the wrong direction
            if sonar >= 3 and (current_yaw <= goal_angle_inf_min or current_yaw >= goal_angle_inf_max):
                # turn the robot counter-clockwise
                if goal_angle_inf > 0:
                    angular_velocity = 0.4
                    robot.set_speed_angle(
                        linear_velocity, angular_velocity)
                else:
                    # turn the robot the other way
                    angular_velocity = -0.4
                    robot.set_speed_angle(
                        linear_velocity, angular_velocity)

            # if the sonar detect something and the current direction is the good one
            elif sonar >= 3 and (goal_angle_inf_min <= current_yaw <= goal_angle_inf_max):
                # stop turning and go forward
                angular_velocity = 0.0
                robot.set_speed_angle(linear_velocity, angular_velocity)

            # if the robotdetect something in fornt of it (3m away)
            elif sonar < 3:
                # slow down
                linear_velocity = linear_velocity * 0.02

                # making sure of where is the robot comapred to the target so it turns the right way (faster)
                if goal_angle_inf > 0:
                    # angular_velocity = -ratio_w(hyp_dist)
                    angular_velocity = (angular_velocity + 0.25) % 2
                else:
                    angular_velocity = (angular_velocity - 0.25) % 2
                    # angular_velocity = ratio_w(hyp_dist)

                robot.set_speed_angle(linear_velocity, angular_velocity)
                time.sleep(0.8)
                linear_velocity = 2
                angular_velocity = 0.0
                robot.set_speed_angle(linear_velocity, angular_velocity)

        # same as earlier code, but the other way
        elif pos_y <= y_goal:

            if sonar >= 3 and (current_yaw <= goal_angle_sup_min or current_yaw >= goal_angle_sup_max):
                if goal_angle_sup > 0:
                    angular_velocity = 0.4
                    robot.set_speed_angle(
                        linear_velocity, angular_velocity)
                else:
                    angular_velocity = -0.4
                    robot.set_speed_angle(
                        linear_velocity, angular_velocity)

            elif sonar >= 3 and (goal_angle_sup_min <= current_yaw <= goal_angle_sup_max):
                angular_velocity = 0.0
                robot.set_speed_angle(linear_velocity, angular_velocity)

            elif sonar < 3:
                linear_velocity = linear_velocity * 0.5

                if goal_angle_sup > 0:
                    # angular_velocity = -ratio_w(hyp_dist)
                    angular_velocity = (angular_velocity + 0.25) % 2
                else:
                    angular_velocity = (angular_velocity - 0.25) % 2
                    # angular_velocity = ratio_w(hyp_dist)

                robot.set_speed_angle(linear_velocity, angular_velocity)
                time.sleep(0.8)
                linear_velocity = 2
                angular_velocity = 0.0
                robot.set_speed_angle(linear_velocity, angular_velocity)

        rate.sleep()


if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Strategy", anonymous=True)

    run_demo()

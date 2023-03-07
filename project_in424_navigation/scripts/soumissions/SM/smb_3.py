#!/usr/bin/env python3

import math
import rospy
import time
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry


class Robot:
    def __init__(self, robot_name):
        self.sonar = 0.0  # Sonar distance
        self.robot_name = robot_name
        self.ns = "/" + self.robot_name

        self.posx = 0.0
        self.posy = 0.0

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.xpos_goal = rospy.get_param("/x_goal")
        self.ypos_goal = rospy.get_param("/y_goal")

        '''Listener and publisher'''
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
        self.posx = data.pose.pose.position.x
        self.posy = data.pose.pose.position.y

    def callbackorientation(self, data):
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
        a = data.pose.pose.orientation.x
        b = data.pose.pose.orientation.y
        c = data.pose.pose.orientation.z
        d = data.pose.pose.orientation.w

        self.roll, self.pitch, self.yaw = euler_from_quaternion(
            self, a, b, c, d)

        return

    def get_sonar(self):
        # DO NOT TOUCH
        return float(self.sonar)

    def get_position(self):
        return float(self.posx), float(self.posy)

    def get_orientation(self):
        return float(self.roll), float(self.pitch), float(self.yaw)

    def get_goal(self):
        return float(self.xpos_goal), float(self.ypos_goal)

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


def run_demo():
    def turn_left():
        lin_vel = 5
        ang_vel = 0.5
        ang_vel = 0.5
        return lin_vel, ang_vel

    def turn_right():
        lin_vel = 5
        ang_vel = -0.5
        return lin_vel, ang_vel

    def manoeuvre_evitement():
        lin_vel = 5
        ang_vel = 0.5
        return lin_vel, ang_vel

    def normal_run():
        linear_velocity = 10
        angular_velocity = 0
        return linear_velocity, angular_velocity

    def stop():
        linear_velocity = 0
        angular_velocity = 0
        return linear_velocity, angular_velocity

    def optimal_angle(x, y, x_goal, y_goal):
        Actuel = np.array([x, y])
        Arrivee = np.array([x_goal, y_goal])
        Droit = np.array([x_goal, y])
        Arrivee_Droit = math.sqrt(
            ((Arrivee[0]-Droit[0])**2)+((Arrivee[1]-Droit[1])**2))
        Actuel_Droit = math.sqrt(
            ((Actuel[0]-Droit[0])**2)+((Actuel[1]-Droit[1])**2))
        print(Arrivee_Droit, Actuel_Droit)
        teta = math.atan(Arrivee_Droit/Actuel_Droit)*(180/math.pi)
        if (y + y_goal < 0):
            teta = -teta
        return teta

    '''Main loop'''
    robot_name = rospy.get_param("~robot_name")
    robot = Robot(robot_name)
    x_goal, y_goal = robot.get_goal()
    x, y = robot.get_position()
    trigger = False

    print("Robot {} is starting".format(robot_name))
    print("Robot goal is x : "+str(x_goal)+" y : "+str(y_goal))
    print("Robot initial pose is x : "+str(x)+" y : "+str(y))

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        # Write your strategy here ...
        linear_velocity = 10
        angular_velocity = 0
        x, y = robot.get_position()
        sonar = robot.get_sonar()
        roll, pitch, yaw = robot.get_orientation()
        roll_Deg, pitch_Deg, yaw_Deg = roll * \
            (180/math.pi), pitch * (180/math.pi), yaw * (180/math.pi)

        teta = optimal_angle(x, y, x_goal, y_goal)
        if (yaw_Deg > teta):
            linear_velocity, angular_velocity = turn_right()
        else:
            linear_velocity, angular_velocity = turn_left()

        if(sonar < 5):
            linear_velocity, angular_velocity = manoeuvre_evitement()
            trigger = True

        if((x_goal-2 < x < x_goal+2) & (y_goal-2 < y < y_goal+2)):
            linear_velocity, angular_velocity = stop()

        print("SONAR VALUE : {:.2f}".format(sonar))
        print("POSITION : x = " + str(x) + "y = " + str(y))
        print("ORIENTATION : roll = " + str(roll_Deg) +
              "pitch = " + str(pitch_Deg) + "yaw = " + str(yaw_Deg))
        print("ANGLE OPTI : teta = " + str(teta))

        # Finishing by publishing the desired speed. DO NOT TOUCH.
        robot.set_speed_angle(linear_velocity, angular_velocity)
        if(trigger == True):
            trigger = False
            time.sleep(2)
        rate.sleep()


if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Strategy", anonymous=True)

    run_demo()
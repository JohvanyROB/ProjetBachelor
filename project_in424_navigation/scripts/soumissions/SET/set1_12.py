#!/usr/bin/env python3

import math
import time
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range


class Robot:
    def __init__(self, robot_name):
        self.sonar = 0.0  # Sonar distance
        self.robot_name = robot_name
        self.ns = "/" + self.robot_name
        self.xpos = 0
        self.ypos = 0
        self.xquaternion = 0
        self.yquaternion = 0
        self.zquaternion = 0
        self.wquaternion = 0

        '''Listener and publisher'''
        rospy.Subscriber(self.ns + "/sensor/sonar_front",
                         Range, self.callbacksonar)

        rospy.Subscriber(self.ns + "/odom",
                         Odometry, self.callbackposition)

        rospy.Subscriber(self.ns + "/odom",
                         Odometry, self.callbackangles)

        self.cmd_vel_pub = rospy.Publisher(
            self.ns + "/cmd_vel", Twist, queue_size=1)

    def callbacksonar(self, data):
        # DO NOT TOUCH
        self.sonar = data.range

    def callbackposition(self, data):
        self.xpos = data.pose.pose.position.x
        self.ypos = data.pose.pose.position.y

    def callbackangles(self, data):

        self.xquaternion = data.pose.pose.orientation.x
        self.yquaternion = data.pose.pose.orientation.y
        self.zquaternion = data.pose.pose.orientation.z
        self.wquaternion = data.pose.pose.orientation.w

    def get_sonar(self):
        # DO NOT TOUCH
        return float(self.sonar)

    def get_pos(self):
        # DO NOT TOUCH
        return float(self.xpos), float(self.ypos)

    def get_angles(self):
        # DO NOT TOUCH
        return float(self.xquaternion), float(self.yquaternion), float(self.zquaternion), float(self.wquaternion)

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
    while not rospy.is_shutdown():
        # Write your strategy here ...

        sonar = robot.get_sonar()

        xpos, ypos = robot.get_pos()

        xquaternion, yquaternion, zquaternion, wquaternion = robot.get_angles()
        xangle, yangle, zangle = robot.euler_from_quaternion(
            xquaternion, yquaternion, zquaternion, wquaternion)  # Conversion from quarternion to radian angles

        xgoal = rospy.get_param("/x_goal")
        ygoal = rospy.get_param("/y_goal")  # We get our goal position

        # We get our direction
        zanglegoal = math.atan2((ygoal-ypos), (xgoal-xpos))

        print("SONAR : {:.2f}".format(sonar))
        print("\n")
        print("X : {:.2f}".format(xpos))
        print("Y : {:.2f}".format(ypos))
        print("X GOAL : {:.2f}".format(xgoal))
        print("Y VOAL : {:.2f}".format(ygoal))
        print("\n")
        print("Z_ANGLE : {:.2f}".format(round(zangle, 1)))
        print("Z_ANGLE GOAL : {:.2f}".format(round(zanglegoal, 1)))
        print("\n")

        if sonar < 4:
            angular_velocity = 0.5
            linear_velocity = 1
            time.sleep(2)

        # xgoal = 13 and ygoal = -27
        elif ((xpos < xgoal + 1) and (xpos > xgoal - 2)) and ((ypos < ygoal + 2) and (ypos > ygoal - 1)):
            angular_velocity = 0
            linear_velocity = 0

        elif round(zanglegoal, 1) + 0.1 < round(zangle, 1):  # dodging monoeuvre process
            if abs(zanglegoal - zangle) < math.pi:
                angular_velocity = -0.5
                linear_velocity = 1
            else:
                angular_velocity = 0.5
                linear_velocity = 1

        elif round(zanglegoal, 1) - 0.1 > round(zangle, 1):  # dodging monoeuvre process
            if abs(zanglegoal - zangle) < math.pi:
                angular_velocity = 0.5
                linear_velocity = 1
            else:
                angular_velocity = -0.5
                linear_velocity = 1

        else:
            angular_velocity = 0
            linear_velocity = 1

        # Finishing by publishing the desired speed. DO NOT TOUCH.
        robot.set_speed_angle(linear_velocity, angular_velocity)
        rate.sleep()


if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Strategy", anonymous=True)

    run_demo()

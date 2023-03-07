#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry


class Robot:
    def __init__(self, robot_name):
        self.sonar = 0.0  # Sonar distance
        self.position = ""
        self.robot_name = robot_name
        self.ns = "/" + self.robot_name

        # Coordinates of the goal about the reference
        self.xgoal = rospy.get_param('x_goal')
        self.ygoal = rospy.get_param('y_goal')

        # Vector with quaternion componants
        self.oriQuant = [0.0, 0.0, 0.0, 0.0]  # Quaternions of the robot

        self.currentX = 0.0  # Coordinates of the robot about the reference
        self.currentY = 0.0
        self.yaw = 0.0  # Angle of the robot around z-axis

        '''Listener and publisher'''
        rospy.Subscriber(self.ns + "/sensor/sonar_front",
                         Range, self.callbacksonar)
        self.cmd_vel_pub = rospy.Publisher(
            self.ns + "/cmd_vel", Twist, queue_size=1)

        rospy.Subscriber(self.ns + "/odom",
                         Odometry, self.callbackposition)
        rospy.Subscriber(self.ns + "/odom",
                         Odometry, self.callbackorientation)
        rospy.Subscriber(self.ns + "/odom",
                         Odometry, self.callbackposXandY)

    def callbacksonar(self, data):
        # DO NOT TOUCH
        self.sonar = data.range

    def callbackposition(self, data):
        # OWN CODE
        self.position = data.pose.pose.position

    def callbackorientation(self, data):
        # OWN CODE
        self.oriQuant[0] = data.pose.pose.orientation.x
        self.oriQuant[1] = data.pose.pose.orientation.y
        self.oriQuant[2] = data.pose.pose.orientation.z
        self.oriQuant[3] = data.pose.pose.orientation.w

    def callbackposXandY(self, data):
        # OWN CODE
        self.currentX = data.pose.pose.position.x
        self.currentY = data.pose.pose.position.y

    def get_sonar(self):
        # DO NOT TOUCH
        return float(self.sonar)

    def get_position(self):
        # OWN CODE
        return str(self.position)

    def get_orientation(self):
        # OWN CODE
        return str(self.oriQuant[0]), str(self.oriQuant[1]), str(self.oriQuant[2]), str(self.oriQuant[3])

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
        #t0 = 2.0 * (w * x + y * z)
        #t1 = 1.0 - 2.0 * (x * x + y * y)
        #roll_x = math.atan2(t0, t1)

        #t2 = 2.0 * (w * y - z * x)
        #t2 = 1.0 if t2 > 1.0 else t2
        #t2 = -1.0 if t2 < -1.0 else t2
        #pitch_y = math.asin(t2)

        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return yaw_z  # in radians

    def Pivot(self, ang):
        if ang < 0.0:
            # turns clockwise with a rotation speed of 0.5 rad/s
            self.set_speed_angle(0.0, -0.5)
        elif ang > 0.0:
            # turns counterclockwise with a rotation speed of 0.5 rad/s
            self.set_speed_angle(0.0, 0.5)
        else:
            self.set_speed_angle(0.0, 0.0)
        t = rospy.Duration(abs(ang)/0.5, 0)  # time the robot needs to turn
        rospy.sleep(t)  # stops the rotation
        self.set_speed_angle(0.0, 0.0)
        return

    def Direction(self):
        dirX = self.xgoal - self.currentX  # x-coordinate of the vector robot-goal
        dirY = self.ygoal - self.currentY  # y-coordinate of the vector robot-goal
        # gets the oriented angle between the vector and the x-axis
        diryaw = math.atan2(dirY, dirX)
        vehyaw = self.euler_from_quaternion(
            self.oriQuant[0], self.oriQuant[1], self.oriQuant[2], self.oriQuant[3])
        # Give the yaw angle of the robot (in radians)
        difyaw = diryaw - vehyaw  # difference between the angles
        if abs(difyaw) > 10**(-1):
            print("Calibrage")
            self.Pivot(difyaw)  # calls the previous function
            self.yaw = diryaw  # set the new value of the yaw angle
        return

    def Approach(self):
        print("Approche")
        t1 = rospy.get_time()
        while self.sonar >= 2.0:
            # moves forward until it sees an obstacles
            self.set_speed_angle(2.0, 0.0)
            t2 = rospy.get_time()
            if t2 - t1 >= 1:
                self.set_speed_angle(0.0, 0.0)  # stops (refresh)
                break
        return

    def Manoeuvre(self):
        if self.sonar <= 3.0:
            print("Esquive")
            # turns counterclockwise with an angle of pi/2
            self.Pivot(math.pi/2)
            son1 = self.sonar
            self.Pivot(-math.pi)  # turns clockwise with an angle of pi
            son2 = self.sonar
            if son1 > son2:
                self.Pivot(math.pi)  # needs to turn back to the previous angle
                self.yaw += math.pi/2  # refreshes the new value of the yaw
            else:
                # refreshes the new value of the yaw (no need to turn back)
                self.yaw -= math.pi/2
            t1 = rospy.get_time()
            while self.sonar >= 2.0:
                # moves forward until there is an obstacle
                self.set_speed_angle(1.0, 0.0)
                t2 = rospy.get_time()
                if t2 - t1 >= 3:
                    self.set_speed_angle(0.0, 0.0)  # stops (refresh)
                    break
        return


def run_demo():
    '''Main loop'''
    robot_name = rospy.get_param("~robot_name")
    robot = Robot(robot_name)
    print("Robot {} is starting".format(robot_name))

    rate = rospy.Rate(2)

    while not rospy.is_shutdown():

        # Write your strategy here ...
        print("\n_____________________________")

        if robot.sonar != 0.0:

            robot.Direction()
            print("Yaw : {}".format(robot.yaw))
            robot.Approach()

            robot.Manoeuvre()
            print("Yaw : {}".format(robot.yaw))
            print("Current_X : {}".format(robot.currentX))
            print("Current_Y : {}".format(robot.currentY))

            if abs(robot.xgoal - robot.currentX) <= 2*10**(-0.0) and abs(robot.ygoal - robot.currentY) <= 2*10**(-0.0):
                print("GOOOOOOOOOOOAL")  # stop condition
                return

        # Finishing by publishing the desired speed. DO NOT TOUCH.
        # robot.set_speed_angle(linear_velocity, angular_velocity)
        rate.sleep()


if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Strategy", anonymous=True)

    run_demo()

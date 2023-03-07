#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry


class Robot:
    def __init__(self, robot_name):
        self.sonar = 5.0  # Sonar distance
        self.odom = [0, 0, 0]  # localisation of the robot
        self.orient = 0  # orientation of the robot
        self.robot_name = robot_name
        self.ns = "/" + self.robot_name
        # localisation of the goal (x)
        self.x_goal = rospy.get_param("/x_goal")
        # localisation of the goal (y)
        self.y_goal = rospy.get_param("/y_goal")

        '''Listener and publisher'''
        rospy.Subscriber(self.ns + "/sensor/sonar_front",
                         Range, self.callbacksonar)
        rospy.Subscriber(self.ns + "/odom", Odometry, self.callbackodom)
        self.cmd_vel_pub = rospy.Publisher(
            self.ns + "/cmd_vel", Twist, queue_size=1)

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

    def callbacksonar(self, data):
        # DO NOT TOUCH
        self.sonar = data.range

    def callbackodom(self, data):
        # DO NOT TOUCH
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        z = data.pose.pose.position.z
        self.odom = [x, y, z]
        x = data.pose.pose.orientation.x
        y = data.pose.pose.orientation.y
        z = data.pose.pose.orientation.z
        w = data.pose.pose.orientation.w
        # we have used part of the euler fonction to build
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)  # this part of our fonction
        yaw_z = math.atan2(t3, t4)
        self.orient = yaw_z  # it return the yaw

    def get_sonar(self):
        # DO NOT TOUCH
        return float(self.sonar)

    def get_xgoal(self):
        # DO NOT TOUCH
        return float(self.x_goal)

    def get_ygoal(self):
        # DO NOT TOUCH
        return float(self.y_goal)

    def get_odom(self):
        # DO NOT TOUCH
        return self.odom

    def get_orient(self):
        # DO NOT TOUCH
        return self.orient

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
    '''Main loop'''
    robot_name = rospy.get_param("~robot_name")
    robot = Robot(robot_name)
    print("Robot {} is starting".format(robot_name))
    xg = robot.get_xgoal()
    yg = robot.get_ygoal()  # we initialise the coornitate of the reach point
    rate = rospy.Rate(2)
    i = 0
    while not rospy.is_shutdown():
        # Write your strategy here ...
        linear_velocity = 0.5
        angular_velocity = 0
        sonar = robot.get_sonar()  # we get if there is something in front of the robot
        odom = robot.get_odom()  # we get the coordinate of the robot
        orient = robot.get_orient()  # we get the orientation of the robot
        # we calculate the angle to go to the goal
        teta = math.atan2(yg-(odom[1]), (xg-odom[0]))

        # we chech if the robot is on the objetif
        if(abs(odom[0]-xg) < 2 and abs(odom[1]-yg) < 2):
            angular_velocity = 0
            linear_velocity = 0
            robot.set_speed_angle(linear_velocity, angular_velocity)
        elif (sonar < 3.0):  # we check if there is an obstacle in front of the robot
            angular_velocity = 2
            linear_velocity = 0
            robot.set_speed_angle(linear_velocity, angular_velocity)
            i = 1
        elif(i == 1):  # if the previous step was to reorient the robot because of an obstacle
            for k in range(5):  # then the following step is this one and it have to move the robot
                angular_velocity = 0  # forward for a short time to avoid completety the obstacle
                linear_velocity = 2
                robot.set_speed_angle(linear_velocity, angular_velocity)
            i = 0
        elif (abs(teta-orient) > 0.5):  # this step is here to steer the robot towards the objective
            linear_velocity = 0
            angular_velocity = -0.5
            robot.set_speed_angle(linear_velocity, angular_velocity)
        else:  # this step is here to move forward the robot
            linear_velocity = 2
            angular_velocity = 0
            robot.set_speed_angle(linear_velocity, angular_velocity)
        robot.set_speed_angle(linear_velocity, angular_velocity)
        rate.sleep()


if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Strategy", anonymous=True)

    run_demo()

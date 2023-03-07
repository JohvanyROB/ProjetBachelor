#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry


class Robot:
    def __init__(self, robot_name):
        self.sonar = 0.0  # Sonar distance
        self.odom_x = 0.0  # odometry of the robot
        self.odom_y = 0.0  # odometry of the robot
        self.robot_name = robot_name
        self.goal_x = rospy.get_param("/x_goal")
        self.goal_y = rospy.get_param("/y_goal")

        self.ns = "/" + self.robot_name

        '''Listener and publisher'''
        rospy.Subscriber(self.ns + "/sensor/sonar_front",
                         Range, self.callbacksonar) #Subscribe for the sensor
        rospy.Subscriber(self.ns + "/odom",
                         Odometry, self.callbackodom)#Subscribe for the robot coordinates
        self.cmd_vel_pub = rospy.Publisher(
            self.ns + "/cmd_vel", Twist, queue_size=1) #Gets the info in the rospy for the goal coordinates

    def callbacksonar(self, data):
        # DO NOT TOUCH
        self.sonar = data.range

    def callbackodom(self, data):
        # DO NOT TOUCH
        self.odom_x = data.pose.pose.position.x
        self.odom_y = data.pose.pose.position.y

    def get_sonar(self):
        # DO NOT TOUCH
        return float(self.sonar)

    def get_odom_x(self): #Get the x position of the robot
        # DO NOT TOUCH
        return float(self.odom_x)

    def get_odom_y(self): #Get the y position of the robot
        # DO NOT TOUCH
        return float(self.odom_y)

    def get_goal_y(self): #Y coordinates of the goal
        # DO NOT TOUCH
        return float(self.goal_y)

    def get_goal_x(self):#X coordinates of the goal
        # DO NOT TOUCH
        return float(self.goal_x)

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
        linear_velocity = 1
        angular_velocity = 0.0
        sonar = robot.get_sonar()
        #Here we get the position of the robot
        odom_x = robot.get_odom_x()
        odom_y = robot.get_odom_y()
        goal_x = robot.get_goal_x()
        goal_y = robot.get_goal_y()
        goal_distance_x = (goal_x)**2 - (odom_x)**2 #Distance to the x coordinates of the goal
        goal_distance_y = (goal_y)**2 - (odom_y)**2 #Distance to the y coordinates of the goal
        degrees_temp = math.atan2(
            goal_distance_x, goal_distance_y)/(math.pi*180) #Here we try to obtain the angle between the robot and the goal
       
        while goal_distance_x != 0 and goal_distance_y != 0:
        #While the robot isn't exactly on the coordinates of the goal, it keeps moving
            linear_velocity += 3
            angular_velocity += 2
        if sonar != 5: #An obstacle is reached, so the robot has to move around
            linear_velocity += 1.5
            angular_velocity -= 2
        if sonar == 5:
            linear_velocity = 3
            angular_velocity = 0

        print("GOAL DISTANCE X :{:.2f}".format(goal_distance_x))
        '''
        print("SONAR VALUE : {:.2f}".format(sonar))
        print("ODOMETRY X VALUE : {:.2f}".format(odom_x))
        print("ODOMETRY Y VALUE : {:.2f}".format(odom_y))
        '''
        # Finishing by publishing the desired speed. DO NOT TOUCH.
        robot.set_speed_angle(linear_velocity, angular_velocity)
        rate.sleep()


if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Strategy", anonymous=True)

    run_demo()

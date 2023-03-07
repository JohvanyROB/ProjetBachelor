#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry


class Robot:
    def __init__(self, robot_name):
        self.position = []
        self.orientation = []
        self.sonar = 0.0  # Sonar distance
        self.robot_name = robot_name
        self.ns = "/" + self.robot_name

        '''Listener and publisher'''
        rospy.Subscriber(self.ns + "/sensor/sonar_front",
                         Range, self.callbacksonar)
        self.cmd_vel_pub = rospy.Publisher(
            self.ns + "/cmd_vel", Twist, queue_size=1)

        rospy.Subscriber(self.ns + "/odom", Odometry, self.callbackposition)

    def callbackposition(self, data):
        self.position = [data.pose.pose.position.x,
                         data.pose.pose.position.y, data.pose.pose.position.z]
        self.orientation = [data.pose.pose.orientation.x,
                            data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]

    def get_position(self):
        try:
            self.position[0]
            return self.position
        except IndexError:
            print("index error")
            return [0, 0, 0]

    def get_orientation(self):
        try:
            x, y, z = self.euler_from_quaternion(
                self.orientation[0], self.orientation[1], self.orientation[2], self.orientation[3])
            return z * 180/math.pi

        except IndexError:
            print("index error")
            return 0

    def callbacksonar(self, data):
        # DO NOT TOUCH
        self.sonar = data.range

    def get_sonar(self):
        # DO NOT TOUCH
        return float(self.sonar)

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

        # print(yaw_z)

        return roll_x, pitch_y, yaw_z  # in radians


def print_all(robot):
    sonar = robot.get_sonar()
    position = robot.get_position()
    orientation = robot.get_orientation()
    print("SONAR VALUE : {:.2f}".format(sonar))
    print(position)
    print(orientation)


def turn(robot, rate, angle):

    if angle > 180:
        angle -= 360
    elif angle < -180:
        angle += 360
    else:
        angle = angle

    orientation = robot.get_orientation()
    #print(orientation, 5)
    while((orientation) > (angle)+0.1 or (orientation) < (angle)-0.1):
        print("turning")
        orientation = robot.get_orientation()
        print_all(robot)
        robot.set_speed_angle(0, (angle-orientation)/50)
        rate.sleep()
    robot.set_speed_angle(0, 0)
    rate.sleep()


def avancer(robot, rate, distance):
    for i in range(distance):
        sonar = robot.get_sonar()
        if sonar >= 2:
            robot.set_speed_angle(1, 0)
            rate.sleep()
        else:
            robot.set_speed_angle(0, 0)
            rate.sleep()
            break


def avoidance_system(robot, rate):
    orientation = robot.get_orientation()
    turn(robot, rate, orientation+45)
    sonar_left = robot.get_sonar()
    turn(robot, rate, orientation-45)
    sonar_right = robot.get_sonar()

    if sonar_right < 2 and sonar_left < 2:
        robot.set_speed_angle(-1, 0)
        rate.sleep()
        avoidance_system(robot, rate)
    else:
        if sonar_left > sonar_right:
            turn(robot, rate, orientation+45)
        else:
            turn(robot, rate, orientation-45)
        sonar = robot.get_sonar()
        if sonar <= 2:
            avoidance_system(robot, rate)
        else:
            avancer(robot, rate, 10)


def run_demo():
    '''Main loop'''
    robot_name = rospy.get_param("~robot_name")
    robot = Robot(robot_name)
    print("Robot {} is starting".format(robot_name))
    rate = rospy.Rate(2)

    goal = [rospy.get_param("/x_goal"), rospy.get_param("/y_goal")]

    while not rospy.is_shutdown():

        # print(goal)
        # Write your strategy here ...

        linear_velocity = 0
        angular_velocity = 0

        position = robot.get_position()

        while(not((goal[1] - 1 < position[1] < goal[1] + 1) and (goal[0] - 1 < position[0] < goal[0] + 1))):

            print("Robot trying to go to goal {} and is at {} position".format(
                goal, position))

            position = robot.get_position()
            angle = math.atan2(goal[1]-position[1],
                               goal[0]-position[0]) * 180/math.pi

            orientation = robot.get_orientation()
            if (orientation > angle + 0.5 or orientation < angle - 0.5):
                turn(robot, rate, angle)

            sonar = robot.get_sonar()

            print(orientation, angle, sonar, goal, position)

            if sonar >= 2:
                avancer(robot, rate, 1)
            else:
                print("Obstacle, je stop")
                linear_velocity = 0
                angular_velocity = 0
                robot.set_speed_angle(linear_velocity, angular_velocity)
                avoidance_system(robot, rate)
                rate.sleep()

        print("Je suis arrivé à destination")
        robot.set_speed_angle(0, 0)
        rate.sleep()
        break


if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Strategy", anonymous=True)

    run_demo()
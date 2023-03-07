#!/usr/bin/env python3

import time
import random
import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry

sonar_threshold = 3
angle_threshold = 2*math.pi/180
goal_threshold = 2

yaw_error = 0

# For avoidance strategy
t0 = 0  # Timer
phase = 0  # Obstacle avoidance phase
yaw_goal = 0    # yaw goal for avoidance strategy

class Robot:
    def __init__(self, robot_name):
        # Sonar
        self.sonar = 0.0  # Sonar distance

        # Robot id
        self.robot_name = robot_name
        self.ns = "/" + self.robot_name

        # Position and orientation
        self.pos = (0, 0)
        self.yaw = 0  # (°)

        # Goal
        self.goal = (rospy.get_param("x_goal"), rospy.get_param("y_goal"))

        # State
        self.state = "STOP"

        # Listener and publisher
        rospy.Subscriber(self.ns + "/sensor/sonar_front",
                         Range, self.callbacksonar)
        rospy.Subscriber(self.ns + "/odom", Odometry, self.callbackodometry)
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

    # CALLBACK
    def callbacksonar(self, data):
        # DO NOT TOUCH
        self.sonar = data.range

    def callbackodometry(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        self.pos = (x, y)

        x = data.pose.pose.orientation.x
        y = data.pose.pose.orientation.y
        z = data.pose.pose.orientation.z
        w = data.pose.pose.orientation.w
        self.yaw = self.euler_from_quaternion(x, y, z, w)[2]

    # GETTER METHODS
    def get_sonar(self):
        return float(self.sonar)

    def get_yaw(self):
        return self.yaw

    def get_pos(self):
        return self.pos

    def get_goal(self):
        return self.goal

    def get_state(self):
        return self.state

    # SETTER METHODS
    def set_state(self, state):
        self.state = state

    # Other methods
    def distance_from(self, pos):
        x, y = pos
        x_r, y_r = self.pos
        return math.sqrt((x-x_r)**2 + (y - y_r)**2)

    def angle_to(self, pos):
        x, y = pos
        x_r, y_r = self.pos
        return - self.yaw + math.atan2(y-y_r, x-x_r)

    # Publish command

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



def reach_goal(robot):
    if robot.distance_from(robot.get_goal()) < goal_threshold:
        robot.set_state("STOP")
        return True
    else:
        return False


def align(robot, kp):
    # Yaw controller
    # The robot is not moving
    yaw_error = robot.angle_to(robot.get_goal())
    angular_vel = yaw_error * kp   # Correcting the yaw error with a gain kp
    return 0, angular_vel


def move_forward(robot, kp):
    # Speed controller
    # The robot is not turning
    distance_error = robot.distance_from(robot.get_goal())
    if distance_error < goal_threshold:
        robot.set_state("STOP")
        return 0, 0
    else:
        return kp * distance_error, 0       # setting speed with a proportional controller


def move(robot, kp_l=1, kp_a=1):
    v = move_forward(robot, kp_l)[0]
    a = align(robot, kp_a)[1]
    return v, a


def avoid_obstacle(robot, kp=1):
    global phase, yaw_goal, t0
    if phase == 0:
        yaw_goal = robot.get_yaw() + math.radians(45)  # Computing new orientation goal
        linear_vel = 0
        angular_vel = 0
        phase = 1
    elif phase == 1:
        yaw_error = yaw_goal - robot.get_yaw()  # Yaw error
        if abs(yaw_error) > angle_threshold:
            # Correcting yaw error with angular velocity and gain kp
            angular_vel = kp * yaw_error
            linear_vel = 0
        else:
            phase = 2
            angular_vel = 0
            linear_vel = 2
            t0 = rospy.get_time()
    elif phase == 2:
        if rospy.get_time() - t0 > 5:       # Moving forward for a duration of 5 seconds
            angular_vel = 0
            linear_vel = 2
        else:
            phase = 0
            robot.set_state("FORWARD")      # Set robot state to Forward state
            angular_vel = 0
            linear_vel = 2

    return linear_vel, angular_vel


def turn_right_or_left(robot, angle):
    # Not yet implemented in the strategy
    x1, y1 = robot.get_pos()
    x2, y2 = robot.get_goal()

    distance_left = math.sqrt(
        (x2 - (x1 + math.cos(angle))**2 + (y2 - (y1+math.sin(angle))**2)))
    distance_right = math.sqrt(
        (x2 - (x1 + math.cos(-angle))**2 + (y2 - (y1+math.sin(-angle))**2)))
    return "right" if distance_right < distance_left else "left"


def strat(robot):
    global phase
    if not reach_goal(robot):  # If robot did not reach the goal yet
        print(robot.get_state())
        if robot.get_sonar() < sonar_threshold and robot.get_state() != "OBSTACLE":  # if sonar detects an obstacle
            robot.set_state("OBSTACLE")  # Setting state to OBSTACLE
            phase = 0                   # Starting phase
            return avoid_obstacle(robot, 1)

        elif robot.get_state() == "OBSTACLE":
            return avoid_obstacle(robot)

        elif robot.get_state() == "FORWARD":
            return move(robot, 1, 1)

        elif robot.get_state() == "STOP":
            # If robot did not reach the goal but it is stopped we set the state to FORWARD
            robot.set_state("FORWARD")
            return move(robot, 1, 1)
    else:

        return 0, 0


def run_demo():
    robot_name = rospy.get_param("~robot_name")
    robot = Robot(robot_name)

    print("Robot {} is starting".format(robot_name))

    rate = rospy.Rate(2)  # 2 Hz
    rate.sleep()

    while not rospy.is_shutdown():
        v, a = strat(robot)
        robot.set_speed_angle(v, a)
        rate.sleep()


if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Strategy", anonymous=True)
    run_demo()

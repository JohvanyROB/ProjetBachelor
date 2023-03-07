#!/usr/bin/env python3

from time import sleep
import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry


def euler_from_quaternion(x, y, z, w):
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

    return roll_x, pitch_y, yaw_z


def constraint(val, mini=-2.0, maxi=2.0):
    if val < mini:
        return mini
    if val > maxi:
        return maxi
    return val


class Robot:
    def __init__(self, robot_name):
        self.goal = (rospy.get_param("x_goal"), rospy.get_param("y_goal"))
        self.sonar = 0.0
        self.pos = (0.0, 0.0, 0.0)
        self.rot = (0.0, 0.0, 0.0)
        self.robot_name = robot_name
        self.ns = "/" + self.robot_name

        rospy.Subscriber(self.ns + "/sensor/sonar_front",
                         Range, self.callbacksonar)
        rospy.Subscriber(self.ns + "/odom",
                         Odometry, self.callbackpos)
        rospy.Subscriber(self.ns + "/odom",
                         Odometry, self.callbackrot)
        self.cmd_vel_pub = rospy.Publisher(
            self.ns + "/cmd_vel", Twist, queue_size=1)

    def callbackrot(self, data):
        x = data.pose.pose.orientation.x
        y = data.pose.pose.orientation.y
        z = data.pose.pose.orientation.z
        w = data.pose.pose.orientation.w
        u, v, w = euler_from_quaternion(x, y, z, w)
        u, v, w = u * 180/math.pi, v * 180/math.pi, w * 180/math.pi
        self.rot = (u, v, w)

    def callbackpos(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        z = data.pose.pose.position.z
        self.pos = (x, y, z)

    def get_pos(self):
        return (self.pos)

    def get_rot(self):
        return (self.rot)

    def callbacksonar(self, data):

        self.sonar = data.range

    def get_sonar(self):

        return float(self.sonar)

    def set_speed_angle(self, linear_vel, angular_vel):

        cmd_vel = Twist()
        cmd_vel.linear.x = constraint(linear_vel)
        cmd_vel.angular.z = constraint(angular_vel, mini=-1, maxi=1)
        self.cmd_vel_pub.publish(cmd_vel)

    def getOrrientRelativeToGoal(self, gx=13, gy=-27):
        x, y = self.pos[0], self.pos[1]
        dx, dy = gx - x, gy - y
        yaw = self.rot[2]
        deg = math.atan2(dy, dx)*180/math.pi - yaw
        if(deg > 180):
            return deg - 360
        if(deg < -180):
            return deg + 360
        return deg

    def turn_left(self):
        newRot = self.rot[2] + 90
        if(newRot > 180):
            newRot -= 360
        elif(newRot < -180):
            newRot += 360
        while abs(self.rot[2] - newRot) >= 5:
            self.set_speed_angle(0, 1)
        self.set_speed_angle(0, 0)

    def turn_right(self):
        newRot = self.rot[2] - 90
        if(newRot > 180):
            newRot -= 360
        elif(newRot < -180):
            newRot += 360
        while abs(self.rot[2] - newRot) >= 5:
            self.set_speed_angle(0, -1)
        self.set_speed_angle(0, 0)

    def evitement(self, direction="avant"):
        counter = 0
        maxtime = 15000
        if direction == "avant":
            deg = self.getOrrientRelativeToGoal(*self.goal)
            onGoal = self.arrive()
            while (self.sonar > 2.5) and (abs(deg) <= 10) and (not onGoal):
                self.set_speed_angle(2, 0)
                onGoal = self.arrive()
                deg = self.getOrrientRelativeToGoal(*self.goal)
            self.set_speed_angle(0, 0)

        elif direction == "gauche":
            while counter <= maxtime:
                self.set_speed_angle(2, 0)
                counter += 1
            self.set_speed_angle(0, 0)

        elif direction == "droite":
            while counter <= maxtime:
                self.set_speed_angle(2, 0)
                counter += 1
            self.set_speed_angle(0, 0)

        else:
            while counter <= maxtime:
                self.set_speed_angle(-2, 0)
                counter += 1
            self.set_speed_angle(0, 0)
            self.check(True)

    def check(self, backwards=False):
        if not backwards:
            if self.sonar > 2.5:
                self.evitement()
                return

        self.turn_left()
        if self.sonar > 2.5:
            self.evitement("gauche")
            return

        self.turn_right()
        self.turn_right()
        if self.sonar > 2.5:
            self.evitement("droite")
            return

        self.turn_left()
        self.evitement("arriere")

    def orientTowardsGoal(self):
        deg = self.getOrrientRelativeToGoal(*self.goal)
        while abs(deg) >= 10:
            if deg >= 0:
                self.set_speed_angle(0, 1)
            else:
                self.set_speed_angle(0, -1)
            deg = self.getOrrientRelativeToGoal(*self.goal)

    def arrive(self):
        if abs(self.goal[0] - self.pos[0]) <= 2 and abs(self.goal[1] - self.pos[1]) <= 2:
            self.set_speed_angle(0, 0)
            return True
        return False


def run():
    robot_name = rospy.get_param("~robot_name")
    robot = Robot(robot_name)
    print(f"Robot {robot_name} is starting")
    sleep(2)
    rate = rospy.Rate(15)
    arrive = False
    while not rospy.is_shutdown():
        if not arrive:
            robot.orientTowardsGoal()
            robot.check()
        arrive = robot.arrive()
        rate.sleep()


if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Strategy", anonymous=True)

    run()
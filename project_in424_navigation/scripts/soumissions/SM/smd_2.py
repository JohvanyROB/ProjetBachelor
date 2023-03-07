#!/usr/bin/env python3
#by Axel ALVES BATISTA and Robin JULLIEN

from sensor_msgs.msg import Range
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
import rospy
import math
from math import atan2, sqrt


class Robot:
    def __init__(self, robot_name):
        self.sonar = 0.0  # Sonar distance
        self.gyro = 0.0  # gyroscope position
        self.robot_name = robot_name
        self.ns = "/" + self.robot_name
        self.px = 0.0
        self.py = 0.0
        self.yaw = 0.0

        '''Listener and publisher'''
        rospy.Subscriber(self.ns + "/sensor/sonar_front",
                         Range, self.callbacksonar)

        self.cmd_vel_pub = rospy.Publisher(
            self.ns + "/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber(self.ns + "/odom", Odometry, self.callback_pose)

    def callback_pose(self, data):
        self.px = data.pose.pose.position.x
        self.py = data.pose.pose.position.y
        ox = data.pose.pose.orientation.x
        oy = data.pose.pose.orientation.y
        oz = data.pose.pose.orientation.z
        ow = data.pose.pose.orientation.w
        _, _,  self.yaw = self.euler_from_quaternion(ox, oy, oz, ow)

    def get_pose(self):
        return [self.px, self.py, self.yaw]

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

        return [roll_x, pitch_y, yaw_z]  # in radians


def run_demo():
    '''Main loop'''
    robot_name = rospy.get_param("~robot_name")
    robot = Robot(robot_name)
    print("Robot {} is starting".format(robot_name))

    rate = rospy.Rate(2)
    goal = Point()
    goal.x = rospy.get_param('/x_goal')
    goal.y = rospy.get_param('/y_goal')
    print(goal)

    while not rospy.is_shutdown():

        # Write your strategy here ...
        # linear_velocity = 1
        angular_velocity = 0
        sonar = robot.get_sonar()
        pose = robot.get_pose()
        theta = pose[2]

        # print(theta)  # theta n'est pas bon
        # print(pose[2])

        x = goal.x - pose[0]
        y = goal.y - pose[1]
        angle_to_goal = atan2(y, x)
        distance = sqrt((x)**2 + (y)**2)  # calcul distance robot au goal

        print("SONAR VALUE : {:.2f}".format(sonar))
        print("pose : ", pose)
        if (sonar < 2):
            linear_velocity = 0
            angular_velocity = 0.8
            robot.set_speed_angle(linear_velocity, angular_velocity)
        else:
            if distance >= 0.5:      #

                # calcul l'angle le plus faible entre le point d'arriver et celui du robot pour lui donner la direction dans laquelle il doit aller
                if (abs(angle_to_goal - theta) > 0.5):
                    # print("test")
                    linear_velocity = 0
                    angular_velocity = 0.8
                    robot.set_speed_angle(linear_velocity, angular_velocity)

                else:
                    linear_velocity = 1
            else:
                linear_velocity = 0
                angular_velocity = 0

        # Finishing by publishing the desired speed. DO NOT TOUCH.
        robot.set_speed_angle(linear_velocity, angular_velocity)
        rate.sleep()


if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Strategy", anonymous=True)

    run_demo()

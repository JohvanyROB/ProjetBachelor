#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range


from nav_msgs.msg import Odometry


class Robot:
    def __init__(self, robot_name):
        self.sonar = 0.0  # Sonar distance
        self.robot_name = robot_name
        self.ns = "/" + self.robot_name

        self.x = 0.0
        self.y = 0.0
        self.orientz = 0.0
        self.yaw = 0.0
        self.alpha = 0.0

        '''Listener and publisher'''
        rospy.Subscriber(self.ns + "/sensor/sonar_front",
                         Range, self.callbacksonar)
        self.cmd_vel_pub = rospy.Publisher(
            self.ns + "/cmd_vel", Twist, queue_size=1)

        rospy.Subscriber(self.ns + "/odom", Odometry, self.callbackposition)

    def callbacksonar(self, data):
        # DO NOT TOUCH
        self.sonar = data.range

    def get_sonar(self):
        # DO NOT TOUCH
        return float(self.sonar)

    def callbackposition(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        #self.orientz = msg.pose.pose.orientation.z
        _, _, self.yaw = self.euler_from_quaternion(
            msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        print("x = ", self.x)
        print("y = ", self.y)
        print("z orient = ", self.yaw)

        x_final = rospy.get_param("/x_goal")  # get the coordinate of x finale

        y_final = rospy.get_param("/y_goal")  # get the coordinate of y finale

        self.alpha = math.atan2((y_final - self.y), (x_final - self.x))
        print("alpha via callbackpos. = ", self.alpha)

    def get_position(self):
        return self.x, self.y, self.yaw, self.alpha

    def constraint(self, val, mini=-2.0, maxi=2.0):
        # DO NOT TOUCH
        if val < mini:
            return mini
        if val > maxi:
            return maxi
        return val

    def set_speed_angle(self, linear_vel, angular_vel):
        # DO NOT TOUCH
        cmd_vel = Twist()
        cmd_vel.linear.x = self.constraint(linear_vel)
        cmd_vel.angular.z = self.constraint(angular_vel, mini=-1, maxi=1)
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
    #print("Robot {} is starting".format(robot_name))

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        # Write your strategy here ...
        # Initialisation:
        linear_velocity = 0  # VITESSE ENTRE -2 et 2
        angular_velocity = 0  # VITESSE ENTRE -1 et 1
        sonar = robot.get_sonar()

        # FINAL POSITION
        x_final = rospy.get_param("/x_goal")  # get the coordinate of x finale
        print("x_goal = ", x_final)  # get the coordinate of x finale

        y_final = rospy.get_param("/y_goal")  # /rosparam
        print("y_goal = ", y_final)

        # Sonar
        #print("SONAR VALUE : {:.2f}".format(sonar))

        # position of the robots and variables
        x, y, yaw, alpha = robot.get_position()

        print("yaw : ", yaw)
        print("alpha : ", alpha)
        yaw = yaw * (180/math.pi)
        alpha = alpha * (180/math.pi)
        print("alpha en degré", alpha)
        print("yaw en degré", yaw)

        # distance between the robot and its goal
        Distance = math.sqrt((x_final-x)**2+(y_final-x)**2)
        print("Distance :", Distance)

        # LOOP
        if abs(x_final - x) > 3 or abs(y_final - y) > 3:

            if(sonar < 1.5):
                linear_velocity = -2
                angular_velocity = 1

            else:
                # the robot rotate towards the target
                if (alpha - yaw) > 5:
                    angular_velocity = 0.2  # VITESSE ENTRE -1 et 1
                    linear_velocity = 0

                elif (alpha - yaw) < -5:
                    angular_velocity = -0.2  # VITESSE ENTRE -1 et 1
                    linear_velocity = 0

                else:
                    linear_velocity = 2
                    angular_velocity = 0

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

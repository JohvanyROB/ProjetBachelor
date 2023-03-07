#!/usr/bin/env python3
import numpy as np
import math
import rospy
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range


class Robot:
    def __init__(self, robot_name):
        self.sonar = 0.0  # Sonar distance
        self.x_pos = 0
        self.y_pos = 0
        self.z_pos = 0
        self.x_deg = 0
        self.y_deg = 0
        self.z_deg = 0
        self.w_deg = 0
        self.xgoal = rospy.get_param("/x_goal")
        self.ygoal = rospy.get_param("/y_goal")
        self.robot_name = robot_name
        self.ns = "/" + self.robot_name

        '''Listener and publisher'''
        rospy.Subscriber(self.ns + "/sensor/sonar_front",
                         Range, self.callbacksonar)

        rospy.Subscriber(self.ns + "/odom",
                         Odometry, self.callbackpos)

        rospy.Subscriber(self.ns + "/odom",
                         Odometry, self.callbackorient)

        self.cmd_vel_pub = rospy.Publisher(
            self.ns + "/cmd_vel", Twist, queue_size=1)

    def callbacksonar(self, data):
        # DO NOT TOUCH
        self.sonar = data.range

    def callbackpos(self, data):
        self.x_pos = data.pose.pose.position.x
        self.y_pos = data.pose.pose.position.y
        self.z_pos = data.pose.pose.position.z

    def callbackorient(self, data):
        self.x_deg = data.pose.pose.orientation.x
        self.y_deg = data.pose.pose.orientation.y
        self.z_deg = data.pose.pose.orientation.z
        self.w_deg = data.pose.pose.orientation.w

    def get_sonar(self):
        # DO NOT TOUCH
        return float(self.sonar)

    def get_pos(self):
        return float(self.x_pos), float(self.y_pos), float(self.z_pos)

    def get_orient(self):
        return float(self.x_deg), float(self.y_deg), float(self.z_deg), float(self.w_deg)

    def get_posgoal(self):
        return float(self.xgoal), float(self.ygoal)

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

        return roll_x * 180/np.pi, pitch_y * 180/np.pi, yaw_z * 180/np.pi # in degrees

    def goal_orient(self, x_goal, y_goal, x_pos, y_pos):
        dist_goal = ((x_goal - x_pos)**2 + (y_goal - y_pos)**2)**0.5
        orient_goal = 2 * np.arctan(((y_pos - y_goal)/dist_goal) / (1 + (x_pos - x_goal)/dist_goal))   
        return dist_goal, orient_goal * 180/np.pi
  


def run_demo():
    '''Main loop'''
    robot_name = rospy.get_param("~robot_name")
    robot = Robot(robot_name)
    print("Robot {} is starting".format(robot_name))

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        # Write your strategy here ...

        # Data recovery
        sonar = robot.get_sonar()
        x, y, z = robot.get_pos()
        x_deg, y_deg, z_deg, w_deg = robot.get_orient()
        z_orient = robot.euler_from_quaternion(
            x_deg, y_deg, z_deg, w_deg)[2]

        xgoal, ygoal = robot.get_posgoal()
        dist_goal, orient_goal = robot.goal_orient(xgoal, ygoal, x, y)

        # Stategy to reach goal
        if dist_goal > 2 :                                                              # The limit delimited by the arrival
            
            if orient_goal > 0 :                                                        # We are on the upper part of the semi-cycle
                if (- 8 < z_orient + 180 - orient_goal < 8) & (sonar == 5):             # Between 0 and 180 °
                    linear_velocity = 1.5
                    angular_velocity = 0
                elif (z_orient + 180 - orient_goal > 8) & (sonar == 5):                 # The front of the robot is above the goal heading
                    linear_velocity = 0.8
                    angular_velocity = -0.3
                elif (z_orient + 180 - orient_goal < -8) & (sonar == 5):                # The front of the robot is bellow the goal heading
                    linear_velocity = 0.8
                    angular_velocity = 0.3
                
                elif (sonar < 5):

                    linear_velocity = -0.2
                    angular_velocity = 1
                    z_orient = orient_goal - 50
                    time.sleep(2.0)


            elif orient_goal < 0 :                                                      # We are on the under part of the semi-cycle
                if (- 8 < z_orient - 180 - orient_goal < 8) & (sonar == 5):             # Between -180 and 0 °
                    linear_velocity = 1.5
                    angular_velocity = 0
                elif (z_orient - 180 - orient_goal > 8) & (sonar == 5):
                    linear_velocity = 0.8
                    angular_velocity = -0.3
                elif (z_orient - 180 - orient_goal < -8) & (sonar == 5):
                    linear_velocity = 0.8
                    angular_velocity = 0.3
                
                elif (sonar < 5):

                    linear_velocity = 0.2
                    angular_velocity = 1
                    z_orient = orient_goal + 50
                    time.sleep(2.0)
        
        else :
            linear_velocity = 0
            angular_velocity = 0
       

        # Getting informations
        print("=============================================", '\n',
              "------------------- SONAR -------------------", '\n',
              "Sonar value : {:.2f}".format(sonar), '\n \n',
              "------------------ POSITION -----------------", '\n',
              "X is : {:.2f}".format(x), '\n',
              "Y is : {:.2f}".format(y), '\n \n',
              "---------------- ORIENTATION ----------------", '\n',
              "Z angle is : {:.2f}".format(z_orient), '\n',
              "Orientation / goal > 0 is : {:.2f}".format(z_orient + 180 - orient_goal), '\n',
              "Orientation / goal < 0 is : {:.2f}".format(z_orient - 180 - orient_goal), '\n \n',
              "-------------- GOAL ORIENTATION -------------", '\n',
              "Goal heading is : {:.2f}".format(orient_goal), '\n \n',
              "-------------- DISTANCE ARRIVEE -------------", '\n',
              "Arrival in : {:.2f}".format(dist_goal), '\n \n',)

        # Finishing by publishing the desired speed. DO NOT TOUCH.
        robot.set_speed_angle(linear_velocity, angular_velocity)
        rate.sleep()


if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Strategy", anonymous=True)

    run_demo()

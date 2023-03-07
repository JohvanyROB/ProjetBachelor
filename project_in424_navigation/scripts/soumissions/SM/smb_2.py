#!/usr/bin/env python3
"""
        Projet In424
        Groupe 2 SM1B

        Mariam Eva Baptiste
        Camille Chatillon
        Elena Ignaczuk
        Emma Laffin
"""

import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry


class Robot:
    def __init__(self, robot_name):
        self.sonar = 0.0  # Sonar distance
        self.x = 0.0  # Position of the robot on X axis
        self.y = 0.0  # Position of the robot on Y axis
        self.yaw = 0.0  # Yaw angle
        self.secur_dist = 3.0  # Distance of security
        self.robot_name = robot_name
        self.ns = "/" + self.robot_name
        # x coordinate of the final destination
        self.xpose_final = rospy.get_param("/x_goal")
        # y coordinate of the final destination
        self.ypose_final = rospy.get_param("/y_goal")

        '''Listener and publisher'''
        rospy.Subscriber(self.ns + "/sensor/sonar_front",
                         Range, self.callbacksonar)
        # Introduction of the library Odometry in our programm
        rospy.Subscriber(self.ns + '/odom', Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher(
            self.ns + "/cmd_vel", Twist, queue_size=1)

    def callbacksonar(self, data):
        # DO NOT TOUCH
        self.sonar = data.range

    def odom_callback(self, msg):
        """
        This defintion allows us to get the position of our robot on uts x and y coordinates
        but also to get the value of its yaw. As we get the yaw in quartanion, we use the definition
        euler_from_quaternion to transform the yaw angle in an euler angle.
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        _, _, self.yaw = self.euler_from_quaternion(
            msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

    def get_sonar(self):
        # DO NOT TOUCH
        return float(self.sonar)

    def get_pose(self):
        """
        Definition that gives the values of the x and y coordinates and the value
        of the yaw of the robot.
        """
        return self.x, self.y, self.yaw

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

    def avoid_obstacle(self):
        """
        Function that allows to avoid obstacle. When the sonar detects a distance lower
        than the security distance, the robot will check its environnement.
        Depending on the obstacles around it, it will act differently.
        """

        """Scanning on the right"""

        # Rotation in the antitrigonometric direction
        self.set_speed_angle(0, -1)
        # Sleep to have a rotation of 1 seconde (corresponding approximativly to a rotation of 60°)
        rospy.sleep(1)
        # Recuperation of the value of the sonar at 60°
        sonar_right_little_side = self.get_sonar()
        print("sonar_right_little_side", sonar_right_little_side)
        # Sleep 0.5 seconde more to have a rotation of approximativly 90°
        rospy.sleep(0.5)
        # Recuperation of the value of the sonar at 90°
        sonar_right_side = self.get_sonar()
        print("sonar_right_side", sonar_right_side)
        # Rotation in the trigonometric direction
        self.set_speed_angle(0, 1)
        # Sleep of 1.5 to come back to the original position
        rospy.sleep(1.5)
        self.set_speed_angle(0, 0)

        """Scanning on the left"""
        # Same as the scanning to the right but for the left

        self.set_speed_angle(0, 1)
        rospy.sleep(1)
        sonar_left_little_side = self.get_sonar()
        print("sonar_left_little_side", sonar_left_little_side)
        rospy.sleep(0.5)
        sonar_left_side = self.get_sonar()
        print("sonar_left_side", sonar_left_side)
        self.set_speed_angle(0, -1)
        rospy.sleep(1.5)
        self.set_speed_angle(0, 0)

        Pause = 0   # Creation the variable pause to prevent two loop from being executed at the same time

        # Creation of the different osbtacles. Each obstacle has a particular shpe. Depending on the shape scanned by the robot, it will avoid
        # the obstacle with a particular manner.

        """ Wall

        In the case of a wall, the sonar will detect nothing on the 90°-right and 90°-left side but will detect obstacle in 60°-right and 60°-left.
        So the robot will try to avoid it by choosing either the left or the right side to turn depending on where the final destination
        is compared to the actual position.
        """

        if ((sonar_right_side >= 5) and (sonar_left_side >= 5) and (sonar_right_little_side <= self.secur_dist) and (sonar_left_little_side <= self.secur_dist) and (Pause == 0)):
            print("The robot avoid the wall by turning on the better side")
            if self.ypose_final < self.y:
                # Rotation of 90° on the right
                self.set_speed_angle(0, -1)
            else:
                # Rotation of 90° on the left
                self.set_speed_angle(0, 1)

            rospy.sleep(1.5)
            self.set_speed_angle(2, 0)
            # Move forward for 1.5 seconds
            rospy.sleep(1.5)
            self.set_speed_angle(0, 0)
            Pause = 1

            """Isolated obstacle

            In the case of an isolated obstacle, the sonar will detect nothing on the 90°-right and 90°-left side and nothing in 50°-right and 50°-left too.
            So the robot will try to avoid it by choosing either the left or the right side to turn depending on where the final destination
            is compared to the actual position.
            """

        elif ((sonar_right_little_side > self.secur_dist) and (sonar_left_little_side > self.secur_dist) and (sonar_right_side > self.secur_dist) and (sonar_left_side > self.secur_dist) and (Pause == 0)):
            print("The robot avoid the obstacle by turning on the better side")
            if self.ypose_final < self.y:
                # Rotation of 90° on the right
                self.set_speed_angle(0, -1)
            else:
                # Rotation of 90° on the left
                self.set_speed_angle(0, 1)

            rospy.sleep(1.5)
            self.set_speed_angle(2, 0)
            # Move forward for 1.5 seconds
            rospy.sleep(1.5)
            self.set_speed_angle(0, 0)
            Pause = 1

            """U-shaped wall

            In the case of a U-obstacle, the sonar will detect obstacle on the 90°-right and 90°-left side and also in 60°-right and 60°-left.
            The robot will in a first time move backward and after will avoid the obstacle by choosing the best way.
            """

        elif ((sonar_right_side <= 4) and (sonar_left_side <= 4) and (sonar_right_little_side <= 4) and (sonar_left_little_side <= 4) and (Pause == 0)):
            print("The robot move backward")
            self.set_speed_angle(-2, 0)
            # Move backward for 2 seconds
            rospy.sleep(2)
            # Choose the best way
            if self.ypose_final < self.y:
                # Rotation of 90° on the right
                self.set_speed_angle(0, -1)
            else:
                # Rotation of 90° on the left
                self.set_speed_angle(0, 1)

            rospy.sleep(1.5)
            self.set_speed_angle(2, 0)
            # Move forward for 1.5 seconds
            rospy.sleep(1.5)
            self.set_speed_angle(0, 0)
            Pause = 1

            """left-L-shaped wall

            In the case of a left-L obstacle, the sonar will detect obstacle on the left and nothing on the right side.
            So the robot will avoid the obstacle by passing on the right and move forward.
            """

        elif ((sonar_right_side >= sonar_left_side) and (sonar_right_little_side >= sonar_left_little_side) and (sonar_right_side >= 5) and (Pause == 0)):
            print("The robot bypass a left-L obstacle througt the right side ")
            self.set_speed_angle(0, -1)
            # Rotation of 90° on the right
            rospy.sleep(1.5)
            self.set_speed_angle(2, 0)
            # Move forward for 1.5 seconds
            rospy.sleep(1.5)
            self.set_speed_angle(0, 0)
            Pause = 1

            """Right-L-shaped wall

            In the case of a left-L obstacle, it's the same things that for the left-L but reversed
            So the robot will avoid the obstacle by passing on the left and move forward.
            """

        elif ((sonar_right_side <= sonar_left_side) and (sonar_right_little_side <= sonar_left_little_side) and (sonar_left_side >= 5) and (Pause == 0)):
            print("The robot bypass a right-L obstacle througt the left side ")
            self.set_speed_angle(0, 1)
            rospy.sleep(1.5)
            self.set_speed_angle(2, 0)
            rospy.sleep(1.5)
            self.set_speed_angle(0, 0)
            Pause = 1

    def angle_theta(self):
        """
        This function calculates The point C which is th eprojection of the position of the robot and the arrival.
        An angle will be deduced from this point and this will be used to define the path that the robot will take.
        """
        c = [self.xpose_final, self.y]  # Determine C coordinates
        BC = math.sqrt((c[0]-self.xpose_final)**2+(c[1]-self.ypose_final)**2)
        AC = math.sqrt((c[0]-self.x)**2+(c[1]-self.y)**2)
        theta = math.atan(BC/AC)
        # The value of theta will depend on where the arrival is from the robot
        if self.ypose_final < self.y:
            theta = -theta
        elif (self.xpose_final < self.x) and (self.ypose_final < self.y):
            theta = -3.142+theta
        elif (self.xpose_final < self.x) and (self.ypose_final > self.y):
            theta = 3.142-theta
        return theta

    def get_trajectory(self, theta, linear_velocity, angular_velocity):
        """
        This function will make the robot turn on itself in order to move to the final point by
        always checking if there is an obstacle on its way.
        2 cases:
         - The yaw is different from theta, the robot will turn on its right or on its left depending
           on the position of the destitantion.
         - The yaw is similar to theta, the robot will move straight forward.

        Before doing anything, the funtion will get the value of the sonar to be sure that there is no
        obstacle in front of the robot.
        """
        if self.get_sonar() < self.secur_dist:
            self.avoid_obstacle()
            print("\nWe avoid the obstacle !")
        else:
            if (round(self.yaw, 2) != round(theta, 2)):
                linear_velocity = 0.0
                if self.yaw > theta:
                    angular_velocity = -0.1
                else:
                    angular_velocity = 0.1
                print("\nWe turn on ourselves")
            else:
                linear_velocity = 1.0
                angular_velocity = 0.0
                print("\nIt's the right way !")

        return linear_velocity, angular_velocity

    def YOU_WIN(self):
        """
        This definition calculate the interval in which we consider the robot at the final destination.
        The robot will reached to correct interval with pose_final +/- 0.1.
        """
        xminus = self.xpose_final - 0.1
        xplus = self.xpose_final + 0.1
        yminus = self.ypose_final - 0.1
        yplus = self.ypose_final + 0.1
        return(xminus, xplus, yminus, yplus)


def run_demo():
    '''Main loop'''
    robot_name = rospy.get_param("~robot_name")
    robot = Robot(robot_name)
    print("Robot {} is starting".format(robot_name))
    # Initialisation of the velocities to 0
    # Only can have the values in list(range(-2.0, 2.0))
    linear_velocity = 0.0
    # Only can have the values in list(range(-1.0, 1.0))
    angular_velocity = 0.0
    # Creation of the interval in which the robot has to be
    xminus, xplus, yminus, yplus = robot.YOU_WIN()
    rate = rospy.Rate(4)

    while not rospy.is_shutdown():
        # The robot will take the optimal path by calculating the angle theta. Before doing anything
        # the robot will check if any object is in front of it or not. If there is an obstacle, the robot will
        # scann all around itself to see the shape of the obstacle. Depending on its shape, the robot will act
        # differently to avoid it. If there is no obstacle, the robot will make sure that the value of its yaw
        # is the same as the value of the theta calculated. If they are the same, the robot will move forward.
        # The robot will reach its arrival only if its coordiantes are similar to the coordinates of the final
        # point with a precision of 0.1. When it has reach the final point, the programm will end.
        sonar = robot.get_sonar()
        if sonar == 0:
            continue
        pose = robot.get_pose()
        theta = robot.angle_theta()

        print("SONAR VALUE : {:.2f}".format(sonar))
        print("\nThe angle theta has the value of: ", theta)
        print("\nyaw: ", pose[2])
        # Check the trajectory
        linear_velocity, angular_velocity = robot.get_trajectory(
            theta, linear_velocity, angular_velocity)
        # Finishing by publishing the desired speed. DO NOT TOUCH.
        print("Our final destination has the coordinates of (",
              robot.xpose_final, ",", robot.ypose_final, ")")
        print("Our actual coordinates of (", robot.x, ",", robot.y, ")\n")

        robot.set_speed_angle(linear_velocity, angular_velocity)

        # Check if the robot has reached the final destination
        if xminus <= robot.x <= xplus:
            if yminus <= robot.y <= yplus:
                linear_velocity = 0.0
                angular_velocity = 0.0
                robot.set_speed_angle(linear_velocity, angular_velocity)
                print(
                    "\nThe robot has reach the final destination ! Congratulation !\n\n")
                break

        rate.sleep()


if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Strategy", anonymous=True)

    run_demo()

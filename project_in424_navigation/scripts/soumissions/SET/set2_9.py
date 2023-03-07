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
        self.position = [0.0, 0.0, 0.0]  # Position du robot
        self.angle = [0.0, 0.0, 0.0, 0.0]  # Position angulaire du robot

        '''Listener and publisher'''
        rospy.Subscriber(self.ns + "/sensor/sonar_front",
                         Range, self.callbacksonar)
        self.cmd_vel_pub = rospy.Publisher(
            self.ns + "/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber(self.ns + "/odom", Odometry, self.callbackposition, )
        rospy.Subscriber(self.ns + "/odom", Odometry, self.callbackorientation)

    def callbackposition(self, data):
        self.position[0] = data.pose.pose.position.x
        self.position[1] = data.pose.pose.position.y
        self.position[2] = data.pose.pose.position.z

    def callbackorientation(self, data):
        self.angle[0] = data.pose.pose.orientation.x
        self.angle[1] = data.pose.pose.orientation.y
        self.angle[2] = data.pose.pose.orientation.z
        self.angle[3] = data.pose.pose.orientation.w

    def callbacksonar(self, data):
        # DO NOT TOUCH
        self.sonar = data.range

    def get_sonar(self):
        # DO NOT TOUCH
        return float(self.sonar)

    def get_orientation(self):
        return self.angle

    def get_position(self):
        return self.position

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
    xg = rospy.get_param("/x_goal")  # Retrieving the xgoal value
    yg = rospy.get_param("/y_goal")  # Retrieving the ygoal value
    print('xg,', xg, 'yg,', yg)
    while not rospy.is_shutdown():
        linear_velocity = 2
        angular_velocity = 0
        # Write your strategy here ...
        orientation = robot.get_orientation()
        position = robot.get_position()
        sonar = robot.get_sonar()  # Retrieving the sonar value
        x, y, z = robot.euler_from_quaternion(
            orientation[0], orientation[1], orientation[2], orientation[3])
        deltax = xg - position[0]
        deltay = yg - position[1]
        # Calculate the direction my robot must take to go to the finish area when we have a deltax > 0
        anglerobot = (math.atan(deltay/deltax))
        # Calculate the direction my robot must take to go to the finish area when we have a deltax < 0
        anglerobot2 = (anglerobot + math.pi) - (2*math.pi)
        print('x,', position[0], 'y,', position[1], 'z,', z)
        print('\n')
        print('anglerobot,', anglerobot)
        print('\n')
        print('anglerobot2,', anglerobot2)
        print('Je roule ')
        droite = -1
        gauche = 1
        # If my robot is on arrival it stops.
        if -1.8 < deltax < 1.8 and -1.8 < deltay < 1.8:
            linear_velocity = 0
            angular_velocity = 0
            print('je suis arrivé')
        else:
            if deltax < 0:      # If I am positioned to the right of my arrival zone, turn right.
                linear_velocity = 0
                angular_velocity = droite
                # The robot finds the direction of the finish area it moves forward.
                if anglerobot2 - 0.2 < z < anglerobot2 + 0.2:
                    print('Je me calibre et je roule en direction de l objectif 2')
                    angular_velocity = 0
                    linear_velocity = 2
                if sonar < 5:       # If the robot encounters an obstacle it dodges it
                    print('Obstacle rencontré, je l evite')
                    robot.set_speed_angle(0.0, 0.0)  # You don't move anymore
                    rospy.sleep(2.0)  # for two seconds
                    robot.set_speed_angle(0.0, gauche)  # You turn left
                    rospy.sleep(1.0)  # For a second
                    robot.set_speed_angle(2.0, 0.0)  # You go straight ahead
                    rospy.sleep(1.0)  # For a second
            else:       # If I am positioned to the left of my arrival zone, turn left.
                linear_velocity = 0
                angular_velocity = gauche
                # The robot finds the direction of the finish area it moves forward.
                if anglerobot - 0.2 < z < anglerobot + 0.2:
                    print('Je me calibre et je roule en direction de l objectif 2')
                    angular_velocity = 0
                    linear_velocity = 2
                if sonar < 5:       # If the robot encounters an obstacle it dodges it
                    print('Obstacle rencontré, je l evite')
                    robot.set_speed_angle(0.0, 0.0)  # You don't move anymore
                    rospy.sleep(2.0)  # for two seconds
                    robot.set_speed_angle(0.0, droite)  # You turn right
                    rospy.sleep(1.0)        # For a second
                    robot.set_speed_angle(2.0, 0.0)  # You go straight ahead
                    rospy.sleep(1.0)  # For a second

        # Finishing by publishing the desired speed. DO NOT TOUCH.
        robot.set_speed_angle(linear_velocity, angular_velocity)
        rate.sleep()


if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Strategy", anonymous=True)

    run_demo()

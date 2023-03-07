#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry


class Robot:
    def __init__(self, robot_name):
        self.sonar = 0.0  # Sonar distance
        self.robot_name = robot_name
        self.ns = "/" + self.robot_name
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.pitch = 0.0

        self.xgoal = rospy.get_param("/x_goal")
        self.ygoal = rospy.get_param("/y_goal")

        self.heading = 0.0
        self.distance2goal = 0.0

        self.sonar_warning = 4.5        # The moment when the robot goes into avoidance mode
        self.goalVar = 1.8              # Space tolerated when in the green square
        self.distanceVar = 15           # Distance when the robot is really close to the goal

        '''Listener and publisher'''
        rospy.Subscriber(self.ns + "/sensor/sonar_front",
                         Range, self.callbacksonar)
        self.cmd_vel_pub = rospy.Publisher(
            self.ns + "/cmd_vel", Twist, queue_size=1)

        rospy.Subscriber(self.ns + "/odom",
                         Odometry, self.callbackpose)

        self.nav_odom = rospy.Publisher(
            self.ns + "/nav_odom", Pose, queue_size=1)

    def callbackpose(self, data):
        self.roll, self.pitch, self.yaw = self.euler_from_quaternion(data.pose.pose.orientation.x, data.pose.pose.orientation.y,
                                                                     data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.z = data.pose.pose.position.z

    def goalposition(self):
        return float(self.xgoal), float(self.ygoal)

    def getGoalVar(self):
        return float(self.goalVar)

    def getDistanceVar(self):
        return int(self.distanceVar)

    def get_position(self):
        return float(self.x), float(self.y), float(self.z)

    def get_orientation(self):
        return float(self.roll), float(self.pitch), float(self.yaw)

    def callbacksonar(self, data):
        # DO NOT TOUCH
        self.sonar = data.range

    def get_sonar(self):
        # DO NOT TOUCH
        return float(self.sonar)

    def get_sonar_warning(self):
        return float(self.sonar_warning)

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
    xgoal, ygoal = robot.goalposition()
    print("Goal positions: ", xgoal, ygoal)
    gVar = robot.getGoalVar()
    dVar = robot.getDistanceVar()

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        # Write your strategy here ...
        # Goal coordinates: x=13, y=-27

        linear_velocity = 0  # Make the robot move
        angular_velocity = 0
        sonar = robot.get_sonar()
        sonar_warning = robot.get_sonar_warning()
        x, y, z = robot.get_position()
        roll, pitch, yaw = robot.get_orientation()

        print("SONAR VALUE : {:.2f}".format(sonar))
        print("ROBOT POS : {:.2f}, {:.2f}, {:.2f}".format(x, y, z))
        print("ROBOT ORIENT : {:.2f}, {:.2f}, {:.2f}".format(roll, yaw, pitch))

        # Corresponds to yaw bc 2 dimensions
        heading = math.atan((ygoal - y) / (xgoal - x))
        # Distance = math.sqrt((ygoal - y)**2 / (xgoal - x)**2)
        print("ROBOT HEADING : {:.2f}".format(heading))

        if (round(xgoal)-gVar < round(x) < round(xgoal)+gVar and round(ygoal)-gVar < round(y) < round(ygoal)+gVar):
            # When in the green square, it stops
            print("FINISHED")
            linear_velocity = 0
            angular_velocity = 0

        else:
            # When not in the green zone
            if (sonar < sonar_warning):
                # When an obstacle is detected

                if (round(xgoal)-(gVar+2.5) < round(x) < round(xgoal)+(gVar+2.5) and round(ygoal)-(gVar+2.5) < round(y) < round(ygoal)+(gVar+2.5)):
                    # When almost in the green zone --> obstacle detection is disabled to let the other bots in the green zone
                    linear_velocity = 5
                    angular_velocity = 0

                if (abs(heading - yaw) > 1.0):
                    # When not in the course
                    print("not in course")
                    linear_velocity = 0
                    angular_velocity = -10
                else:
                    if (sonar < sonar_warning - 3):
                        # When an obstacle is detected too close --> Stops and turns on itself until nothing is on its way
                        print("CRITICAL OBSTACLE")
                        linear_velocity = -5
                        angular_velocity = -8
                    else:
                        # When an obstacle is detected at a normal distance --> Turns (depending on the heading) and still goes forward
                        if (abs(heading) - abs(yaw) > 0):
                            angular_velocity = -5
                            linear_velocity = 4
                        else:
                            angular_velocity = 5
                            linear_velocity = 4

            else:
                # When no obstacle detected
                if (heading - 0.08 < yaw < heading + 0.08):
                    # Keeps going towards the goal, no turns
                    print("going straight")
                    angular_velocity = 0
                    linear_velocity = 5

                else:
                    # Looking for the right orientation when not in the track (created by us)
                    print("looking around...")
                    if (abs(heading) - abs(yaw) > 0):
                        angular_velocity = -.2
                        linear_velocity = 1.5
                    else:
                        angular_velocity = .2
                        linear_velocity = 2.0

            # Finishing by publishing the desired speed. DO NOT TOUCH.
        robot.set_speed_angle(linear_velocity, angular_velocity)
        rate.sleep()


if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Strategy", anonymous=True)
    run_demo()
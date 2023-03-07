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

        self.x_pos = 0.0
        self.y_pos = 0.0
        self.yaw_z = 0.0  # in radians

        # retrieve target/goal position
        self.x_target = (rospy.get_param("/x_goal"))
        self.y_target = (rospy.get_param("/y_goal"))

        '''Listener and publisher'''
        rospy.Subscriber(self.ns + "/sensor/sonar_front",
                         Range, self.callbacksonar)
        self.cmd_vel_pub = rospy.Publisher(
            self.ns + "/cmd_vel", Twist, queue_size=1)

        rospy.Subscriber(self.ns + "/odom",  # subscribe to odom topic
                         Odometry, self.callbackposition)
        self.cmd_vel_pub = rospy.Publisher(
            self.ns + "/cmd_vel", Twist, queue_size=1)

    def callbacksonar(self, data):
        # DO NOT TOUCH
        self.sonar = data.range

    def get_target(self):  # to store target/goal position in variables
        return float(self.x_target), float(self.y_target)

    def get_sonar(self):
        # DO NOT TOUCH
        return float(self.sonar)

    # retrieve position (x,y) and yaw of the robot from odom
    def callbackposition(self, data):
        self.x_pos = data.pose.pose.position.x
        self.y_pos = data.pose.pose.position.y

        (roll_x, pitch_y, yaw_z) = self.euler_from_quaternion(
            data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)  # convert quaternions to actual angles
        self.yaw_z = yaw_z

    def get_position(self):  # to store position (x,y) and yaw into variables
        return self.x_pos, self.y_pos, self.yaw_z

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


def determine_LOS(x_pos, y_pos, x_target, y_target):
    """
        determine the Line Of Sight between the robot and the target (in a target-centered referential)
        the angle is =0 rad when the robot->target direction is aligned with the north
        LOS angle is increasing clockwise and goes from 0 to 2pi rad (both indicating the North)
    """
    LOS = 0
    x_relative = x_pos-x_target
    y_relative = y_pos-y_target
    if((x_relative >= 0) and (y_relative > 0)):  # 1st quadrant with target-centered reference
        LOS = math.pi+math.atan(abs(x_relative) / abs(y_relative))
    if((x_relative < 0) and (y_relative >= 0)):  # 2nd quadrant with target-centered reference
        LOS = math.pi/2+math.atan(abs(y_relative) / abs(x_relative))
    if((x_relative <= 0) and (y_relative < 0)):  # 3rd quadrant with target-centered reference
        LOS = math.atan(abs(x_relative) / abs(y_relative))
    if((x_relative > 0) and (y_relative <= 0)):  # 4th quadrant with target-centered reference
        LOS = 3*math.pi/2+math.atan(abs(y_relative) / abs(x_relative))
    return LOS


def run_demo():
    '''Main loop'''
    robot_name = rospy.get_param("~robot_name")
    robot = Robot(robot_name)
    print("Robot {} is starting".format(robot_name))

    [x_target, y_target] = robot.get_target()
    # [x_target, y_target] = [30, 20]  #arbitrary change target coordinates for multiple test situations
    print("Target at : {:+.2f} {:+.2f}".format(x_target, y_target))

    [x_pos, y_pos, yaw_z] = robot.get_position()  # starting position
    # previous_LOS = math.atan2(-(x_target-x_pos), -(y_target-y_pos))  # in radians
    previous_LOS = math.atan2((x_target-x_pos), (y_target-y_pos))  # in radians
    previous_LOS = math.atan2(
        (-x_target+x_pos), (-y_target+y_pos))  # in radians
    previous_HEADING = math.pi/2
    lambda_guid = 3  # PN guidance coeff: lambda>2 to ensure non-asymptotic terminal phase
    print("Line of Sight at start : {:+.2f} ".format(previous_LOS))

    angular_velocity = 0

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        # Write your strategy here ...

        """=====THE ROBOT IS GOING FULL SPEED, WE WILL ONLY CONTROL ITS HEADING ANGLE====="""

        linear_velocity = 2
        angular_velocity = 0
        # We are going at full speed since we will only control heading angle according the current line of site
        # We will just slow down to half speed when an obstacle is detected
        sonar = robot.get_sonar()

        # print("SONAR VALUE : {:.2f}".format(sonar))

        [x_pos, y_pos, yaw_z] = robot.get_position()  # retrieve position
        # print("Current position (x,y) : {:+.2f} {:+.2f}".format(x_pos, y_pos))
        # print("Current Angle (yaw in radians) : {:+.2f}".format(yaw_z))

        # GUIDANCE STRATEGY
        # try to maintain the heading angle in the line of sight of the target

        # print("Line of Sight : {:+.2f} ".format(LOS))
        # print("yaw : {:+.2f} ".format(yaw_z))
        # print("Heading : {:+.2f} ".format(math.pi/2+yaw_z))

        """=====DETERMINE THE ANGLE (LINE OF SIGHT) ALONG WHICH THE ROBOT SHOULD BE HEADING====="""

        LOS = determine_LOS(x_pos, y_pos, x_target, y_target)

        # we want the Heading angle to be equal to the LOS
        # but we control the yaw, Heading=pi/2 -yaw
        # ==> yaw = pi/2 - Heading
        required_yaw = math.pi/2 - LOS

        # store values for next iteration
        previous_LOS = LOS
        previous_HEADING = math.pi/2-yaw_z

        #print("Line of Sight (degrees)) : {:+.2f} ".format((LOS)))
        #print("Heading (degrees)) : {:+.2f} ".format((math.pi/2-yaw_z)))

        """=====MODIFY ANGULAR VELOCITY TO CONSTANTLY STEER THE HEADING ANGLE TO MATCH IT TO THE LOS====="""

        if yaw_z < required_yaw:
            # here we are not setting angular velocity to its maximum to avoid brutal changes of direction,
            # the idea is to have a relatively smooth trajectory all the way to the destination
            angular_velocity = 0.2  # steer left
        if yaw_z > required_yaw:
            angular_velocity = -0.2  # steer right

        """=====ENSURE WE DETECT AND AVOID COLLISION====="""

        # Priority given to collision avoidance so last check before setting speed
        # COLLISION AVOIDANCE STRATEGY
        sonar = robot.get_sonar()
        if(sonar < 4):  # in the case we detect an obstacle below range 4
            #print("DANGER AHEAD")
            linear_velocity = 1  # slow down a little to prevent immediate collision
            # turn (left or right depending on the sign) as quickly as possible to avoid the obstacle
            angular_velocity = -1
            # arbitrary choice on which side we are going to avoid collision

        """=====VERIFY IF ARRIVED====="""

        # check if we reached tarket zone
        [x_pos, y_pos, yaw_z] = robot.get_position()  # retrieve position
        if((x_target-2 < x_pos < x_target+2) and (y_target-2 < y_pos < y_target+2)):
            linear_velocity = 0  # stop here if we reached the goal
            angular_velocity = 0
            robot.set_speed_angle(linear_velocity, angular_velocity)
            print("TARGET REACHED")
            while(True):  # stay there forever as we reached the target
                pass  # just an instruction doing absolutely nothing

        # Finishing by publishing the desired speed. DO NOT TOUCH.
        robot.set_speed_angle(linear_velocity, angular_velocity)
        rate.sleep()


if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Strategy", anonymous=True)

    run_demo()

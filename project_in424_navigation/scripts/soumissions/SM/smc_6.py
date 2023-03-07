#!/usr/bin/env python3

import math as m
import rospy
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry


# Robot class, all parameters use for the robot itself
# and also all the method are represented here
class Robot:
    def __init__(self, robot_name):
        self.sonar = 0.0  # Sonar distance
        self.robot_name = robot_name
        self.ns = "/" + self.robot_name

        '''Listener and publisher'''
        rospy.Subscriber(self.ns + "/sensor/sonar_front",
                         Range, self.callbacksonar)
        self.cmd_vel_pub = rospy.Publisher(
            self.ns + "/cmd_vel", Twist, queue_size=1)

        rospy.Subscriber(self.ns + "/odom", Odometry,
                         self.callback_robot_position)

        # angle between orientation and point during dodge (degrees)
        self.beta = 0
        # angle between orientation and goal (degrees)
        self.alpha = 0
        self.phase = 0              # phase variable tp switch within the 5 phases
        self.distance_goal = 0      # distance between position and goal
        self.distance = 0
        self.distance_phase = False
        self.prev_obs_x = 1000      # init pos for decision
        self.prev_obs_y = 1000

        self.goal_pos = Point()
        self.goal_pos.x = rospy.get_param('/x_goal')    # get goal position
        self.goal_pos.y = rospy.get_param('/y_goal')

        self.last_pos = Point()     # for dodge method
        self.position = Pose()      # robot position
        self.x_pred = 0
        self.y_pred = 0

    def callback_robot_position(self, data):
        self.position = data.pose.pose

    def callbacksonar(self, data):
        # DO NOT TOUCH
        self.sonar = data.range

    def get_sonar(self):
        # DO NOT TOUCH
        return float(self.sonar)

    def get_coordinate(self):
        return self.position.position

    def get_orientation(self):
        return self.position.orientation

    def constraint(self, val, min=-2.0, max=2.0):
        if not val:
            return 0.

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

    # Calculate the orientation angle between the robot and a point
    # Input parameter : x and y pose of the point
    # Return parameter : alpha angle (degree)
    def set_alpha(self, x, y):
        """
        1. Calculate the slope of both vector (orientation, robot-goal)
        2. Calculate the angle between the vectors
        """

        quat_ori = self.get_orientation()
        ori = self.euler_from_quaternion(
            quat_ori.x, quat_ori.y, quat_ori.z, quat_ori.w)
        pos = self.get_coordinate()

        # 1.
        m_o = self.r2d(ori.z)  # slope of orientation
        dx = abs(pos.x - x)
        dy = abs(pos.y - y)

        # home-made atan2 function
        if (x > pos.x and y > pos.y):
            alpha = self.r2d(m.atan(dy/dx))
            if m_o >= alpha and m_o <= alpha + 180:
                alpha -= m_o
            else:
                if m_o <= 360 and m_o >= alpha + 180:
                    alpha += 360 - m_o
                else:
                    alpha -= m_o

        elif (x > pos.x and y < pos.y):
            alpha = 360 - self.r2d(m.atan(dy/dx))
            if m_o >= alpha - 180 and m_o <= alpha:
                alpha -= m_o
            else:
                if m_o >= 0 and m_o <= alpha - 180:
                    alpha += -360 - m_o
                else:
                    alpha -= m_o

        elif (x < pos.x and y > pos.y):
            alpha = 180 - self.r2d(m.atan(dy/dx))
            if m_o >= alpha and m_o <= alpha + 180:
                alpha -= m_o
            else:
                if m_o <= 360 and m_o <= alpha:
                    alpha -= m_o
                else:
                    alpha += 360 - m_o

        elif (x < pos.x and y < pos.y):
            alpha = self.r2d(m.atan(dy/dx)) + 180
            if m_o < alpha and m_o >= alpha - 180:
                alpha -= m_o
            else:
                if m_o >= alpha:
                    alpha -= m_o
                else:
                    alpha += -360 - m_o

        elif(dx == 0 and y > pos.y):
            alpha = 90
            if m_o >= 90 and m_o <= 270:
                alpha -= m_o
            else:
                if m_o < 90:
                    alpha -= m_o
                else:
                    alpha += 360 - m_o

        elif(dx == 0 and y < pos.y):
            alpha = 270
            if m_o >= 90 and m_o <= 270:
                alpha -= m_o
            else:
                if m_o > 270:
                    alpha -= m_o
                else:
                    alpha = -(m_o + 90)

        elif x < pos.x and dy == 0:
            alpha = 180 - m_o

        elif x > pos.x and dy == 0:
            alpha = 0
            if m_o <= 180:
                alpha -= m_o
            else:
                alpha = 360 - m_o

        return alpha

    def get_alpha(self):
        return self.alpha

    # Convert radian to degree
    # Input parameter : angle in radian
    # Return parameter : angle in degree
    def r2d(self, r):
        return r*180/m.pi

    # Convert degree to radian
    # Input parameter : angle in degree
    # Return parameter : angle in radian
    def d2r(self, d):
        return d*m.pi/180

    # Rotate the robot to a desired angle
    # Input parameter : angle in degree
    # Return parameter : none
    def turn_toward_goal(self, alpha):

        direction = 1 if alpha >= 0 else -1

        if abs(alpha) < 10 and abs(alpha) > 1:
            speed = 0.1

        elif (abs(alpha) <= 1 and self.phase == 0):
            speed = 0
            self.phase = 1

        elif (abs(alpha) <= 1 and self.phase == 3):
            speed = 0
            self.phase = 4

        else:
            speed = 0.3

        self.set_speed_angle(0.0, direction * speed)

    # Calculate the distance between a point and the position of robot
    # Input parameter : x and y pose of the point
    # Return parameter : the distance
    def distance_point(self, x, y):

        pos = self.get_coordinate()
        d = m.sqrt((pos.x - x)**2 + (pos.y - y)**2)

        return d

    # Move the robot forward till meeting an obstacle
    # Input parameter : none
    # Return parameter : none
    def move_forward(self):

        sonar = self.get_sonar()

        if sonar >= 5.0 or sonar == 0.0:
            speed = 2

        elif sonar >= 3.5:
            speed = 0.5
            pos = self.get_coordinate()
            self.last_pos = pos

        else:
            speed = 0.0
            self.phase = 2

        self.set_speed_angle(speed, 0.0)

    def conv_angle(self, alpha):

        if (alpha < 0):
            alpha = alpha + 360

        return alpha

    # Move the robot forward of a desired distance
    # Input parameter : none
    # Return parameter : none
    def move_distance(self):

        sonar = self.get_sonar()

        if (not(self.distance_phase)):
            self.distance = self.distance_point(
                self.last_pos.x, self.last_pos.y)
            self.distance_phase = True

        c = m.sqrt(5**2 + self.distance**2)
        error = c - self.distance_point(self.last_pos.x, self.last_pos.y)

        if (error >= 3 or c == 0.0):
            speed = 1.5

        elif (error <= 0.2 or sonar <= 3.25):
            speed = 0.0
            pos = self.get_coordinate()
            self.last_pos = pos
            self.distance_phase = False
            self.phase = 0

        else:
            speed = 0.5

        self.set_speed_angle(speed, 0.0)

    # Calculate two points at a precise distance and right or left
    # and choose the best one (considering the distance) to go
    # Input parameter : none
    # Return parameter :  predict postion (x_pred, y_pred)
    def dodge_obstacle(self):

        quat_ori = self.get_orientation()
        ori = self.euler_from_quaternion(
            quat_ori.x, quat_ori.y, quat_ori.z, quat_ori.w)
        pos = self.get_coordinate()

        # 1.
        alphar = ori.z - m.pi/2  # slope of orientation
        x_predr = pos.x + 5*m.cos(alphar)
        y_predr = pos.y + 5*m.sin(alphar)

        dr = m.sqrt((x_predr - self.goal_pos.x)**2 +
                    (y_predr - self.goal_pos.y)**2)

        alphal = ori.z + m.pi/2  # slope of orientation
        x_predl = pos.x + 5*m.cos(alphal)
        y_predl = pos.y + 5*m.sin(alphal)

        dl = m.sqrt((x_predl - self.goal_pos.x)**2 +
                    (y_predl - self.goal_pos.y)**2)

        """
        print("\n")
        print(pos.x, pos.y)
        print("dist right :"+str(dr))
        print("prev_obs_x: {:.2f}, prev_obs_y: {:.2f}".format(
            self.prev_obs_x, self.prev_obs_y))
        print("x_predr: {:.2f}, y_predr: {:.2f}".format(x_predr, y_predr))
        print("dist left :"+str(dl))
        print("x_predl: {:.2f}, y_predl: {:.2f}".format(x_predl, y_predl))
        print("\n")
        """
        self.phase = 3

        if (abs(x_predl - self.prev_obs_x) <= 3 and abs(y_predl - self.prev_obs_y) <= 3) \
                or (abs(x_predl) >= 45) or (abs(y_predl) >= 45):
            self.prev_obs_x, self.prev_obs_y = pos.x, pos.y

            return x_predr, y_predr

        elif (abs(x_predr - self.prev_obs_x) <= 3 and abs(y_predr - self.prev_obs_y) <= 3) \
                or (abs(x_predr) >= 31) or (abs(y_predr) >= 31):
            self.prev_obs_x, self.prev_obs_y = pos.x, pos.y

            return x_predl, y_predl

        elif dl >= dr:
            self.prev_obs_x, self.prev_obs_y = pos.x, pos.y

            return x_predr, y_predr

        else:
            self.prev_obs_x, self.prev_obs_y = pos.x, pos.y

            return x_predl, y_predl

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll_x = m.atan2(t0, t1)

        t2 = 2.0 * (w * y - z * x)
        t2 = 1.0 if t2 > 1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = m.asin(t2)

        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw_z = m.atan2(t3, t4)

        if (yaw_z < 0):
            yaw_z = yaw_z + 2*m.pi

        p = Point()
        p.x, p.y, p.z = roll_x, pitch_y, yaw_z
        return p  # in radians

# Main loop


def run_demo():

    robot_name = rospy.get_param("~robot_name")
    robot = Robot(robot_name)
    print("Robot {} is starting".format(robot_name))

    rate = rospy.Rate(20)
    # linear_velocity = 0
    # angular_velocity = 0.2

    while not rospy.is_shutdown():
        # Write your strategy here ...

        sonar = robot.get_sonar()
        pos = robot.get_coordinate()
        quat_ori = robot.get_orientation()
        ori = robot.euler_from_quaternion(
            quat_ori.x, quat_ori.y, quat_ori.z, quat_ori.w)

        robot.distance_goal = robot.distance_point(
            robot.goal_pos.x, robot.goal_pos.y)

        dx = abs(pos.x - robot.goal_pos.x)
        dy = abs(pos.y - robot.goal_pos.y)

        if(dx != 0 and dy != 0):

            robot.alpha = robot.set_alpha(robot.goal_pos.x, robot.goal_pos.y)

        if (robot.phase != 10):
            #print("SONAR VALUE : {:.2f}".format(sonar))
            print("POS:", pos)
            print("ORI Z :", ori.z*180/m.pi)
            #print("Alpha: ", robot.alpha)

        # Phase 0 : Rotate to be in front of the goal
        if ((dx != 0 and dy != 0) and sonar != 0.0 and robot.phase == 0):
            robot.turn_toward_goal(robot.get_alpha())
            # print("Alpha : " + str(robot.alpha))
            print("Phase 0 : Turn toward goal ")

        # Phase 1 : Move forward till meeting an object
        # if alpha angle to high, return to phase 0 to correct the angle derivation
        # if object detected, go to phase 2, dodge
        elif(robot.phase == 1):
            if abs(robot.alpha) > 2:
                robot.phase = 0

            else:
                robot.move_forward()
            print("Phase 1 : Move forward ")

        # Phase 2 : Object detected, calculate x_pred,y_pred
        # Making decision between turn right or left
        elif(robot.phase == 2):
            robot.x_pred, robot.y_pred = robot.dodge_obstacle()
            print("Phase 2 : Dodge ")

        # Phase 3 : Rotate toward predict point
        # Go to phase 4 to move after rotation
        elif(robot.phase == 3):
            robot.beta = robot.set_alpha(robot.x_pred, robot.y_pred)
            #print("Beta : " + str(robot.beta))
            robot.turn_toward_goal(robot.beta)
            print("Phase 3 : Turn ")

        # Phase 4 : Move 5 in the choosen direction
        # return to phase 0 if traveled distance
        # go to phase 2 (dodge) if object encountered
        elif(robot.phase == 4):
            robot.move_distance()
            print("Phase 4 : Move 5 ")

        #print("dist : " + str(robot.distance_goal))

        # Finale phase : Robot reach the goal
        if(robot.distance_goal <= 2 and robot.distance_goal != 0):
            robot.phase = 10
            robot.set_speed_angle(0.0, 1.0)
            print("Phase 10 : YEAAAAHHH ")

        # Finishing by publishing the desired speed. DO NOT TOUCH.
        #robot.set_speed_angle(0.0, 0.2)
        rate.sleep()


if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Strategy", anonymous=True)

    run_demo()

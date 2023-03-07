#!/usr/bin/env python3

# Importing Libraries
import math
import rospy

# Importing Ros Libraries to get Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry

# Importing time library to make the robot sleep
import time

# We get the goals coordinates
gx = rospy.get_param("/x_goal")
gy = rospy.get_param("/y_goal")

# We use this function to get the yaw of the Robot
def euler_from_quaternion(x, y, z, w):
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


class Robot:

    def __init__(self, robot_name):

        self.sonar = 0.0  # Sonar distance
        self.robot_name = robot_name
        self.ns = "/" + self.robot_name
        self.pos = [0, 0, 0] # Corresponds respectively to : X , Y , YAW

        '''Listener and publisher'''
        rospy.Subscriber(self.ns + "/sensor/sonar_front",
                         Range, self.callbacksonar) # Subscribe to the sonar 
        rospy.Subscriber(self.ns + "/odom", Odometry, self.callbackpos) # Subscribe to the Odometry

        self.cmd_vel_pub = rospy.Publisher(
            self.ns + "/cmd_vel", Twist, queue_size=1)

    # Student Code
    def callbackpos(self, data):

        # We call the position of the robot
        p = data.pose.pose.position
        x = p.x
        y = p.y

        # We call the orientation of the robot represented as quaternions. We use the euler_from_quaternion to get the roll pitch and yaw.
        v = data.pose.pose.orientation
        yaw = euler_from_quaternion(v.x, v.y, v.z, v.w)[2]

        # We only export the variables that we need for the following part
        self.pos = [x, y, yaw]

    # This function return the position of the robot as [x, y, yaw]
    def get_pos(self):
        return self.pos

    def callbacksonar(self, data):
        # DO NOT TOUCH

        self.sonar = data.range

    def get_sonar(self):
        # DO NOT TOUCH
        return float(self.sonar)

    def constraint(self, val, mi=-2.0, ma=2.0):
        # DO NOT TOUCH
        if val < mi:
            return mi
        if val > ma:
            return ma
        return val

    def set_speed_angle(self, linear_vel, angular_vel):
        # DO NOT TOUCH
        cmd_vel = Twist()
        cmd_vel.linear.x = self.constraint(linear_vel)
        cmd_vel.angular.z = self.constraint(angular_vel, mi=-1, ma=1)
        self.cmd_vel_pub.publish(cmd_vel)


def run_demo():
    '''Main loop'''
    robot_name = rospy.get_param("~robot_name")
    robot = Robot(robot_name)
    print("Robot {} is starting".format(robot_name))
    # We set a timer to 0 which will used for obstacles avoidance
    timer = 0

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():

        # We import the actual position of the robot
        [x, y, yaw] = robot.get_pos()

        # We calculate the relative position of the goal and its distance
        dx = gx-x
        dy = gy-y
        dist = (dx**2 + dy**2)**0.5

        # We calculate the angle between the reference of the robot and the goal
        ob = math.atan2(dy, dx)

        # We calculate the angular velocity by using the angle of the robot and the angle of the objective in the robot reference
        angular_velocity = ob-yaw
        linear_velocity = 2


        sonar = robot.get_sonar()
    
        # If the sonar is detecting and obstacle -> The Avoid state is triggered by a timer. 
        if sonar < 2.5:
            timer = time.time()

        if timer != 0:
            if time.time() - timer < 1:

                linear_velocity = 0
                angular_velocity = 1

            elif time.time() - timer < 4:

                linear_velocity = 2
                angular_velocity = 0
            else:
                timer = 0

        # If the robot is inside the goal 
        if dist < 2.5:

            linear_velocity = 0
            angular_velocity = 0

        
        print("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n----------------------------------------")
        print("Distance : " + str(((gx - x)**2 + (gy - y)**2)**0.5))
        print("----------------------------------------")
        print("Angle : target : " + str(ob*180/math.pi))
        print("Angle : robot  : " + str(yaw*180/math.pi))
        print("Diff : " + str((ob-yaw)*180/math.pi))
        print("----------------------------------------")
        print("SONAR VALUE : {:.2f}".format(sonar))
        print(f"POSITION : {robot.get_pos()}")
        print("----------------------------------------")

        # Finishing by publishing the desired speed. DO NOT TOUCH.
        robot.set_speed_angle(linear_velocity, angular_velocity)
        rate.sleep()


if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Strategy", anonymous=True)

    run_demo()

#!/usr/bin/env python3

# roslaunch project_in424_description simu.launch nbr_robot:=1
# roslaunch project_in424_navigation agent.launch name:=robot_1
import math
import rospy
import time
import set2_11_A_star as aS
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import WorldState

'''
current_i = initial_position_r
current_j = initial_position_c

start_f[2] = {current_i, current_j}
end_f[2] = {final_position_r, final_position_c}

'''


class Robot:
    def __init__(self, robot_name):
        self.sonar = 0.0  # Sonar distance
        self.robot_name = robot_name
        self.ns = "/" + self.robot_name
        self.x = 0.0
        self.y = 0.0
        self.i = 0.0
        self.j = 0.0
        self.x_o = 0.0
        self.y_o = 0.0
        self.z_o = 0.0
        self.w_o = 0.0
        self.list = []

        '''Listener and publisher'''
        rospy.Subscriber(self.ns + "/sensor/sonar_front",
                         Range, self.callbacksonar)
        rospy.Subscriber(self.ns + "/odom", Odometry, self.callbackGPS)
        self.cmd_vel_pub = rospy.Publisher(
            self.ns + "/cmd_vel", Twist, queue_size=1)

    def callbacksonar(self, data):
        # DO NOT TOUCH
        self.sonar = data.range

    def get_sonar(self):
        # DO NOT TOUCH
        return float(self.sonar)

    def callbackGPS(self, data):
        # DO NOT TOUCH
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y

        self.x_o = data.pose.pose.orientation.x
        self.y_o = data.pose.pose.orientation.y
        self.z_o = data.pose.pose.orientation.z
        self.w_o = data.pose.pose.orientation.w

    def get_GPS(self):
        # DO NOT TOUCH
        return (float(self.x), float(self.y))

    def get_rotation(self):
        # DO NOT TOUCH
        return (self.euler_from_quaternion(self.x_o, self.y_o, self.z_o, self.w_o))

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

        if -180 <= yaw_z*180/math.pi < -0.01:
            yaw_z = yaw_z+2*math.pi

        return roll_x, pitch_y, yaw_z  # in radians

    def turn_left(self, angular_velocity):
        print("turn left")
        roll, pitch, yaw = self.get_rotation()
        linear_velocity = 0

        goal = round((yaw*180/math.pi+90)/10)*10
        print("goal2=", goal)
        if round((yaw*180/math.pi)/10)*10 == 0:
            yaw = 360
        while(round(yaw*180/math.pi) != goal):

            if round((yaw*180/math.pi)/10)*10 == 360:
                self.set_speed_angle(linear_velocity, angular_velocity)

            elif(round(yaw*180/math.pi) > goal):
                self.set_speed_angle(linear_velocity, -angular_velocity)
            else:
                self.set_speed_angle(linear_velocity, angular_velocity)
            roll, pitch, yaw = self.get_rotation()
        self.set_speed_angle(0, 0)
        print("Yaw=", round(yaw*180/math.pi))

    def turn_right(self, angular_velocity):
        print("turn_right")
        roll, pitch, yaw = self.get_rotation()
        linear_velocity = 0

        print("Yaw=", round(yaw*180/math.pi))
        if round(yaw*180/math.pi) == 0:
            yaw = 2*math.pi
        goal = round((yaw*180/math.pi-90)/10)*10
        print("goal2=", goal)
        while(round(yaw*180/math.pi) != goal):

            if(round(yaw*180/math.pi) < goal):
                self.set_speed_angle(linear_velocity, angular_velocity)
            else:
                self.set_speed_angle(linear_velocity, -angular_velocity)
            roll, pitch, yaw = self.get_rotation()
        self.set_speed_angle(0, 0)

    def move(self, dist, linear_velocity):
        roll, pitch, yaw = self.get_rotation()
        GPS_x, GPS_y = self.get_GPS()
        angular_velocity = 0
        if round((yaw*180/math.pi)/10)*10 == 360:
            yaw = 0
        if (round((yaw*180/math.pi)/10)*10 == 0):
            goal = round(GPS_x+dist, 1)

            while(round(GPS_x, 1) != goal):

                GPS_x, GPS_y = self.get_GPS()

                if(round(GPS_x, 1) > goal):
                    self.set_speed_angle(-linear_velocity, angular_velocity)
                else:
                    self.set_speed_angle(linear_velocity, angular_velocity)

        if (round((yaw*180/math.pi)/10)*10 == -180 or round((yaw*180/math.pi)/10)*10 == 180):
            goal = round(GPS_x-dist, 1)

            while(round(GPS_x, 1) != goal):

                GPS_x, GPS_y = self.get_GPS()

                if(round(GPS_x, 1) < goal):
                    self.set_speed_angle(-linear_velocity, angular_velocity)
                else:
                    self.set_speed_angle(linear_velocity, angular_velocity)

        if (round((yaw*180/math.pi)/10)*10 == 90):
            goal = round(GPS_y+dist, 1)

            while(round(GPS_y, 1) != goal):

                GPS_x, GPS_y = self.get_GPS()

                if(round(GPS_y, 1) > goal):
                    self.set_speed_angle(-linear_velocity, angular_velocity)
                else:
                    self.set_speed_angle(linear_velocity, angular_velocity)

        if (round((yaw*180/math.pi)/10)*10 == 270):
            goal = round(GPS_y-dist,1)

            while(round(GPS_y, 1) != goal):

                GPS_x, GPS_y = self.get_GPS()

                if(round(GPS_y, 1) < goal):
                    self.set_speed_angle(-linear_velocity, angular_velocity)
                else:
                    self.set_speed_angle(linear_velocity, angular_velocity)

        self.set_speed_angle(0, 0)

    def find_i_j(self, rows, cols):
        GPS_x, GPS_y = self.get_GPS()
        for i in range(rows):
            var1 = rows/2 - (i)
            var2 = rows/2 - (i + 1)
            for j in range(cols):
                var3 = -cols/2 + (j)
                var4 = -cols/2 + (j + 1)

                if (var2 <= GPS_y < var1 and var3 <= GPS_x < var4):
                    self.i = i
                    self.j = j
                    m_i = (var3+var4)/2
                    m_j = (var1+var2)/2
        return (int(self.i), int(self.j), float(m_i), float(m_j))

    def mid(self, rows, cols):
        GPS_x, GPS_y = self.get_GPS()
        e_i, e_j, mid_i, mid_j = find_i_j(GPS_x, GPS_y, rows, cols)
        while(round(GPS_x, 2) == round(mid_i, 1)):
            if round(GPS_x, 2) > round(mid_i, 1):
                self.set_speed_angle(-0.5, 0)
                GPS_x, GPS_y = self.get_GPS()
            else:
                self.set_speed_angle(0.5, 0)
                GPS_x, GPS_y = self.get_GPS()

        self.turn_left(0.09)

        while(round(GPS_y, 2) == round(mid_j, 1)):
            if round(GPS_x, 2) > round(mid_j, 1):
                self.set_speed_angle(-0.5, 0)
                GPS_x, GPS_y = self.get_GPS()
            else:
                self.set_speed_angle(0.5, 0)
                GPS_x, GPS_y = self.get_GPS()

        self.turn_right(0.09)


def find_i_j(GPS_x, GPS_y, rows, cols):
    i_s = 0
    j_s = 0
    for i in range(rows):
        var1 = rows/2 - (i)
        var2 = rows/2 - (i + 1)
        for j in range(cols):
            var3 = -cols/2 + (j)
            var4 = -cols/2 + (j + 1)

            if (var2 <= GPS_y < var1 and var3 <= GPS_x < var4):
                i_s = i
                j_s = j
                m_i = (var3+var4)/2
                m_j = (var1+var2)/2
    return (int(i_s), int(j_s), float(m_i), float(m_j))


def printmatrix(maze1):
    for i in range(len(maze1)):
        for j in range(len(maze1[0])):
            print(str(maze1[i][j])+" ", end="")
        print("\n")


def printmatrix33(maze1, i, j):
    print(str(maze1[i-1][j-1])+" ", end="")
    print(str(maze1[i-1][j])+" ", end="")
    print(str(maze1[i-1][j+1])+" ", end="")
    print("\n")
    print(str(maze1[i][j-1])+" ", end="")
    print(str(maze1[i][j])+" ", end="")
    print(str(maze1[i][j+1])+" ", end="")
    print("\n")
    print(str(maze1[i+1][j-1])+" ", end="")
    print(str(maze1[i+1][j])+" ", end="")
    print(str(maze1[i+1][j+1])+" ", end="")
    print("\n")


def run_demo(x_goal, y_goal):
    '''Main loop'''
    robot_name = rospy.get_param("~robot_name")
    robot = Robot(robot_name)
    print("Robot {} is starting".format(robot_name))
    rows = 68
    cols = 94
    distance_face = 5
    obstacle = aS.matric(rows, cols)
    location = aS.matric(rows, cols)
    rate = rospy.Rate(2)
    deb = 0
    deb1 = 0
    start_spot = []
    end_spot = []
    e_i, e_j, mid_i, mid_j = find_i_j(x_goal, y_goal, rows, cols)
    end_spot.append(e_i)
    end_spot.append(e_j)
    end_spot.append(mid_i)
    end_spot.append(mid_j)
    print(str(end_spot))
    while not rospy.is_shutdown():
        # Write your strategy here ...
        time.sleep(1)
        linear_velocity = 1
        angular_velocity = 0.2
        run_dist=0.5
        sonar = robot.get_sonar()
        roll, pitch, yaw = robot.get_rotation()
        GPS_x, GPS_y = robot.get_GPS()
        current_i, current_j, mid_i, mid_j = robot.find_i_j(rows, cols)
        if deb1 == 0:
            start_spot.append(current_i)
            start_spot.append(current_j)
            deb1 = 1
        location, obstacle = aS.astar(
            obstacle, location, start_spot, end_spot, rows, cols)
        '''
        print("SONAR VALUE : {:.2f}".format(sonar))
        print("X : {:.2f}".format(GPS_x))
        print("Y : {:.2f}".format(GPS_y))
        print("Yaw : {:.2f}".format(round(yaw*180/math.pi)))
        print("\n("+str(current_i)+","+str(current_j)+")\n")
        print("\n("+str(mid_i)+","+str(mid_j)+")\n")
        '''
        # robot.mid(rows,cols)
        while (current_i != e_i or current_j != e_j):
            location[current_i][current_j] = 4
            time.sleep(0.25)
            if (location[current_i][current_j + 1] == 1):

                if (round((yaw*180/math.pi)/10)*10 == 0):

                    distance = robot.get_sonar()
                    #print("SONAR VALUE : {:.2f}".format(sonar))
                    if (distance < distance_face and deb == 0):

                        obstacle[current_i][current_j+5] = 2
                        start_spot[0] = current_i
                        start_spot[1] = current_j
                        location, obstacle = aS.astar(
                            obstacle, location, start_spot, end_spot, rows, cols)

                        deb = 1

                    else:
                        robot.move(run_dist, linear_velocity)
                        #print("j'ai avance")
                        current_i, current_j, mid_i, mid_j = robot.find_i_j(
                            rows, cols)
                        roll, pitch, yaw = robot.get_rotation()
                        #print("\n("+str(current_i)+","+str(current_j)+")\n")
                        deb = 0

                elif (round((yaw*180/math.pi)/10)*10 == 90):
                    distance = robot.get_sonar()
                    if (distance < distance_face and deb == 0):

                        obstacle[current_i][current_j+5] = 2
                        start_spot[0] = current_i
                        start_spot[1] = current_j
                        location, obstacle = aS.astar(
                            obstacle, location, start_spot, end_spot, rows, cols)

                        deb = 1

                    else:

                        robot.turn_right(angular_velocity)
                        robot.move(run_dist, linear_velocity)
                        #print("j'ai avance")
                        current_i, current_j, mid_i, mid_j = robot.find_i_j(
                            rows, cols)
                        #print("\n("+str(current_i)+","+str(current_j)+")\n")
                        roll, pitch, yaw = robot.get_rotation()
                        deb = 0
                

                elif (round((yaw*180/math.pi)/10)*10 == 270):
                    #print("location[current_i][current_j + 1];270")
                    distance = robot.get_sonar()
                    if (distance < distance_face and deb == 0):
                        obstacle[current_i][current_j+5] = 2
                        start_spot[0] = current_i
                        start_spot[1] = current_j
                        location, obstacle = aS.astar(
                            obstacle, location, start_spot, end_spot, rows, cols)

                        deb = 1

                    else:

                        robot.turn_left(angular_velocity)
                        robot.move(run_dist, linear_velocity)
                        #print("j'ai avance")
                        current_i, current_j, mid_i, mid_j = robot.find_i_j(
                            rows, cols)
                        #print("\n("+str(current_i)+","+str(current_j)+")\n")
                        roll, pitch, yaw = robot.get_rotation()
                        deb = 0
                else:
                    #print("Problem1")
                    current_i, current_j, mid_i, mid_j = robot.find_i_j(rows, cols)
                    start_spot[0] = current_i
                    start_spot[1] = current_j
                    location, obstacle = aS.astar(
                        obstacle, location, start_spot, end_spot, rows, cols)

            elif(location[current_i + 1][current_j] == 1):

                if (round((yaw*180/math.pi)/10)*10 == 0):
                    #print("location[current_i+1][current_j ];0")
                    distance = robot.get_sonar()

                    if (distance < distance_face and deb == 0):

                        obstacle[current_i+5][current_j] = 2
                        start_spot[0] = current_i
                        start_spot[1] = current_j
                        location, obstacle = aS.astar(
                            obstacle, location, start_spot, end_spot, rows, cols)

                        deb = 1
                    else:
                        robot.turn_right(angular_velocity)
                        robot.move(run_dist, linear_velocity)
                        #print("j'ai avance")
                        current_i, current_j, mid_i, mid_j = robot.find_i_j(
                            rows, cols)
                        #print("\n("+str(current_i)+","+str(current_j)+")\n")
                        roll, pitch, yaw = robot.get_rotation()
                        deb = 0

                elif (round((yaw*180/math.pi)/10)*10 == 270):
                    #print("location[current_i+1][current_j ];270")
                    distance = robot.get_sonar()
                    if (distance < distance_face and deb == 0):
                        obstacle[current_i+5][current_j] = 2
                        start_spot[0] = current_i
                        start_spot[1] = current_j
                        location, obstacle = aS.astar(
                            obstacle, location, start_spot, end_spot, rows, cols)
                        deb = 1

                    else:

                        robot.move(run_dist, linear_velocity)
                        #print("j'ai avance")
                        current_i, current_j, mid_i, mid_j = robot.find_i_j(
                            rows, cols)
                        #print("\n("+str(current_i)+","+str(current_j)+")\n")
                        roll, pitch, yaw = robot.get_rotation()
                        deb = 0

                elif (round((yaw*180/math.pi)/10)*10 == 180):
                    #print("location[current_i+1][current_j ];180")
                    distance = robot.get_sonar()
                    if (distance < distance_face and deb == 0):
                        obstacle[current_i+5][current_j] = 2
                        start_spot[0] = current_i
                        start_spot[1] = current_j
                        location, obstacle = aS.astar(
                            obstacle, location, start_spot, end_spot, rows, cols)

                        deb = 1

                    else:

                        robot.turn_left(angular_velocity)
                        robot.move(run_dist, linear_velocity)
                        current_i, current_j, mid_i, mid_j = robot.find_i_j(
                            rows, cols)
                        #print("\n("+str(current_i)+","+str(current_j)+")\n")
                        roll, pitch, yaw = robot.get_rotation()
                        deb = 0
                else:
                    #print("Problem1")
                    current_i, current_j, mid_i, mid_j = robot.find_i_j(rows, cols)
                    start_spot[0] = current_i
                    start_spot[1] = current_j
                    location, obstacle = aS.astar(
                        obstacle, location, start_spot, end_spot, rows, cols)
                    deb = 0

            elif(location[current_i - 1][current_j] == 1):

                if (round((yaw*180/math.pi)/10)*10 == 0):
                    #print("location[current_i-1][current_j ];0")
                    distance = robot.get_sonar()

                    if (distance < distance_face and deb == 0):

                        obstacle[current_i-5][current_j] = 2
                        start_spot[0] = current_i
                        start_spot[1] = current_j
                        location, obstacle = aS.astar(
                            obstacle, location, start_spot, end_spot, rows, cols)

                        deb = 1
                    else:
                        robot.turn_left(angular_velocity)
                        robot.move(run_dist, linear_velocity)
                        current_i, current_j, mid_i, mid_j = robot.find_i_j(
                            rows, cols)
                        #print("\n("+str(current_i)+","+str(current_j)+")\n")
                        roll, pitch, yaw = robot.get_rotation()
                        deb = 0

                elif (round((yaw*180/math.pi)/10)*10 == 90):
                    #print("location[current_i-1][current_j ];90")
                    distance = robot.get_sonar()
                    if (distance < distance_face and deb == 0):

                        obstacle[current_i-5][current_j] = 2
                        start_spot[0] = current_i
                        start_spot[1] = current_j
                        location, obstacle = aS.astar(
                            obstacle, location, start_spot, end_spot, rows, cols)

                        deb = 1

                    else:

                        robot.move(run_dist, linear_velocity)
                        current_i, current_j, mid_i, mid_j = robot.find_i_j(
                            rows, cols)
                        #print("\n("+str(current_i)+","+str(current_j)+")\n")
                        roll, pitch, yaw = robot.get_rotation()
                        deb = 0

                elif (round((yaw*180/math.pi)/10)*10 == 180):
                    #print("location[current_i-1][current_j ];0")
                    distance = robot.get_sonar()
                    if (distance < distance_face and deb == 0):
                        obstacle[current_i+5][current_j] = 2
                        start_spot[0] = current_i
                        start_spot[1] = current_j
                        location, obstacle = aS.astar(
                            obstacle, location, start_spot, end_spot, rows, cols)

                        deb = 1

                    else:

                        robot.turn_right(angular_velocity)
                        robot.move(run_dist, linear_velocity)
                        current_i, current_j, mid_i, mid_j = robot.find_i_j(
                            rows, cols)
                        #print("\n("+str(current_i)+","+str(current_j)+")\n")
                        roll, pitch, yaw = robot.get_rotation()
                        deb = 0
                else:
                   # print("Problem1")
                    current_i, current_j, mid_i, mid_j = robot.find_i_j(rows, cols)
                    start_spot[0] = current_i
                    start_spot[1] = current_j
                    location, obstacle = aS.astar(
                        obstacle, location, start_spot, end_spot, rows, cols)
                    deb = 0
                    

            elif(location[current_i][current_j - 1] == 1):

                if (round((yaw*180/math.pi)/10)*10 == 90):
                    #print("location[current_i][current_j-1];90")
                    distance = robot.get_sonar()
                    if (distance < distance_face and deb == 0):

                        obstacle[current_i][current_j-5] = 2
                        start_spot[0] = current_i
                        start_spot[1] = current_j
                        location, obstacle = aS.astar(
                            obstacle, location, start_spot, end_spot, rows, cols)

                        deb = 1

                    else:
                        robot.turn_left(angular_velocity)
                        robot.move(run_dist, linear_velocity)
                        current_i, current_j, mid_i, mid_j = robot.find_i_j(
                            rows, cols)
                        #print("\n("+str(current_i)+","+str(current_j)+")\n")
                        roll, pitch, yaw = robot.get_rotation()
                        deb = 0

                elif (round((yaw*180/math.pi)/10)*10 == 270):
                    #print("location[current_i][current_j-1];270")
                    distance = robot.get_sonar()
                    if (distance < distance_face and deb == 0):
                        obstacle[current_i+5][current_j] = 2
                        start_spot[0] = current_i
                        start_spot[1] = current_j
                        location, obstacle = aS.astar(
                            obstacle, location, start_spot, end_spot, rows, cols)

                        deb = 1

                    else:

                        robot.turn_right(angular_velocity)
                        robot.move(run_dist, linear_velocity)
                        current_i, current_j, mid_i, mid_j = robot.find_i_j(
                            rows, cols)
                        #print("\n("+str(current_i)+","+str(current_j)+")\n")
                        roll, pitch, yaw = robot.get_rotation()
                        deb = 0

                elif (round((yaw*180/math.pi)/10)*10 == 180):
                    #print("location[current_i][current_j-1];180")
                    distance = robot.get_sonar()
                    if (distance < distance_face and deb == 0):
                        obstacle[current_i+5][current_j] = 2
                        start_spot[0] = current_i
                        start_spot[1] = current_j
                        location, obstacle = aS.astar(
                            obstacle, location, start_spot, end_spot, rows, cols)

                        deb = 1

                    else:

                        robot.move(run_dist, linear_velocity)
                        current_i, current_j, mid_i, mid_j = robot.find_i_j(
                            rows, cols)
                        #print("\n("+str(current_i)+","+str(current_j)+")\n")
                        roll, pitch, yaw = robot.get_rotation()
                        deb = 0
                else:
                    print("Problem1")
                    current_i, current_j, mid_i, mid_j = robot.find_i_j(rows, cols)
                    start_spot[0] = current_i
                    start_spot[1] = current_j
                    location, obstacle = aS.astar(
                        obstacle, location, start_spot, end_spot, rows, cols)

            else:
                print("Problem2")
                current_i, current_j, mid_i, mid_j = robot.find_i_j(rows, cols)
                start_spot[0] = current_i
                start_spot[1] = current_j
                location, obstacle = aS.astar(
                    obstacle, location, start_spot, end_spot, rows, cols)
                deb = 0

    rate.sleep()


if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Strategy", anonymous=True)

    x_goal = rospy.get_param('/x_goal')
    y_goal = rospy.get_param('/y_goal')
    print("X_goal : {:.2f}".format(x_goal))
    print("Y_goal : {:.2f}".format(y_goal))

    run_demo(x_goal, y_goal)

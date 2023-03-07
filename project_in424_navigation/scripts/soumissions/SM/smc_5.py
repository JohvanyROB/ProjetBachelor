#!/usr/bin/env python3

import math
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry


class Robot:
    def __init__(self, robot_name):
        self.sonar = 0.0  # Sonar distance
        self.pose = [0.0, 0.0, 0.0]  # Position
        self.ang = [0.0, 0.0, 0.0, 0.0]  # Angle
        self.robot_name = robot_name
        self.ns = "/" + self.robot_name

        '''Listener and publisher'''
        rospy.Subscriber(self.ns + "/sensor/sonar_front",
                         Range, self.callbacksonar)
        rospy.Subscriber(self.ns + "/odom", Odometry, self.callbackpose)
        rospy.Subscriber(self.ns + "/odom", Odometry, self.callbackorientation)
        self.cmd_vel_pub = rospy.Publisher(
            self.ns + "/cmd_vel", Twist, queue_size=1)

    def callbacksonar(self, data):
        # DO NOT TOUCH
        self.sonar = data.range

    def get_sonar(self):
        # DO NOT TOUCH
        return float(self.sonar)

    def callbackpose(self, data):
        self.pose[0] = data.pose.pose.position.x
        self.pose[1] = data.pose.pose.position.y
        self.pose[2] = data.pose.pose.position.z

    def get_pose(self):
        return self.pose

    def callbackorientation(self, data):
        self.ang[0] = data.pose.pose.orientation.x
        self.ang[1] = data.pose.pose.orientation.y
        self.ang[2] = data.pose.pose.orientation.z
        self.ang[3] = data.pose.pose.orientation.w

    def get_ang(self):
        return self.ang

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
        Convert a quaternion into euler angles (roll, pitch, bot_dir)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        bot_dir is rotation around z in radians (counterclockwise)
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
        bot_dir_z = math.atan2(t3, t4)

        return roll_x, pitch_y, bot_dir_z  # in radians


def dif_angle(a1, a2):
    # return the difference bewtween 2 angle
    # this function was necessary because we work with angle that range from -180 to 180
    # example the dif between -170 and 170 is 20, not 340
    if a1*a2 >= 0:
        return abs(a1-a2)
    else:
        if a1 > 0:
            da180_1, da180_2 = 180-a1, 180+a2
            da0_1, da0_2 = a1, -a2
            da180 = da180_1+da180_2
            da0 = da0_1+da0_2

        else:
            da180_1, da180_2 = 180+a1, 180-a2
            da0_1, da0_2 = -a1, a2
            da180 = da180_1+da180_2
            da0 = da0_1+da0_2

        if da180 < da0:
            return da180
        else:
            return da0


def left_or_right(dirgoal, dirbot):
    # allows to chose which direction to rotate to reach an angle goal
    # true = turn left

    if dirgoal >= 0:
        if dirbot >= 0:
            if dirbot <= dirgoal:
                return True
            else:
                return False
        else:
            if 180+dirbot >= dirgoal:
                return True
            else:
                return False
    else:
        if dirbot <= 0:
            if dirbot <= dirgoal:
                return True
            else:
                return False
        else:
            if dirbot-180 >= dirgoal:
                return True
            else:
                return False


def calcul_angle_comprehensible(goal_dir, bot_dir, ecartx, ecarty):
    # we prefered working with degrees and not radians
    # moreover, it allowed us to correct the mistake
    # on the right side where angles are "wrong"
    bot_dird = np.rad2deg(bot_dir)

    goal_dird = np.rad2deg(goal_dir)
    print(goal_dird)
    if ecartx < 0:
        if ecarty < 0:
            goal_dird -= 180
        else:
            goal_dird += 180
    """
    #Le coté gauche n'a pas besoin de changement
    else:
        if ecarty < 0:
            #print("HAUT GAUCHE")
            pass

        else:
            #print("BAS GAUCHE")
            pass
    """
    #print("goal_dir et bot_dir", goal_dird, bot_dird)
    return goal_dird, bot_dird


def reorientation(angle_goal, angle_bot, linear_vel=False):
    # return the angular velocity depending on the dif of the angles
    # the further you are, the faster you rotate
    cd = left_or_right(-angle_goal, -angle_bot)

    if dif_angle(angle_goal, angle_bot) < 2:
        angular_velocity = 0
        linear_velocity = 2
        go = True

    elif dif_angle(angle_goal, angle_bot) < 5:
        angular_velocity = ang_vel(cd, 0.05)
        linear_velocity = 2
        go = False

    elif dif_angle(angle_goal, angle_bot) < 10:
        angular_velocity = ang_vel(cd, 0.15)
        linear_velocity = 1.8
        go = False

    elif dif_angle(angle_goal, angle_bot) < 40:
        angular_velocity = ang_vel(cd, 0.3)
        linear_velocity = 1
        go = False

    elif dif_angle(angle_goal, angle_bot) < 80:
        angular_velocity = ang_vel(cd, 0.6)
        linear_velocity = 1
        go = False

    else:
        angular_velocity = ang_vel(cd, 1)
        linear_velocity = 0
        go = False

    if linear_vel:
        return linear_velocity, angular_velocity
    else:
        return go, angular_velocity


def norme(pos1, pos2):
    # calculate the norm
    dist = ((pos1[0]-pos2[0])**2+(pos1[1]-pos2[1])**2)**0.5
    return dist


def on_spot(ecartx, ecarty):
    # check if the bot is on the goal
    # and return True if it's the case
    if abs(ecartx) < 0.5 and abs(ecarty) < 0.5:
        print("Victory!")
        return True
    else:
        return False


def ang_vel(direction, coef):
    # decide the rotation speed of the bot
    # and the rotation direction
    if direction:
        angular_velocity = -coef
    else:
        angular_velocity = coef
    return angular_velocity


def run_demo():
    '''Main loop'''
    robot_name = rospy.get_param("~robot_name")
    robot = Robot(robot_name)
    print("Robot {} is starting".format(robot_name))

    rate = rospy.Rate(2)
    # we initialise all the variables
    new_obstacle = True
    contournement = False
    verif_gauche, verif_droite = False, False
    stop_gauche, stop_droite = False, False
    angle_found = False
    orientation_esquive = False
    wait_til_back = False
    check_if_goal_clear = False
    wait_til_dir = False
    come_from = False
    reoriented_once = False

    cf = 0
    distance_max = 3
    init_dir = 0
    premiere_boucle = True
    angular_velocity = 0
    linear_velocity = 2
    while not rospy.is_shutdown():

        print("\n")
        # Write your strategy here ...
        sonar = robot.get_sonar()
        pose = robot.get_pose()
        ang = robot.get_ang()
        roll_x, pitch_y, true_z = robot.euler_from_quaternion(
            ang[0], ang[1], ang[2], ang[3])

        xg = rospy.get_param("/x_goal")
        yg = rospy.get_param("/y_goal")
        deltax = xg - round(pose[0], 2)
        deltay = yg - round(pose[1], 2)

        # we calculate the distance to the goal
        goal_dist = ((deltax**2)+(deltay)**2)**0.5
        try:  # because it's rounded, it can ==0, so we prevent that case
            true_goal_dir = np.arctan(deltay/deltax)

        except:
            pass

        goal_dir, bot_dir_z = calcul_angle_comprehensible(
            true_goal_dir, true_z, deltax, deltay)  # transform the angles to degress

        if premiere_boucle:
            # the first loop might bug if we relaunch the program and not the simulation
            premiere_boucle = False
        elif on_spot(deltax, deltay):
            print("The BOT has reached the destination")
            linear_velocity = 0
            angular_velocity = 0

        else:

            if contournement:

                # decide if the bot must avoid an obstacle by turning left or right
                bar = 10  # bar = best angle range,  basically check a + or - 10 degrees
                # arround an angle to see if we can have an angle with a higher sonar

                if wait_til_back:
                    # wait until back to the starting angle
                    reoriented, angular_velocity = reorientation(
                        init_dir, bot_dir_z)
                    linear_velocity = 0
                    if reoriented:
                        wait_til_back = False
                        angular_velocity = ang_vel(wait_til_dir, 0.3)

                elif sonar >= distance_max and not angle_found:
                    if not verif_gauche:
                        # distance_max is the treshold, below that we consider the obstacle is too close
                        # and thus cannot dodge by moving toward that angle

                        esquive_gauche = [sonar, bot_dir_z]
                        best_esquive_gauche = [sonar, bot_dir_z]

                        angular_velocity = ang_vel(False, 0.1)
                        verif_gauche = True
                        stop_gauche = False

                    elif verif_gauche and not stop_gauche:
                        if esquive_gauche[1] - bar < bot_dir_z < esquive_gauche[1] + bar:
                            if sonar > best_esquive_gauche[0]:
                                # We check arround if there's a better angle (+ or -10 deg)
                                best_esquive_gauche = [sonar, bot_dir_z]

                        if best_esquive_gauche[0] >= 5:
                            # if it's already >=5, no need to look further

                            if not come_from:
                                # except if we find that we come from this direction
                                angle_found = True
                                best_esquive_droite = [0, 720]
                            else:
                                if dif_angle(cf, best_esquive_gauche[1]) < 140:
                                    angle_found = True
                                    best_esquive_droite = [0, 720]
                                else:
                                    stop_gauche = True
                                    wait_til_back = True
                                    wait_til_dir = True

                        else:
                            # reinitialisation of the variables

                            stop_gauche = True
                            wait_til_back = True
                            wait_til_dir = True

                    # idem on the right side
                    elif not verif_droite:
                        esquive_droite = [sonar, bot_dir_z]
                        best_esquive_droite = [sonar, bot_dir_z]

                        verif_droite = True
                        angular_velocity = ang_vel(True, 0.1)
                        stop_droite = False

                    elif verif_droite and not stop_droite:
                        if esquive_droite[1] - bar < bot_dir_z < esquive_droite[1] + bar:
                            if sonar > best_esquive_droite[0]:
                                best_esquive_droite = [sonar, bot_dir_z]
                            else:
                                angular_velocity = 0
                                stop_droite = True
                        else:
                            angular_velocity = 0
                            stop_droite = True
                    else:
                        # reinitialisation of the variables
                        verif_gauche = False
                        verif_droite = False

                        angle_found = True

                if angle_found:

                    # now we check which direction between the two (if two are given) is the best one
                    # as well a reinitialising some variables
                    contournement = False

                    verif_gauche = False
                    verif_droite = False
                    angle_found = False
                    orientation_esquive = True

                    check_correction = True

                    pose_esq = [pose[0], pose[1]]

                    angle_d = dif_angle(best_esquive_droite[1], init_dir)
                    angle_g = dif_angle(best_esquive_gauche[1], init_dir)

                    no_issues = False
                    if come_from:
                        # we check if we don't come from one of these directions
                        # in which case we don't take it (except if it's both)
                        if dif_angle(cf, best_esquive_droite[1]) > 120:
                            cfdroite = True
                        else:
                            cfdroite = False

                        if dif_angle(cf, best_esquive_gauche[1]) > 120:
                            cfgauche = True
                        else:
                            cfgauche = False

                        if cfgauche and cfdroite:
                            # both are from where we are comming
                            if bot_dir_z < 0:
                                ori_esq = bot_dir_z + 180
                            else:
                                ori_esq = bot_dir_z - 180
                            dist_esq = 5

                        else:
                            if cfgauche:
                                # we come from the left direction
                                ori_esq = best_esquive_droite[1]
                                dist_esq = best_esquive_droite[0]
                                cd = True

                            elif cfdroite:
                                # we come from the right direction
                                ori_esq = best_esquive_gauche[1]
                                dist_esq = best_esquive_gauche[0]
                                cd = False

                            else:
                                no_issues = True
                                # no trouble with these angles

                    if not come_from or no_issues:
                        # we prioritize the direction where the sonar is the bigger one
                        if best_esquive_droite[0] > best_esquive_gauche[0]:
                            # we rotate on the right
                            cd = True  # cd = contournement direction
                            ori_esq = best_esquive_droite[1]
                            dist_esq = best_esquive_droite[0]
                            if dist_esq < 1.5:
                                ori_esq += 10  # if an obstacle is too close it might block the wheels
                                # but it rarely get to this point

                        elif best_esquive_droite[0] < best_esquive_gauche[0]:
                            # we rotate on the left
                            cd = False
                            ori_esq = best_esquive_gauche[1]
                            dist_esq = best_esquive_gauche[0]
                            if dist_esq < 1.5:
                                ori_esq -= 10
                        else:
                            # if both angles have the same sonar
                            # we take the one which is closer to the direction we want to go to

                            if angle_d < angle_g:
                                # we rotate on the right
                                cd = True  # cd = contournement direction
                                ori_esq = best_esquive_droite[1]
                                dist_esq = best_esquive_droite[0]
                                if dist_esq < 1.5:
                                    ori_esq += 10
                            else:
                                # we rotate on the left
                                cd = False  # cd = contournement direction
                                ori_esq = best_esquive_gauche[1]
                                dist_esq = best_esquive_gauche[0]
                                if dist_esq < 1.5:
                                    ori_esq += 10
                    # no we just give some initial angular velocity ot reach the angle
                    # but it will quickly be changed on the next loop
                    # it only allows to start "earlier"
                    if dif_angle(ori_esq, bot_dir_z) < 20:
                        angular_velocity = ang_vel(cd, 0.2)
                    else:
                        angular_velocity = ang_vel(cd, 0.4)

            elif orientation_esquive:
                # we orient the bot to the direction chosen before

                # on the first iteration, we lock the rotating direction for some
                # later collision dodge
                if not reoriented_once:
                    reoriented, angular_velocity = reorientation(
                        ori_esq, bot_dir_z)
                    if reoriented:
                        reoriented_once = True
                        previous_goal_dist = goal_dist

                if reoriented_once:

                    dist_til_esq = ((pose[0]-pose_esq[0])
                                    ** 2 + (pose[1]-pose_esq[1])**2)**0.5

                    dif_goal_dist = previous_goal_dist - goal_dist
                    previous_goal_dist = goal_dist
                    if sonar >= 5 and dif_goal_dist > -0.05:
                        # the further we are, the faster we go (forward)
                        angular_velocity = 0
                        linear_velocity = 2

                    elif 2.5 < sonar < 5 and dif_angle(bot_dir_z, ori_esq) < 90 and dif_goal_dist > -0.05:
                        # if it doesn't deviate too much from the current trajectory,
                        # we can try to avoid the thing in front of the bot
                        # it works well when it's a wall

                        if check_correction:
                            check_correction = False
                            prec_sonar = sonar
                            correction_dir = True
                            linear_velocity = 0
                            angular_velocity = ang_vel(correction_dir, 0.3)

                        else:  # if the sonar , we change the direction, this only happen at the beggining

                            if sonar > prec_sonar:
                                correction_dir = True
                            else:
                                correction_dir = False
                            linear_velocity = 1
                            angular_velocity = ang_vel(correction_dir, 0.3)

                    elif sonar < 2:
                        # if we are too close to an obstacle, we move backward
                        # and turn arround
                        linear_velocity = -2
                        angular_velocity = ang_vel(not correction_dir, 0.5)

                    elif dist_til_esq < dist_esq-0.5:
                        # if we are close to finishing the dodge, we can slow down
                        # to avoid overshooting the objective
                        angular_velocity = 0
                        linear_velocity = 0.6 + dist_til_esq/dist_esq

                    else:
                        # the obstacle have been dodge
                        # or it's a new one
                        linear_velocity = 0
                        angular_velocity = 0
                        orientation_esquive = False
                        check_if_goal_clear = True
                        come_from = True
                        come_from_goal = True
                        cf = bot_dir_z  # to avoid going back from there

            elif check_if_goal_clear:
                # after dodging the obstacle, we check if there is a clear path to the goal
                # so we first orient to it
                reoriented_once = False
                linear_velocity = 0

                reoriented, angular_velocity = reorientation(
                    goal_dir, bot_dir_z)
                if reoriented:

                    new_obstacle = True
                    check_if_goal_clear = False
                    if come_from_goal:
                        cf = bot_dir_z

            elif sonar < 3 and sonar < goal_dist:

                if new_obstacle:
                    # if it's a new obstacle, we go back to the procedure of finding a dodging path above
                    init_dir = bot_dir_z
                    linear_velocity = 0
                    angular_velocity = ang_vel(False, 0.4)
                    contournement = True
                    new_obstacle = False

            elif goal_dist < 5:
                # if we are close to the goal, we can go straight to it
                reoriented, angular_velocity = reorientation(
                    goal_dir, bot_dir_z)
                if reoriented:
                    linear_velocity = 1
                else:
                    linear_velocity = 0

            else:

                linear_velocity, angular_velocity = reorientation(
                    goal_dir, bot_dir_z, linear_vel=True)
        if sonar < 1:
            linear_velocity = -0.5
        else:
            if linear_velocity == -0.5:
                linear_velocity = 0
        # Finishing by publishing the desired speed. DO NOT TOUCH.
        print(angular_velocity)
        robot.set_speed_angle(linear_velocity, angular_velocity)

        rate.sleep()


if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Strategy", anonymous=True)

    run_demo()

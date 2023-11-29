#!/usr/bin/env python

import rospy
import math
import cv2
import os
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
from libbehaviors import *
from geometry_msgs.msg import PoseWithCovarianceStamped

flag: bool = False

DIRECTORY = "/home/phil/catkin_ws/src/racecar/"

class PathFollowing:
    def __init__(self):
        os.chdir(DIRECTORY)
        self.doBrushfireMap()
        
        self.max_speed = rospy.get_param("~max_speed", 1)
        self.max_steering = rospy.get_param("~max_steering", 0.37)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber(
            "scan", LaserScan, self.scan_callback, queue_size=1
        )
        self.odom_sub = rospy.Subscriber(
            "localization_pose", PoseWithCovarianceStamped, self.pose_callback, queue_size=1
        )


    def scan_callback(self, msg):
        #######################################################################
        # Wall estimations
        #######################################################################
        ranges = np.array(msg.ranges)

        # Left side
        d_data = []
        theta_data = []
        n_good_scan = 0

        for i in range(70, 110):
            scan_is_good = (ranges[i] > msg.range_min) & (ranges[i] < msg.range_max)

            if scan_is_good:
                d_data.append(ranges[i])
                theta_data.append(msg.angle_min + i * msg.angle_increment)

                n_good_scan = n_good_scan + 1

        if n_good_scan > 2:
            d = np.array(d_data)
            theta = np.array(theta_data)

            y = d * np.sin(theta)
            x = d * np.cos(theta)

            ones = np.ones(x.shape)

            A = np.column_stack((x, ones))

            # Least square
            estimation = np.linalg.lstsq(A, y)[0]  # (ATA)^-1 ATy

            m = estimation[0]  # slope
            b = estimation[1]  # offset

            self.theta_left = np.arctan(m)
            self.y_left = b

        # Right side

        d_data = []
        theta_data = []
        n_good_scan = 0

        for i in range(250, 290):
            scan_is_good = (ranges[i] > msg.range_min) & (ranges[i] < msg.range_max)

            if scan_is_good:
                d_data.append(ranges[i])
                theta_data.append(msg.angle_min + i * msg.angle_increment)

                n_good_scan = n_good_scan + 1

        if n_good_scan > 2:
            d = np.array(d_data)
            theta = np.array(theta_data)

            y = d * np.sin(theta)
            x = d * np.cos(theta)

            ones = np.ones(x.shape)

            A = np.column_stack((x, ones))

            # Least square
            estimation = np.linalg.lstsq(A, y)[0]  # (ATA)^-1 ATy

            m = estimation[0]  # slope
            b = estimation[1]  # offset

            self.theta_right = np.arctan(m)
            self.y_right = b

        self.laser_theta = 0.5 * (self.theta_left + self.theta_right)
        self.laser_y = 0.5 * (self.y_left + self.y_right)

        #######################################################################
        # Our code
        #######################################################################
        # Because the lidar is oriented backward on the racecar,
        # if we want the middle value of the ranges to be forward:
        # l2 = len(msg.ranges)/2;
        # ranges = msg.ranges[l2:len(msg.ranges)] + msg.ranges[0:l2]

        self.laser_y

        vec: list = [[0], [self.laser_theta], [self.laser_y]]
        goal: list = [[0], [0], [0]]

        erreur = [
            goal[0][0] - vec[0][0],
            goal[1][0] - vec[1][0],
            goal[2][0] - vec[2][0],
        ]

        K_path: list = [[8.5676, 0, 0], [0, -0.5383, 0.3162]]
        cmd: list = np.dot(K_path, erreur)

        twist = Twist()
        twist.linear.x = self.max_speed
        twist.angular.z = cmd[1]

        self.cmd_vel_pub.publish(twist)

    def pose_callback(self, pose_in: PoseWithCovarianceStamped):
        brushfireMapUpdate = self.brushfireMap.copy()

        pose = pose_in.pose.pose.position
        level = 5
        square_array = np.zeros((2 * level + 1, 2 * level + 1)).astype(int)

        current_squarex = int(round(self.px_par_m * pose.x + self.offsetx, 0))
        current_squarey = int(round(self.px_par_m * pose.y + self.offsety, 0))
        rospy.logwarn(str(current_squarex) + ", " + str(current_squarey))
        
        for larg in range(-level, level + 1):
            for haut in range(-level, level + 1):
                # rospy.logwarn(str(larg) + ", " + str(haut))
                square_array[larg + level][haut + level] = self.brushfireMapOg[current_squarex + larg][current_squarey + haut]
                brushfireMapUpdate[current_squarex + larg][current_squarey + haut] = int(100)
                np.append(square_array, self.brushfireMap[int(round(self.px_par_m*(pose.x+larg), 0))][int(round(self.px_par_m*(pose.y+haut), 0))])

        max_index = np.unravel_index(np.argmax(square_array, axis=None), square_array.shape)

        print(square_array)
        print("Index of maximum value:", max_index)
        print("")


        maximum = np.amax(brushfireMapUpdate)
        if maximum > 1:
            mask = brushfireMapUpdate == 1
            brushfireMapUpdate = (
                brushfireMapUpdate.astype(float) / float(maximum) * 225.0 + 30.0
            )
            brushfireMapUpdate[mask] = 0
            # Flip image to get x->up, y->left (like top view in RVIZ looking towards x-axis)
            cv2.imwrite(os.path.join(DIRECTORY + "brushfire2.bmp"), brushfireMapUpdate)
            rospy.loginfo("Exported brushfire.bmp")
        else:
            rospy.loginfo("brushfire failed! Is brusfire implemented?")

    def doBrushfireMap(self):
        try:
            rospy.wait_for_service("/racecar/get_map", timeout=5)
            get_map = rospy.ServiceProxy("/racecar/get_map", GetMap)
            response = get_map()
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return

        grid = np.reshape(
            response.map.data, [response.map.info.height, response.map.info.width]
        )
        self.brushfireMap = brushfire(grid)
        self.brushfireMap = self.brushfireMap.transpose().copy()

        self.px_par_m = 211 / 21  # 211px 398px et 21m x 40m

        self.offsety = int(round((3.65) * self.px_par_m, 0))
        self.offsetx = int(round((8.9) * self.px_par_m, 0))

        maximum = np.amax(self.brushfireMap)
        self.brushfireMapOg = self.brushfireMap.copy()

        if maximum > 1:
            mask = self.brushfireMap == 1
            self.brushfireMap = (
                self.brushfireMap.astype(float) / float(maximum) * 225.0 + 30.0
            )
            self.brushfireMap[mask] = 0
            # Flip image to get x->up, y->left (like top view in RVIZ looking towards x-axis)
            cv2.imwrite(os.path.join(DIRECTORY + "brushfire2.bmp"), self.brushfireMap)
            rospy.loginfo("Exported brushfire.bmp")
        else:
            rospy.loginfo("brushfire failed! Is brusfire implemented?")

        # grid[grid == -1] = 89
        # grid[grid == 0] = 178
        # grid[grid == 100] = 0

        # rospy.loginfo(os.listdir(directory))

        # rospy.loginfo(cv2.imwrite("map.bmp", cv2.transpose(cv2.flip(grid, -1))))
        # rospy.loginfo("Exported map.bmp")

        return


def main():
    rospy.init_node("path_following")
    pathFollowing = PathFollowing()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

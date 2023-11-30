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
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
import math as m
from move_base_msgs.msg import MoveBaseActionGoal
from racecar_behaviors.srv import goal, goalRequest, goalResponse
from geometry_msgs.msg import PoseStamped
from threading import Lock

flag: bool = True

DIRECTORY = "/home/phil/catkin_ws/src/racecar/"

# def create_image_from_array(array, title='Image from Array'):
#     try:
#         plt.imshow(array, cmap='viridis', interpolation='nearest')
#         plt.title(title)
#         plt.colorbar()
#         plt.show()
#     except:
#         plt.close()
#         return

class PathFollowing:
    def __init__(self):
        os.chdir(DIRECTORY)
        # self.doBrushfireMap()

        self.lock = Lock()
        
        self.max_speed = rospy.get_param("~max_speed", 1)
        self.max_steering = rospy.get_param("~max_steering", 0.37)
        # self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        # self.scan_sub = rospy.Subscriber("scan", LaserScan, self.scan_callback, queue_size=1)
        # self.odom_sub = rospy.Subscriber("/racecar/odometry/corrected", Odometry, self.pose_callback, queue_size=1)

        self.goalSub = rospy.Subscriber("/racecar/move_base/current_goal", PoseStamped, self.mvCurrentGoalCallback, queue_size=1)
        # self.goalServer = rospy.Service('/pertiglance/goal', goal, self.goalSrvCallback)

        self.currentGoal: PoseStamped = PoseStamped()

    def mvCurrentGoalCallback(self, currentGoal: PoseStamped):
        with self.lock:
            self.currentGoal = currentGoal

    # def goalSrvCallback(self, req: goalRequest):
    #     rep: goalResponse = goalResponse
    #     try:
    #         pub = rospy.Publisher('/racecar/move_base/goal', MoveBaseActionGoal, queue_size=1)
    #         msg: MoveBaseActionGoal = MoveBaseActionGoal()

    #         msg.goal.target_pose.header.frame_id = "racecar/map"
    #         msg.goal.target_pose.header.seq = 1 
    #         msg.goal.target_pose.pose.position.x = req.posX
    #         msg.goal.target_pose.pose.position.y = req.posY
            
    #         angle_z_rad = m.radians(req.theta_deg)
            
    #         #Quaternion angle conversion
    #         cy = math.cos(angle_z_rad * 0.5)
    #         sy = math.sin(angle_z_rad * 0.5)
    #         cz = math.cos(0)
    #         sz = math.sin(0)

    #         msg.goal.target_pose.pose.orientation.w = cy * cz
    #         msg.goal.target_pose.pose.orientation.z = sy * cz

    #         pub.publish(msg)

    #         rospy.sleep(rospy.Duration(0.5))
    #         if self.currentGoal.pose.position.x != msg.goal.target_pose.pose.position.x:
    #             raise Exception("")
            
    #         rospy.loginfo("Succes sending goal")
    #         return goalResponse(True)
    #     except:
    #         rospy.logerr("Error sending goal")
    #         return goalResponse(False) 

    # def scan_callback(self, msg):
    #     #######################################################################
    #     # Wall estimations
    #     #######################################################################
    #     ranges = np.array(msg.ranges)

    #     # Left side
    #     d_data = []
    #     theta_data = []
    #     n_good_scan = 0

    #     for i in range(70, 110):
    #         scan_is_good = (ranges[i] > msg.range_min) & (ranges[i] < msg.range_max)

    #         if scan_is_good:
    #             d_data.append(ranges[i])
    #             theta_data.append(msg.angle_min + i * msg.angle_increment)

    #             n_good_scan = n_good_scan + 1

    #     if n_good_scan > 2:
    #         d = np.array(d_data)
    #         theta = np.array(theta_data)

    #         y = d * np.sin(theta)
    #         x = d * np.cos(theta)

    #         ones = np.ones(x.shape)

    #         A = np.column_stack((x, ones))

    #         # Least square
    #         estimation = np.linalg.lstsq(A, y, rcond=None)[0]  # (ATA)^-1 ATy

    #         m = estimation[0]  # slope
    #         b = estimation[1]  # offset

    #         self.theta_left = np.arctan(m)
    #         self.y_left = b

    #     # Right side

    #     d_data = []
    #     theta_data = []
    #     n_good_scan = 0

    #     for i in range(250, 290):
    #         scan_is_good = (ranges[i] > msg.range_min) & (ranges[i] < msg.range_max)

    #         if scan_is_good:
    #             d_data.append(ranges[i])
    #             theta_data.append(msg.angle_min + i * msg.angle_increment)

    #             n_good_scan = n_good_scan + 1

    #     if n_good_scan > 2:
    #         d = np.array(d_data)
    #         theta = np.array(theta_data)

    #         y = d * np.sin(theta)
    #         x = d * np.cos(theta)

    #         ones = np.ones(x.shape)

    #         A = np.column_stack((x, ones))

    #         # Least square
    #         estimation = np.linalg.lstsq(A, y, rcond=None)[0]  # (ATA)^-1 ATy

    #         m = estimation[0]  # slope
    #         b = estimation[1]  # offset

    #         self.theta_right = np.arctan(m)
    #         self.y_right = b

    #     self.laser_theta = 0.5 * (self.theta_left + self.theta_right)
    #     self.laser_y = 0.5 * (self.y_left + self.y_right)

    #     #######################################################################
    #     # Our code
    #     #######################################################################
    #     # Because the lidar is oriented backward on the racecar,
    #     # if we want the middle value of the ranges to be forward:
    #     # l2 = len(msg.ranges)/2;
    #     # ranges = msg.ranges[l2:len(msg.ranges)] + msg.ranges[0:l2]

    #     self.laser_y

    #     vec: list = [[0], [self.laser_theta], [self.laser_y]]
    #     goal: list = [[0], [0], [0]]

    #     erreur = [
    #         goal[0][0] - vec[0][0],
    #         goal[1][0] - vec[1][0],
    #         goal[2][0] - vec[2][0],
    #     ]

    #     K_path: list = [[8.5676, 0, 0], [0, -0.5383, 0.3162]]
    #     cmd: list = np.dot(K_path, erreur)

    #     twist = Twist()
    #     twist.linear.x = self.max_speed
    #     twist.angular.z = cmd[1]

    #     self.cmd_vel_pub.publish(twist)

    # def pose_callback(self, pose_in: Odometry):
    #     brushfireMapUpdate = self.brushfireMap.copy()

    #     pose = pose_in.pose.pose.position
    #     level = 5
    #     square_array = np.zeros((2 * level + 1, 2 * level + 1)).astype(int)

    #     current_squarex = int(round(self.px_par_m * pose.x + self.offsetx_px, 0))
    #     current_squarey = int(round(self.px_par_m * pose.y + self.offsety_py, 0))
    #     rospy.logwarn(str(current_squarex) + ", " + str(current_squarey))
        
    #     for larg in range(-level, level + 1):
    #         for haut in range(-level, level + 1):
    #             square_array[larg + level][haut + level] = self.brushfireMapOg[current_squarex + larg][current_squarey + haut]
    #             # brushfireMapUpdate[current_squarex + larg][current_squarey + haut] = int(100)

    #     # print(square_array)
    #     # print("")

    #     # HardCoding current goal
    #     goal: Point = Point()
    #     goal.x = 13.5
    #     goal.y = 2.1
        
    #     self.doAStarMap(pose_in.pose.pose.position, goal)

    #     # maximum = np.amax(brushfireMapUpdate)
    #     # if maximum > 1:
    #     #     mask = brushfireMapUpdate == 1
    #     #     brushfireMapUpdate = (
    #     #         brushfireMapUpdate.astype(float) / float(maximum) * 225.0 + 30.0
    #     #     )
    #     #     brushfireMapUpdate[mask] = 0
    #     #     # Flip image to get x->up, y->left (like top view in RVIZ looking towards x-axis)
    #     #     cv2.imwrite(os.path.join(DIRECTORY + "brushfire2.bmp"), brushfireMapUpdate)
    #     #     rospy.loginfo("Exported brushfire.bmp")
    #     # else:
    #     #     rospy.loginfo("brushfire failed! Is brusfire implemented?")

    # def doBrushfireMap(self):
    #     try:
    #         rospy.wait_for_service("/racecar/get_map", timeout=5)
    #         get_map = rospy.ServiceProxy("/racecar/get_map", GetMap)
    #         response = get_map()
    #     except rospy.ServiceException as e:
    #         rospy.logerr("Service call failed: %s" % e)
    #         return

    #     grid = np.reshape(
    #         response.map.data, [response.map.info.height, response.map.info.width]
    #     )

    #     self.gridOg = grid.copy()

    #     self.brushfireMap = brushfire(grid)
    #     self.brushfireMap = self.brushfireMap.transpose().copy()

    #     self.px_par_m = 211 / 21  # 211px 398px et 21m x 40m

    #     self.offsetx_m: float = 3.65
    #     self.offsety_m: float = 8.9
    #     self.offsety_py = int(round(self.offsetx_m * self.px_par_m, 0))
    #     self.offsetx_px = int(round(self.offsety_m* self.px_par_m, 0))

    #     maximum = np.amax(self.brushfireMap)
    #     self.brushfireMapOg = self.brushfireMap.copy()

    #     if maximum > 1:
    #         mask = self.brushfireMap == 1
    #         self.brushfireMap = (self.brushfireMap.astype(float) / float(maximum) * 225.0 + 30.0)
    #         self.brushfireMap[mask] = 0
    #         # Flip image to get x->up, y->left (like top view in RVIZ looking towards x-axis)
    #         cv2.imwrite(os.path.join(DIRECTORY + "brushfire2.bmp"), self.brushfireMap)
    #         rospy.loginfo("Exported brushfire.bmp")
    #     else:
    #         rospy.loginfo("brushfire failed! Is brusfire implemented?")

    #     return

    # def doAStarMap(self, position: Point, goal: Point):
    #     global flag
    #     if flag:
    #         flag = False
    #         #self.offsety = int(round((3.65) * self.px_par_m, 0))
    #         #self.offsetx = int(round((8.9) * self.px_par_m, 0))
            
    #         map = self.gridOg.copy()
    #         map = map.transpose().copy()
    #         mToPx: float = 10

    #         # # Getting racecar position on map (maybe useless)
    #         # pos_index_x = int(round(mToPx * position.x + self.offsetx, 0))
    #         # pos_index_y = int(round(mToPx * position.y + self.offsety, 0))
    #         # rospy.loginfo("racecar at :" + str(pos_index_x) + ", " + str(pos_index_y))

    #         # Getting goal psoition on map
    #         goal_index_x = int(round(mToPx * goal.x + self.offsetx_px, 0))
    #         goal_index_y = int(round(mToPx * goal.y + self.offsety_py, 0))

    #         # rospy.loginfo("goal at :" + str(goal_index_x) + ", " + str(goal_index_y))
    #         # rospy.loginfo(map[goal_index_x][goal_index_y])

    #         # Getting length in m in flight
    #         maximum: int = max([map.shape[0], map.shape[1]])
    #         rospy.loginfo(maximum)

    #         for x in range(0, map.shape[0]):
    #             for y in range(0, map.shape[1]):
    #                 if map[x][y] == 0:
    #                     map[x][y] = maximum - abs(m.sqrt(m.pow((float(x)/mToPx - self.offsety_m - goal.x), 2) + m.pow(((float(y)/mToPx - self.offsetx_m) - goal.y), 2)))*10
    #                     # rospy.loginfo("map[%d][%d]: %f", x, y, map[x][y])
    #                 else:
    #                     map[x][y] = -1

    #         map[goal_index_x][goal_index_y] = -1
    #         # mapBurshfire = self.brushfireMapOg.copy()                
    #         # mapAStar = (self.brushfireMapOg)* map

    #         create_image_from_array(map)

def main():
    rospy.init_node("path_following")
    pathFollowing = PathFollowing()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

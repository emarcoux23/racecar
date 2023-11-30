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
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped
from threading import Lock

def quaternion_to_yaw(w, x, y, z):
    # Calculate yaw (rotation around the vertical axis)
    yaw_rad = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
    yaw_deg = np.degrees(yaw_rad)
    return yaw_deg


class GoalManager:
    def __init__(self):
        self.lock = Lock()
        self.goalList: list = list()
        self.currentGoal: PoseStamped = PoseStamped()
        self.goalReached = False
        self.goalReachedReset = False

        self.goalSub = rospy.Subscriber("/racecar/move_base/current_goal", PoseStamped, self.mvCurrentGoalCallback, queue_size=1)
        self.goalServer = rospy.Service('/pertiglance/goal', goal, self.goalSrvCallback)
        self.goalPub = rospy.Publisher('/racecar/move_base/goal', MoveBaseActionGoal, queue_size=1)
        self.currentGoalStatus = rospy.Subscriber("/racecar/move_base/status", GoalStatusArray, self.goalStatusCallback, queue_size=1)
        self.currentGoal: PoseStamped = PoseStamped()

        self.pub_goal = rospy.Timer(rospy.Duration(0.25), self.goalCallback)

    def goalCallback(self, unused):
        with self.lock:
            if len(self.goalList) != 0:
                req = self.goalList[0]
                
                rospy.logwarn(self.currentGoal.pose.position.x != self.goalList[0].posX or self.currentGoal.pose.position.y != self.goalList[0].posY)
                

                if (self.currentGoal.pose.position.x != req.posX or 
                    self.currentGoal.pose.position.y != req.posY or
                    quaternion_to_yaw(  self.currentGoal.pose.orientation.w,
                                        self.currentGoal.pose.orientation.x,
                                        self.currentGoal.pose.orientation.y,
                                        self.currentGoal.pose.orientation.z)
                                                                            != req.theta_deg):
                    
                    msg: MoveBaseActionGoal = MoveBaseActionGoal()

                    msg.goal.target_pose.header.frame_id = "racecar/map"
                    msg.goal.target_pose.header.seq = 1
                    msg.goal.target_pose.pose.position.x = req.posX
                    msg.goal.target_pose.pose.position.y = req.posY
                    
                    angle_z_rad = m.radians(req.theta_deg)
                    
                    #Quaternion angle conversion
                    cy = math.cos(angle_z_rad * 0.5)
                    sy = math.sin(angle_z_rad * 0.5)
                    cz = math.cos(0)

                    msg.goal.target_pose.pose.orientation.w = cy * cz
                    msg.goal.target_pose.pose.orientation.z = sy * cz

                    self.goalPub.publish(msg)
                    # rospy.sleep(rospy.Duration(0.5))
                    # if self.currentGoal.pose.position.x != msg.goal.target_pose.pose.position.x:
                    #     raise Exception("")

    def goalStatusCallback(self, msg: GoalStatusArray):
        with self.lock:
            if (len(self.goalList) > 0):
                if (len(msg.status_list) > 0 and msg.status_list[0].status == 3 and self.goalReachedReset == True):
                    self.goalReached = True
                else:
                    self.goalReachedReset = True

                if (self.goalReached):
                    self.goalList.pop(0)
                    self.goalReached = False
                    self.goalReachedReset = False

    def goalSrvCallback(self, req: goalRequest):
        with self.lock:
            # Final goal
            if req.type == 0:
                self.goalList.append(req)
            
            # Waypoint
            elif req.type == 1:
                if len(self.goalList) > 0:
                    self.goalList.insert(-1, req)
                else:
                    rospy.logwarn("No final goal set, can't add waypoint")
            
            # BLOB
            elif req.type == 2:
                self.goalList.insert(0, req)

            # Clear
            elif req.type == 3:
                self.goalList = list()

            # Skip current Goal
            elif req.type == 4:
                self.goalList.pop(0)

            return goalResponse(True)

    def mvCurrentGoalCallback(self, msg):
        with self.lock:
            self.currentGoal = msg

def main():
    rospy.init_node("path_following")
    pathFollowing = GoalManager()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

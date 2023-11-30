#!/usr/bin/env python

import rospy
import math
import cv2
import os
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from threading import Lock

class OdomCorrection:
    def __init__(self):
        self.lock = Lock()
        self.pose_sub = rospy.Subscriber("localization_pose", PoseWithCovarianceStamped, self.poseCallback, queue_size=1)
        self.odom_sub = rospy.Subscriber("/racecar/odometry/filtered", Odometry, self.odomCallback, queue_size=1)
        self.odom_corrected_pub = rospy.Publisher("/racecar/odometry/corrected", Odometry, queue_size=1)

        self.timerOdomCorrected = rospy.Timer(rospy.Duration(0.05), self.positionCorrection)

    def poseCallback(self, pose: PoseWithCovarianceStamped):
        with self.lock:
            self.posePosition = pose.pose.pose.position
            self.lastOdomPoseUpdate = self.odomPosition

    def odomCallback(self, odom: Odometry):
        with self.lock:
            self.odomPosition = odom.pose.pose.position

        odom_corrected: Odometry = Odometry()
        odom_corrected.pose.pose.position.x = (self.odomPosition.x - self.lastOdomPoseUpdate.x + self.posePosition.x)
        odom_corrected.pose.pose.position.y = (self.odomPosition.y - self.lastOdomPoseUpdate.y + self.posePosition.y)
        
        self.odom_corrected_pub.publish(odom_corrected)

    def positionCorrection(self, timer: rospy.Timer):
        nothing = 1

def main():
    rospy.init_node("odom_correction")
    odomCorrection = OdomCorrection()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

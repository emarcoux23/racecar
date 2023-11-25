#!/usr/bin/env python

import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class PathFollowing:
    def __init__(self):
        self.max_speed = rospy.get_param("~max_speed", 1)
        self.max_steering = rospy.get_param("~max_steering", 0.37)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber(
            "scan", LaserScan, self.scan_callback, queue_size=1
        )
        self.odom_sub = rospy.Subscriber(
            "odom", Odometry, self.odom_callback, queue_size=1
        )

    def scan_callback(self, msg):
        #######################################################################
        # Wall estimations
        #######################################################################
        ranges = np.array( msg.ranges )

        # Left side
        d_data      = []
        theta_data  = []
        n_good_scan = 0
        
        for i in range(70,110):
            
            scan_is_good = (( ranges[i] > msg.range_min) &
                            ( ranges[i] < msg.range_max) )
                           
            if scan_is_good:
                
                d_data.append( ranges[i] )
                theta_data.append( msg.angle_min + i * msg.angle_increment )
                
                n_good_scan = n_good_scan + 1
                
        if n_good_scan > 2 :
                
            d     = np.array( d_data )
            theta = np.array( theta_data ) 
            
            y     = d * np.sin( theta )
            x     = d * np.cos( theta )
            
            ones  = np.ones( x.shape )
            
            A = np.column_stack( ( x ,  ones ) )
            
            # Least square
            estimation = np.linalg.lstsq(A,y)[0] # (ATA)^-1 ATy
            
            m = estimation[0] # slope
            b = estimation[1] # offset
            
            self.theta_left = np.arctan( m )
            self.y_left     = b
            
        # Right side
        
        d_data      = []
        theta_data  = []
        n_good_scan = 0
        
        for i in range(250,290):
            
            scan_is_good = (( ranges[i] > msg.range_min) &
                            ( ranges[i] < msg.range_max) )
                           
            if scan_is_good:
                
                d_data.append( ranges[i] )
                theta_data.append( msg.angle_min + i * msg.angle_increment )
                
                n_good_scan = n_good_scan + 1
                
        if n_good_scan > 2 :
                
            d     = np.array( d_data )
            theta = np.array( theta_data ) 
            
            y     = d * np.sin( theta )
            x     = d * np.cos( theta )
            
            ones  = np.ones( x.shape )
            
            A = np.column_stack( ( x ,  ones ) )
            
            # Least square
            estimation = np.linalg.lstsq(A,y)[0] # (ATA)^-1 ATy
            
            m = estimation[0] # slope
            b = estimation[1] # offset
            
            self.theta_right = np.arctan( m )
            self.y_right    = b
            
        self.laser_theta = 0.5 * ( self.theta_left + self.theta_right )
        self.laser_y     = 0.5 * ( self.y_left + self.y_right )
        
        #######################################################################
        # Our code
        #######################################################################
        # Because the lidar is oriented backward on the racecar,
        # if we want the middle value of the ranges to be forward:
        # l2 = len(msg.ranges)/2;
        # ranges = msg.ranges[l2:len(msg.ranges)] + msg.ranges[0:l2]
        
        vec: list = [[0], [self.laser_theta], [self.laser_y]]
        goal: list = [[0], [0], [0]]

        erreur = [goal[0][0] - vec[0][0],
                  goal[1][0] - vec[1][0],
                  goal[2][0] - vec[2][0]]

        K_path: list = [[8.5676, 0, 0], [0, -0.5383, 0.3162]]
        cmd:list = np.dot(K_path, erreur)

        twist = Twist()
        twist.linear.x = self.max_speed
        twist.angular.z = cmd[1]

        self.cmd_vel_pub.publish(twist)

    def odom_callback(self, msg):
        rospy.loginfo("Current speed = %f m/s", msg.twist.twist.linear.x)


def main():
    rospy.init_node("path_following")
    pathFollowing = PathFollowing()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

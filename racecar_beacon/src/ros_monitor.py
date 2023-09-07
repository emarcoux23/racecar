#!/usr/bin/env python

import rospy
import socket
import threading

from tf.transformations import euler_from_quaternion

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class ROSMonitor:
    def __init__(self):

        rospy.loginfo("ros_monitor.py: Starting")

        # Add your subscriber here (odom? laserscan?):
        self.sub_odom = rospy.Subscriber("/odometry/filtered", Odometry, self.cb_odometry)
        self.sub_laser = rospy.Subscriber("/scan", LaserScan, self.cb_laserscan)

        # Current robot state:
        self.pos = (0,0,0)
        self.obstacle = False
        self.id = 0xFAFABEBE

        # Params :
        self.remote_request_port = rospy.get_param("remote_request_port", 65432)
        self.pos_broadcast_port  = rospy.get_param("pos_broadcast_port", 65431)

        # Thread for RemoteRequest handling:
        print("Je suis ici")
        self.pb_thread = threading.Thread(target=self.pb_loop)
        self.pb_thread.start()

    def cb_odometry(self, msg:Odometry):
        (roll, pitch, yaw) = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.pos = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        #rospy.loginfo("ros_monitor.py: pos(x, y, yaw): " + str(self.pos))

    def cb_laserscan(self, msg:LaserScan): 
        if min(msg.ranges) <= 1:
            self.obstacle = True
            #rospy.logwarn("ros_monitor.py: Obstacle <=1m ?: " + str(self.obstacle))            
        else:
            self.obstacle = False
            #rospy.loginfo("ros_monitor.py: Obstacle <=1m ?: " + str(self.obstacle))                  

    def pb_loop(self):
        print("pb_loop enter")
        self.pb_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)    
        self.pb_socket.connect(("127.0.0.1", self.pos_broadcast_port))
        self.pb_socket.send(bytes(self.pos))                                    #TODO ajouter le reste
        while True:
            pass

if __name__=="__main__":
    rospy.init_node("ros_monitor")
    node = ROSMonitor()
    rospy.spin()

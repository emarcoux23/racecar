#!/usr/bin/env python

import rospy
import socket
import threading
import struct
import sys

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
        self.pos_x = 0
        self.pos_y = 0
        self.pos_yaw = 0
        self.obstacle = False
        self.id = 0xFAFABEBE

        # Params :
        self.remote_request_port = rospy.get_param("remote_request_port", 65432)
        self.pos_broadcast_port  = rospy.get_param("pos_broadcast_port", 65431)

        # Thread for RemoteRequest handling:
        print("Je suis ici")
        self.rr_thread = threading.Thread(target=self.rr_loop)
        self.rr_thread.start()

        self.pb_init()

    def cb_odometry(self, msg:Odometry):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        (roll, pitch, self.pos_yaw) = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

    def cb_laserscan(self, msg:LaserScan): 
        if min(msg.ranges) <= 1.0:
            self.obstacle = True          
        else:
            self.obstacle = False                  

    def pb_init(self):
        print("pb_loop enter")
        self.server_address = ("192.168.10.255", self.pos_broadcast_port)

        self.pb_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.pb_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        
        self.timer_pb: rospy.Timer = rospy.Timer(rospy.Duration(1.0), self.pb_send)

    def pb_send(self, some_arg):

        try:
            self.pos_x = 1.23
            self.pos_y = 2.34
            self.pos_yaw = 3.45
            self.pos_x_bin = struct.pack('!f', self.pos_x)
            self.pos_y_bin = struct.pack('!f', self.pos_y)
            self.pos_yaw_bin = struct.pack('!f', self.pos_yaw)
            self.id_bin = struct.pack('!I', self.id)

            self.pos_send = self.pos_x_bin + self.pos_y_bin + self.pos_yaw_bin + self.id_bin
            self.pb_socket.sendto(self.pos_send, self.server_address)

            rospy.loginfo("Sending to " + self.server_address[0])

        except Exception as e:
            rospy.logerr(e)

    def rr_loop(self, some_arg):
        self.rr_socket.send("RPOS") #TODO modifier

if __name__=="__main__":
    rospy.init_node("ros_monitor")
    node = ROSMonitor()
    rospy.spin()
    node.pb_socket.close()

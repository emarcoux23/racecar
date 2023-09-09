#!/usr/bin/env python

import rospy
import socket
import threading
import struct
import sys
import time

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
        self.rr_thread = threading.Thread(target=self.rr_loop)
        self.rr_thread.setDaemon(True)
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
        # print("pb_loop enter")
        self.server_address = ("127.0.0.255", self.pos_broadcast_port)

        self.pb_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.pb_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        
        self.timer_pb: rospy.Timer = rospy.Timer(rospy.Duration(1.0), self.pb_send)

    def pb_send(self, some_arg):

        try:
            pos_x_bin = struct.pack('!f', self.pos_x)
            pos_y_bin = struct.pack('!f', self.pos_y)
            pos_yaw_bin = struct.pack('!f', self.pos_yaw)
            id_bin = struct.pack('!I', self.id)

            self.pos_send = pos_x_bin + pos_y_bin + pos_yaw_bin + id_bin
            self.pb_socket.sendto(self.pos_send, self.server_address)

            #rospy.loginfo("Sending to " + self.server_address[0])

        except Exception as e:
            rospy.logerr(e)

    
    def rr_sendback(self, cmd_msg, rr_socket:socket.socket):
        if cmd_msg == "RPOS":
            pos_x_bin = struct.pack('!f', self.pos_x)
            pos_y_bin = struct.pack('!f', self.pos_y)
            pos_yaw_bin = struct.pack('!fxxxx', self.pos_yaw)
            pos_send = pos_x_bin + pos_y_bin + pos_yaw_bin
            rr_socket.send(pos_send)
            print("Je renvoie: " + str(pos_send))
        elif cmd_msg == "OBSF":
            rr_socket.send(struct.pack('!Ixxxxxxxxxxxx', self.obstacle))
            print("Je renvoie: " + str(bool(self.obstacle)))
        elif cmd_msg == "RBID":
            rr_socket.send(struct.pack('!Ixxxxxxxxxxxx', self.id))
            print("Je renvoie: " + str(hex(self.id)))
        else:
            rospy.logfatal("How did we get here?")

        print("rr_feedback exit")
    
    def rr_loop(self):
        self.rr_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.rr_socket.bind(('127.0.0.1', self.remote_request_port))
        self.rr_socket.listen()

        while True:
            try:
                temp_socket, adress = self.rr_socket.accept()              

                with temp_socket:
                    data = temp_socket.recv(4)
                    cmd_msg = data.decode()
                    print("J'ai reçu : " + cmd_msg)
                    self.rr_sendback(cmd_msg, temp_socket)
                    temp_socket.close()
            except:
                None


if __name__=="__main__":
    rospy.init_node("ros_monitor")
    node = ROSMonitor()
    rospy.spin()
    node.pb_socket.close()
    node.rr_socket.close()
    print("Sockets closed")

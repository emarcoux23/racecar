#!/usr/bin/env python

import socket
import struct
import rospy

HOST = '127.0.0.1'
# This process should listen to a different port than the PositionBroadcast client.
PORT = 65432

print("Initialize remote_client.py")

def get_cmd():
    print("Available commands : RPOS or OBSF or RBID")
    user_cmd = input("Enter you selection : ").upper()

    if user_cmd in ("RPOS", "OBSF", "RBID"):
        rc_socket:socket.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        rc_socket.connect((HOST, PORT))
        user_cmd_encoded = user_cmd.encode()
        rc_socket.send(user_cmd_encoded)
        data = rc_socket.recv(16)
        print("Response from user command " + str(user_cmd) + " is: ") 

        if (user_cmd == "RPOS"):
            data_x = struct.unpack("!f", data[0:4])[0]
            data_y = struct.unpack("!f", data[4:8])[0]
            data_theta = struct.unpack("!f", data[8:12])[0]

            print("x:"        + str(data_x)
                + " - y:"     + str(data_y)
                + " - theta:" + str(data_theta))

        elif (user_cmd == "OBSF"):
            data_obsf = struct.unpack("!I", data[0:4])[0]
            print("Obstacle?: " + str(bool(data_obsf)))

        elif (user_cmd == "RBID"):
            data_id = struct.unpack("!I", data[0:4])[0]
            print("Robot ID: " + str(hex(data_id)).upper().replace("X", "x"))
        else:
            rospy.logfatal("How did we get here?")

        print("")
        rc_socket.close()
        
    else : 
        print("Your command doesn't exist, try again")

while True:
    try : 
        get_cmd()
    except KeyboardInterrupt:
        exit()
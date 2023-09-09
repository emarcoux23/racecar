#!/usr/bin/env python

import socket
import struct

HOST = ""
PORT = 65431

server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
server_socket.setsockopt(socket.SOL_SOCKET,   socket.SO_REUSEPORT, 1)
server_socket.setsockopt(socket.SOL_SOCKET,   socket.SO_BROADCAST, 1)
server_socket.bind((HOST, PORT))

try:
    while True:
        data, addr = server_socket.recvfrom(16)
        data_x   = struct.unpack("!f", data[0:4])[0]
        data_y   = struct.unpack("!f", data[4:8])[0]
        data_yaw = struct.unpack("!f", data[8:12])[0]
        data_id  = struct.unpack("!I", data[12:16])[0]

        print("received message: x:" + str(data_x)
              + " - y:"              + str(data_y)
              + " - yaw:"            + str(data_yaw)
              + " - id:"             + str(data_id))

except KeyboardInterrupt:
    server_socket.close()
    exit()

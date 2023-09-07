#!/usr/bin/env python

import socket

HOST = '127.0.0.1'
# This process should listen to a different port than the RemoteRequest client.
PORT = 65431

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
server_socket.bind((HOST, PORT))
server_socket.listen(1)

try:
    while True:
        print("Hoooooooooo")
        conn, addr = server_socket.accept()
        with conn:
            while True:
                print("Hiiiiiiiiiiii")
                data = conn.recv(16)
                if not data : break
                print("Haaaaaaaaaaaaaaaaaa")
                conn.sendall(data)

except KeyboardInterrupt:
    exit()
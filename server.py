from utils import network_scan, ping, start_mission, image_stitch
from imutils import paths
from dronekit import LocationLocal
import numpy as np
import threading
import socket
import pickle
import select
import queue
import os

#network information
HEADERSIZE = 20
SCAN_TIMEOUT = 5
SERVER_PORT = 1234
SERVER_IP = socket.gethostname()

class data_return:
        def __init__(self, image, location, volts, finish_mission):
                self.image = image
                self.location = location
                self.volts = volts

class ClientThread(threading.Thread):
        def __init__(self,client_address, client_socket, wp_queue):
                threading.Thread.__init__(self)
                self.client_socket = client_socket
                self.wp_queue = wp_queue
                print(f"connection from {client_address} on new thread...")
        def run(self):          
                for c, wp in enumerate(self.wp_queue):
                        self.wp = wp
                        #send 
                        msg =  pickle.dumps(self.wp)
                        command = bytes(f'{len(msg):<{HEADERSIZE}}', "utf-8") + msg
                        self.client_socket.send(command)
                        print (f"Waypoint x={wp.east} y={wp.north} sent to client")
                        #receive buffered data
                        full_msg = b''
                        new_msg = True
                        while True:
                                msg = self.client_socket.recv(16)
                                if new_msg:
                                        print("Receiving Data")
                                        msglen = int(msg[:HEADERSIZE])
                                        new_msg = False
                
                
                                full_msg += msg
                
                                if len(full_msg) - HEADERSIZE == msglen:
                                        recvd_data = pickle.loads(full_msg[HEADERSIZE:])
                                        new_msg = True
                                        full_msg = b'' 
                                        break                             
                        #save received data to disk
                        with open(f".//recvd_data/{client_address}_recvd_data_{c}.pickle", "wb") as file:
                                pickle.dump(recvd_data, file, pickle.HIGHEST_PROTOCOL)
                                print(f"datafrane {c} saved to disk")
                
                msg =  pickle.dumps('end_mission')
                end_mission = bytes(f'{len(msg):<{HEADERSIZE}}', "utf-8") + msg
                self.client_socket.send(end_mission)
                print(f"End mission command sent to {client_address}")                               
                
                

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind((SERVER_IP, SERVER_PORT))
print("Server started")
                                        
client_queue = queue.Queue()
wp_queue = queue.Queue()

print(f"Scanning Network for {SCAN_TIMEOUT} seconds")
scan_result = network_scan("192.168.68.1-255",SCAN_TIMEOUT)
print(f"{len(scan_result)} clients identified on network")

for i in range(len(scan_result)):
        client_queue.put(scan_result[i]["ip"])
        
wp_queue = [LocationLocal(-69, -10, 20), LocationLocal(-42, 15, 20), LocationLocal(-7, 56, 20)]

print("Starting image stitching algorithm as background thread...")
stitch_thread = threading.Thread(target = image_stitch)
stitch_thread.start()



print("Waiting for client request...")
while True:
        server.listen(1)
        client_socket, client_address = server.accept()
        newthread = ClientThread(client_address, client_socket, wp_queue)
        newthread.start()  

   
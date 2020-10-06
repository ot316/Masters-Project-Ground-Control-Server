from utils import network_scan, ping, start_mission, image_stitch, euclidean_distance 
from imutils import paths
from dronekit import LocationLocal
from geopy.distance import geodesic
import numpy as np
import threading
import socket
import pickle
import queue
import os
import json
import math


#network constants
HEADERSIZE = 20
SCAN_TIMEOUT = 1
SERVER_PORT = 1234
SERVER_IP = socket.gethostname()

#mapping constants
FOV = 2*math.pi/3 # in radians
EARTHRADIUS = 6371000 #in meters
                
# class definition for returned data from the drone, an object containing attributes for a numpy array of iamge data, battery voltage and relative Location               
class data_return:
        def __init__(self, image, location, volts, finish_mission):
                self.image = image
                self.location = location
                self.volts = volts

#thread class for the client drone
class ClientThread(threading.Thread):
        def __init__(self,client_address, client_socket, wp_queue):
                threading.Thread.__init__(self)
                self.client_socket = client_socket
                self.wp_queue = wp_queue
                print(f"connection from {client_address} on new thread...")
        def run(self):          
                for c, wp in enumerate(self.wp_queue):
                        self.wp = wp
                        #send data from wp, serialise data using pickle
                        msg =  pickle.dumps(self.wp)
                        # float length header to the start of the message. Headersize is a constant size of 20
                        command = bytes(f'{len(msg):<{HEADERSIZE}}', "utf-8") + msg
                        self.client_socket.send(command)
                        print (f"Waypoint x={wp.east} y={wp.north} sent to client")
                        full_msg = b''
                        new_msg = True
                        while True:
                                #continuously receive chunks of a message
                                msg = self.client_socket.recv(16)
                                if new_msg:
                                        print("Receiving Data")
                                        msglen = int(msg[:HEADERSIZE])
                                        new_msg = False
                
                                # append chunks to full message
                                full_msg += msg
                
                                # check if message is the same length as the message length received in header
                                if len(full_msg) - HEADERSIZE == msglen:
                                        recvd_data = pickle.loads(full_msg[HEADERSIZE:])
                                        new_msg = True
                                        full_msg = b'' 
                                        break                             
                        #save received data to disk as a serialised binary file
                        with open(f".//recvd_data/{client_address}_recvd_data_{c}.pickle", "wb") as file:
                                pickle.dump(recvd_data, file, pickle.HIGHEST_PROTOCOL)
                                print(f"datafrane {c} saved to disk")
                
                # wp list is empty, send end mission command.
                msg =  pickle.dumps('end_mission')
                end_mission = bytes(f'{len(msg):<{HEADERSIZE}}', "utf-8") + msg
                self.client_socket.send(end_mission)
                print(f"End mission command sent to {client_address}")
                

# load the coordinate data from the json file saved from geojson.io               
with open('geo_coordinates.json') as file:
        print("Reading coordinates from JSON...")
        coordinates = json.load(file)

# delimit the 2 geometries; point and ploygon
geometry_1 = coordinates['features'][0]['geometry'] 
geometry_2 = coordinates['features'][1]['geometry'] 

#establish which geometry is which
if geometry_1['type'] == 'Point':
        GCS_position = geometry_1['coordinates']
        geo_fence = geometry_2['coordinates']
else:
        GCS_position = geometry_2['coordinates']
        geo_fence = geometry_1['coordinates']        

#print the location of the ground control server and the coordinates of the corners of the geofence
print('GCS position =', GCS_position)
print('Geo Fence =', geo_fence[0][:-1], '\n')

#find the x and y distance of the geo fence
y = geodesic(geo_fence[0][0][0], geo_fence[0][1][0]).m
x = geodesic(geo_fence[0][1][1], geo_fence[0][2][1]).m

print('Geo fence x dimension:', x, 'm')
print('Geo fence y dimension:', y, 'm\n')

# compute the x and y distance between the location of the GCS (starting point) and the nearest coordinate
origin_x = math.inf
origin_y = math.inf
for i in range(3):
        x_check = (GCS_position[0] - geo_fence[0][i][0]) * - EARTHRADIUS * math.cos((GCS_position[1] + geo_fence[0][i][1]) * math.pi / 360) / 360
        y_check = (GCS_position[1] - geo_fence[0][i][1]) * - EARTHRADIUS / 360
        if math.sqrt(x_check**2 * y_check**2) < math.sqrt(origin_x**2 * origin_y**2):
                origin_x = x_check
                origin_y = y_check                

if origin_x == math.inf or origin_y == math.inf:
        print("invalid geometry given...")
        sys.exit()
                
print('x displacement from origin:', origin_x, 'm')
print('y displacement from origin:', origin_y, 'm\n')
print('Please enter map width in m...')

#take user input for the width in m of a single drone photograph.
map_scale = input()
map_scale = map_scale
#map_scale = 60

# calculate the altitude from the cameras field of view
try:
        map_scale = int(map_scale)
        target_alt = map_scale / (2 * math.sin(FOV / 2)) 
        if target_alt > 100:
                print("This scale would cause the drones to exceed 400m altitude, please enter a smaller scale")
                sys.exit()
 
except:
        print("please enter integer")
        
# create grid of all waypoints that would resolve a full image when stitched together.
y_step = y / map_scale
x_step = x / map_scale
grid = np.zeros((int(y /map_scale), int(x / map_scale)), dtype=(int,2))

for i in range(grid.shape[1]):
        for j in range(grid.shape[0]):
                grid[j,i] = (x/grid.shape[1] * j, y/grid.shape[0] * i)  
                
#convert grid of tuples to 1 dimensional array of tuples
waypoints = grid.reshape(-1,2)

#scan network and return number of clients. This is how the program knows how many drones are available on the network.
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind((SERVER_IP, SERVER_PORT))
print("Server started")
                                        
client_queue = queue.Queue()
wp_queue = queue.Queue()

print(f"Scanning Network for {SCAN_TIMEOUT} seconds")
scan_result = network_scan("192.168.68.1-255",SCAN_TIMEOUT)
print(f"{len(scan_result) - 1} clients identified on network\n")

# ignore unnecessary drones
if len(scan_result) > waypoints.shape[0]:
        scan_result = scan_result[:waypoints.shape[0]]

# create list of scanned IPs
for i in range(len(scan_result)):
        client_queue.put(scan_result[i]["ip"])
        print(scan_result[i]["ip"])
print("\n")

print("Starting image stitching algorithm as background thread...")
stitch_thread = threading.Thread(target = image_stitch)

#if no devices are found exit
if len(scan_result) == 0:
        print("no devices found on network, exiting")
        #sys.exit()
        scan_result = ['192.168.68.1','192.168.68.2']
        
path_length = int(waypoints.shape[0] / len(scan_result))
waypoint_queue = []

#segment the coordinates into sections depending on how many drones are connected
for i in range(len(scan_result)-1):
        waypoint_queue.append(waypoints[(i * path_length):((i + 1) * path_length), :])
        
print("Waiting for client request...")
i = 0
# continuously look for incoming connections
while True:
        wp_queue = []
        server.listen(1)
        client_socket, client_address = server.accept()
        # add waypoints to Location Local object and add target altitude.
        for j in waypoint_queue[i]:
                wp_queue.append(LocationLocal(j[0] + origin_x,j[1] + origin_y, -1 * target_alt))
        newthread = ClientThread(client_address, client_socket, wp_queue)
        newthread.start()
        i += 1

   
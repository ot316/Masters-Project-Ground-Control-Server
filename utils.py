import scapy.all as scapy
import argparse
import pickle
import ping3
import socket
import os
import cv2
import numpy as np
import math


def get_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("-t", dest="target")
    options = parser.parse_args()
    return options


def network_scan(ip, timeout):
    arp_request = scapy.ARP(pdst=ip)
    broadcast = scapy.Ether(dst="ff:ff:ff:ff:ff:ff")
    arp_request_broadcast = broadcast/arp_request
    answered_list = scapy.srp(arp_request_broadcast,
                              timeout=timeout, verbose=False)[0]
    clients_list = []
    for element in answered_list:
        client_dict = {"ip": element[1].psrc, "mac": element[1].hwsrc}
        clients_list.append(client_dict)
    return clients_list


def ping(ip):
    if ping3.ping(ip):
        return True
    else:
        return False
    
    
def start_mission(HEADERSIZE, server_ip, server_port, wp_list):
    def data_transfer(HEADERSIZE, server_ip, server_port, wp):
        msg = pickle.dumps(wp)
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
            server_socket.bind((server_ip, server_port))
            server_socket.listen()
            client_socket, client_address = server_socket.accept()        
            with client_socket:
                    # send
                    print(f"connection from {client_address} has been established")
                    command = bytes(f'{len(msg):<{HEADERSIZE}}', "utf-8") + msg
                    client_socket.send(command)
        
                    #receive buffered data
                    full_msg = b''
                    new_msg = True
                    while True:
                        msg = client_socket.recv(16)
                        if new_msg:
                            print("Data received")
                            msglen = int(msg[:HEADERSIZE])
                            new_msg = False
        
        
                        full_msg += msg
        
                        if len(full_msg) - HEADERSIZE == msglen:
                            recvd_data = pickle.loads(full_msg[HEADERSIZE:])
                            new_msg = True
                            full_msg = b'' 
                            break
        return recvd_data
        
               
     
    def end_mission(HEADERSIZE, server_ip, server_port):
        msg = pickle.dumps('finish_mission')
        print("Establishing connection...")
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
            server_socket.bind((server_ip, server_port))
            server_socket.listen()
            client_socket, client_address = server_socket.accept()
            with client_socket:
                # send
                print(f"connection from {client_address} has been established")
                command = bytes(f'{len(msg):<{HEADERSIZE}}', "utf-8") + msg
                client_socket.send(command)
                print(f"{client_address} Mission copmlete")
                
                
                
    for c, wp in enumerate(wp_list):
        recvd_data = data_transfer(HEADERSIZE, client_socket, client_address, wp)
        #save received data to disk
        with open(f".//recvd_data/{client_address}_recvd_data_{c}.pickle", "wb") as file:
            pickle.dump(recvd_data, file, pickle.HIGHEST_PROTOCOL)
            print("data saved to disk")         
    end_mission(HEADERSIZE, client_socket, client_address,)

def euclidean_distance(coord1, coord2):
    R = 6372800  # Earth radius in meters
    lat1, lon1 = coord1
    lat2, lon2 = coord2
    
    phi1, phi2 = math.radians(lat1), math.radians(lat2) 
    dphi       = math.radians(lat2 - lat1)
    dlambda    = math.radians(lon2 - lon1)
    
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    
    return 2*R*math.atan2(math.sqrt(a), math.sqrt(1 - a))    

def trim(frame):
    #crop top
    if not np.sum(frame[0]):
        return trim(frame[1:])
    #crop bottom
    elif not np.sum(frame[-1]):
        return trim(frame[:-2])
    #crop left
    elif not np.sum(frame[:,0]):
        return trim(frame[:,1:]) 
    #crop right
    elif not np.sum(frame[:,-1]):
        return trim(frame[:,:-2])    
    return frame
    
    
def image_stitch():                      
    # Image stitching Algorithm
    #loop over image paths and load each image
    sift = cv2.xfeatures2d.SIFT_create()
    counter = 0
    while True:
        for i in range(1,len(os.listdir(".//recvd_data"))):
            try:
                img_ = img3
            except:        
                byte_data = os.listdir(".//recvd_data")[-1]
                with open(".//recvd_data/" + byte_data, 'rb') as file:
                    data = pickle.load(file)
                img_ = data.image

            img1 = cv2.cvtColor(img_,cv2.COLOR_BGR2GRAY)
            kp, des = sift.detectAndCompute(img1, None)
            byte_data = os.listdir(".//recvd_data")[-i-1]
            with open(".//recvd_data/" + byte_data, 'rb') as file:
                data = pickle.load(file)
            img = data.image
            img2 = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            kp1, des1 = sift.detectAndCompute(img2, None)
            match = cv2.BFMatcher()               
            matches = match.knnMatch(des,des1,k=2)
            good = []
            for m,n in matches:
                if m.distance < 0.9*n.distance:
                    good.append(m)

            MIN_MATCH_COUNT = 10
            if len(good) > MIN_MATCH_COUNT:
                src_pts = np.float32([ kp[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
                dst_pts = np.float32([ kp1[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
                M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
                h,w = img1.shape
                pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
                dst = cv2.perspectiveTransform(pts, M)
                img2 = cv2.polylines(img2,[np.int32(dst)],True,255,3, cv2.LINE_AA)

            else:
                print("Not enought matches are found - %d/%d", (len(good)/MIN_MATCH_COUNT))     

            img3 = cv2.warpPerspective(img_, M, (img.shape[1] + img_.shape[1], img.shape[0]))
            img3[0:img.shape[0],0:img.shape[1]] = img
            img3 = trim(img3)
            counter += 1
            if counter != len(os.listdir(".//recvd_data")):
                cv2.imwrite(".//output_data/stitched_image.png", img3) 
                print("Stitched Image saved as 'stitched_image.png', awaiting more images..") 
            else:
                while counter == len(os.listdir(".//recvd_data")):
                    time.sleep(1)
                image_stitch()
            
                    
     
                
                
if __name__ == '__main__':
    options = get_arguments()
    scan_result = network_scan(options.target, 10)
    print(scan_result)
    for client in scan_result:
        print(ping(client["ip"]))

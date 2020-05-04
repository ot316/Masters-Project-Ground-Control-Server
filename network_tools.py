import scapy.all as scapy
import argparse
import ping3
import socket


def get_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("-t", dest="target")
    options = parser.parse_args()
    return options


def scan(ip, timeout):
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




if __name__ == '__main__':
    options = get_arguments()
    scan_result = scan(options.target, 10)
    print(scan_result)
    for client in scan_result:
        print(ping(client["ip"]))

from network_tools import scan, ping
import socket

HEADERSIZE = 20
port = 1234
ip = socket.gethostname()

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((ip, port))
s.listen(20)  # queue for extra messages


while True:
    clientsocket, address = s.accept()
    msg = "welcome to the server"
    msg = f'{len(msg):<{HEADERSIZE}}' + msg
    print(f"connection from {address} has been established")
    clientsocket.send(bytes(msg, "utf-8"))

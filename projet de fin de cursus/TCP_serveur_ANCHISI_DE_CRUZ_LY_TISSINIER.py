# TCP Server
# Server program for robots to communicate
# Authors: Ambre ANCHISI, Ocean DE CRUZ, Alexandre LY, Axel TISSINIER
# IPSA SAA AU516 Final project
# 5 Jan 2023

import socket
import threading

# Creation of socket server
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Link of socket to an adress and a port
server_socket.bind(('192.168.1.159', 5000))

# Listening of socket waiting for a connections
server_socket.listen()

# dictionary to store client IP addresses
clients = {}

def handle_client(client_socket, client_address):
    # Sending a welcome message to the customer
    message = "Connection accepted"
    client_socket.send(bytes(message, 'utf-8'))
    print(f"Server send 'Connection accepted' to {client_address}")

    while True:
        # Receiving a message from the customer
        data = client_socket.recv(1024)
        if not data:
            break
        print(f"Received message from client {client_address}: {data.decode()}")
        
        # Processing the message received and sending a response to the customer
        if data.decode() == "Lost ball":
            message = "Search"
            # Send "Search" message to other clientsSend "Search" message to other clients
            for addr, sock in clients.items():
                if addr != client_address:
                    sock.send(bytes("Search", 'utf-8'))
                    print(f"Server send 'Search' to {addr}")
        elif data.decode() == "Search" and len(clients) > 1:
            message = "Search"
        elif data.decode() == "Found ball" and len(clients) > 1:
            message = "Leader"
            # Sending the "Follower" message to other customers
            for addr, sock in clients.items():
                if addr != client_address:
                    sock.send(bytes("Follower", 'utf-8'))
                    print(f"Server send 'Follower' to {addr}")
        else:
            message = "Invalid mode"
        client_socket.send(bytes(message, 'utf-8'))
        print(f"Server send '{message}' to {client_address}")
        
    # Closing the server socket
    client_socket.close()

while len(clients) < 2:
    # Accepting a client's connection
    client_socket, client_address = server_socket.accept()
    print(f"Request connection from {client_address}")
    print(f"Connection accepted for {client_address}")
    # Recording of the client's IP address
    clients[client_address] = client_socket
    # Creation of a thread to manage communications with the client
    t = threading.Thread(target=handle_client, args=(client_socket, client_address))
    t.start()
    
# If there are at least two clients, the "Search" message is sent to all clients
for addr, sock in clients.items():
    sock.send(bytes("Search", 'utf-8'))
    print(f"Server send 'Search' to {addr}")

x = ''
while x!='Shutdown':
    x = input()
    b = bytes(x, 'utf-8')
    for addr, sock in clients.items():
        sock.send(b)
        print(f"Server send {x} to {addr}")
# Closing the server socket
server_socket.close()
# -*- coding: utf-8 -*-
"""
Created on Sat Jan  7 07:28:31 2023

@author: lyale
"""

import socket
import threading

# Création de la socket serveur
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Liaison de la socket à une adresse et un port
server_socket.bind(('192.168.18.10', 5000))

# Ecoute de la socket en attente de connexions
server_socket.listen()

# Dictionnaire pour stocker les adresses IP des clients
clients = {}

def handle_client(client_socket, client_address):
    # Envoi d'un message de bienvenue au client
    message = "Connection accepted"
    client_socket.send(bytes(message, 'utf-8'))
    print(f"Server send 'Connection accepted' to {client_address}")

    while True:
        # Réception d'un message du client
        data = client_socket.recv(1024)
        if not data:
            break
        print(f"Received message from client {client_address}: {data.decode()}")
        
        # Traitement du message reçu et envoi d'une réponse au client
        if data.decode() == "Lost ball":
            message = "Search"
            # Envoi du message "Search" aux autres clients
            for addr, sock in clients.items():
                if addr != client_address:
                    sock.send(bytes("Search", 'utf-8'))
        elif data.decode() == "Search" and len(clients) > 1:
            message = "Search"
        elif data.decode() == "Found ball" and len(clients) > 1:
            message = "Leader"
            # Envoi du message "Follower" aux autres clients
            for addr, sock in clients.items():
                if addr != client_address:
                    sock.send(bytes("Follower", 'utf-8'))
                    print(f"Server send 'Follower' to {addr}")
        else:
            message = "Invalid mode"
        client_socket.send(bytes(message, 'utf-8'))
        print(f"Server send '{message}' to {client_address}")
        
    # Fermeture de la socket client
    client_socket.close()

while len(clients) < 2:
    # Acceptation de la connexion d'un client
    client_socket, client_address = server_socket.accept()
    print(f"Request connection from {client_address}")
    print(f"Connection accepted for {client_address}")
    # Enregistrement de l'adresse IP du client
    clients[client_address] = client_socket
    # Création d'un thread pour gérer les communications avec le client
    t = threading.Thread(target=handle_client, args=(client_socket, client_address))
    t.start()
    
# Si il y a au moins deux clients, envoi du message "Search" à tous les clients
for addr, sock in clients.items():
    sock.send(bytes("Search", 'utf-8'))
    print(f"Server send 'Search' to {addr}")

x = ''
while x!='Shutdown':
    x = input()
    b = bytes(x, 'utf-8')
    for addr, sock in clients.items():
        if addr != client_address and x!='stop':
            sock.send(b)
            print(f"Server send {x} to {client_address}")
# Fermeture de la socket serveur
server_socket.close()
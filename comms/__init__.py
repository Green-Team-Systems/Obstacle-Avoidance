# ===============================================================
# Created By: Tyler Fedrizzi
# Authors: Tyler Fedrizzi, Rohan Potdar
# Created On: December 18th, 2020
# Last Modified: March 21st, 2021
# 
# Description: Class to simulate the communications infrastructure
#              of a drone.
# ===============================================================

from queue import Queue, Full, Empty 
import time
from threading import Thread, ThreadError, Lock
import socket
import selectors
HOST = '127.0.0.1'
num_drones = 5
# pylint: disable=unsubscriptable-object
# pylint was being annoying on VSCode so added the above line
# Drone is responsible for reading/writing messages as Packet(s)
# Main caller is responsible for creating KillerClients

class Packet():
    '''
    Packet definition - currently based on MAVLINK packet, but will move to TCP/UDP
    MAVLINK Packet Reference: https://erlerobotics.gitbooks.io/erlerobot/content/en/mavlink/mavlink.html
    '''
    max_payload_size = 249  # bytes, 255 - 6
    head = 0xFE # 254
    payload_keys = {0: "bytes", 1: "string", 2: "dict", 3:"list/tuple"}  # for payload_info

    def __init__(self, header, payload_size, seqnum, sender, target, payload_info, payload):
        self.header = header                # 1 byte
        self.payload_size = payload_size    # 1 byte range 0 - 249
        self.seqnum = seqnum                # 1 byte; range 0-240; for messages spanning multiple packets
        self.sender = sender                # Sending drone ID 1 byte
        self.target = target                # Receiving drone ID 1 byte
        self.payload_info = payload_info    # 1 byte; one of payload_keys
        self.payload = payload              # Read/write according to payload_info


class Transmitter(Thread):

    def __init__(self, ID, PORTS):
        Thread.__init__(self, daemon=True)
        self.ID = ID
        self.transmit_queue = Queue()       #  The internal data queue for sending messages
        self.quit = False
        self.clients = []
        self.PORTS = PORTS
    
    def read_packet(self, packet: Packet) -> bytes:
        # TODO: Make this a static/class method
        payload_info = packet.payload_info
        data = bytes([packet.header, packet.payload_size, packet.seqnum, packet.sender, packet.target, payload_info])
        if payload_info == 0:
            data += packet.payload
        elif payload_info == 1:
            data += packet.payload.encode('utf-8')
        return data
    
    def transmit_packet(self, packet) -> bool:
        target = packet.target
        client = self.clients[target]
        data = self.read_packet(packet)
        client.sendall(data)
        return True
    
    def start(self):
        print("Starting transmitter for drone ", self.ID)
        Thread.start(self)
        for ID, PORT in enumerate(self.PORTS):
            if ID == self.ID:
                continue                # Don't connect to parent drone
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            flag = 1
            while flag:
                flag = client.connect_ex((HOST, PORT))
            self.clients.append(client)

    def run(self):
        try:
            while not self.quit:
                packet = self.transmit_queue.get(block=True)
                ACK = self.transmit_packet(packet)
                if not ACK:
                    print("Transmission Error from ", self.ID, " to ", packet.target)
        except Exception as error:
            print(error)
        finally:
            print('Quitting transmitter', self.ID)
            [client.close() for client in self.clients]     # I'm lazy; feel free to unroll


class Receiver(Thread):

    def __init__(self, ID, PORTS):
        Thread.__init__(self, daemon=True)
        self.ID = ID
        self.receive_queue = Queue()             # The internal data queue for receiving messages
        self.quit = False
        self.lsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.lsock.bind((HOST, 0))
        self.PORT = self.lsock.getsockname()[1]
        PORTS[ID] = self.PORT
        print("Bound transmitter ", self.ID, " to ", self.PORT)
        self.clients = []
        self.sel = selectors.DefaultSelector()
    
    def accept_conn(self, lsock):
        conn, addr = lsock.accept()
        # print(self.ID, ' connected to by client: ', addr[1])
        conn.setblocking(False)
        self.clients.append(conn)
        self.sel.register(conn, selectors.EVENT_READ, data=addr)
        
    def start(self):
        Thread.start(self)
        self.lsock.listen()
        self.lsock.setblocking(False)
        self.sel.register(self.lsock, selectors.EVENT_READ, data=None)
        print(self.ID, " listening on ", self.PORT)
        try:
            while len(self.clients) < num_drones:
                events = self.sel.select(timeout=None)
                for key, _ in events:
                    self.accept_conn(key.fileobj)
        except Exception as error:
            print(error)
            for conn in self.clients:
                self.sel.unregister(conn)
                conn.close()
        finally:
            self.sel.unregister(self.lsock)
            self.lsock.close()
        
    def write_packet(self, recv_data: bytes) -> Packet:
        header = recv_data[0]
        payload_size = recv_data[1]
        seqnum = recv_data[2]
        sender = recv_data[3]
        target = recv_data[4]
        payload_info = recv_data[5]
        payload = recv_data[6: payload_size + 6]
        return Packet(header, payload_size, seqnum, sender, target, payload_info, payload)
    
    def read_conn(self, key, mask):
        conn = key.fileobj
        recv_data = conn.recv(1024)  # Should be ready to read
        if recv_data and recv_data != b'255':
            packet = self.write_packet(recv_data)
            self.receive_queue.put(packet)
        else:
            self.quit = True
    
    def run(self):
        try:
            while not self.quit:
                # This blocks until one of the files monitored by sel is ready for I/O
                events = self.sel.select(timeout=None)
                for key, mask in events:
                    self.read_conn(key, mask)
        except Exception as error:
            print(error)
        finally:
            for conn in self.clients:
                self.sel.unregister(conn)
                conn.close()
            self.sel.close()


class Comms:
    '''
        Instance of a drone's communication system
    '''
    def __init__(self, ID, PORTS, transmit_distance=200, transmit_modulation='QAM', num_drones=5):
        self.ID = ID                                    # Each drone should have a unique ID
        self.transmit_distance = transmit_distance      # meters
        self.transmit_modulation = transmit_modulation  # Protocol to transmit information
        self.receiver = Receiver(self.ID, PORTS)
        self.transmitter = Transmitter(self.ID, PORTS)
        self.started = False
    
    def start(self):
        if self.started:
            print("Already started comms for drone", self.ID)  # Need this otherwise...bad stuff happens?
            return
        # At this point, all drones->comms->receivers should have been intialized i.e. PORTS should be filled in
        self.receiver.start()
        self.transmitter.start()

    def terminate(self):
        self.transmitter.quit = True
        self.transmitter.join()
        self.receiver.join()                            # Main should have started KillerClient by now

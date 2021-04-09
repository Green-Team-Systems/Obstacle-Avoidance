import signal
import socket

class GracefulKiller:
    kill_now = False

    def __init__(self):
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)
        
    def exit_gracefully(self, signum, frame):
        self.kill_now = True

class KillerClient:

    def __init__(self, HOST, PORTS):
        self.clients = []
        for PORT in PORTS:
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            flag = 1
            while flag:
                flag = client.connect_ex((HOST, PORT))
            print("Killer connected to PORT: ", PORT)
            self.clients.append(client)

    def kill(self):
        for client in self.clients:
            client.sendall(b'255')
            client.close()
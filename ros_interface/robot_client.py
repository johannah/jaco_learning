import socket
import time

class RobotCommunicator():
    def __init__(self, robot_ip="132.206.73.29", port=10003):
        self.robot_ip = robot_ip
        self.port = port
        self.connected = False
        self.connect()

    def connect(self):
        while not self.connected:
            self.tcp_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
            # connect to computer
            print("attempting to connect with robot at {}".format(self.robot_ip))
            self.tcp_socket.connect((self.robot_ip, self.port))
            print('connected')
            self.connected = True
            if not self.connected:
                time.sleep(1)

    def send(self, data):
        print('sending', data)
        self.tcp_socket.sendall(data)
        print('rx', self.tcp_socket.recv(1024))

    def disconnect(self):
        self.send('close')
        print('disconnected from {}'.format(self.robot_ip))
        self.tcp_socket.close()
        self.connected = False

if __name__ == '__main__':
    rc = RobotCommunicator(robot_ip='127.0.0.1')
    rc.send('stuff')
    rc.send('stuff1')
    rc.send('stuff2')
    rc.disconnect()


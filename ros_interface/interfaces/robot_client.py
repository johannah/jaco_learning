import socket
import time
import numpy as np
from IPython import embed

class RobotCommunicator():
    def __init__(self, robot_ip="127.0.0.1", port=9100):
        self.robot_ip = robot_ip
        self.port = port
        self.connected = False
        self.connect()

    def connect(self):
        while not self.connected:
            print("attempting to connect with robot at {}".format(self.robot_ip))
            self.tcp_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
            # connect to computer
            self.tcp_socket.connect((self.robot_ip, self.port))
            print('connected')
            self.connected = True
            if not self.connected:
                time.sleep(1)

    def send(self, fn, cmd):
        data = '<|{}**{}|>'.format(fn,cmd)
        print('sending', data)
        self.tcp_socket.send(data.encode())
        ret_msg = self.tcp_socket.recv(1024).decode()
        print('rx', ret_msg)
        return ret_msg

    def disconnect(self):
        self.send('END', '')
        print('disconnected from {}'.format(self.robot_ip))
        self.tcp_socket.close()
        self.connected = False

# How fast can we actually publish commands to the robot
def run_test_routine(rc, duration_secs=1):
    cmd_freq = 50 # hz
    cmd_rate = 1.0/float(cmd_freq)
    cmd_steps = int(duration_secs/cmd_rate)
    cmds = np.zeros((cmd_steps,7))
    # move base clockwise
    cmds[:,0] = 10 
    cmds[:,3] = 100 
    # move arm back
    #cmds[30:60,1] = 10
    for i in range(cmds.shape[0]):
        rc.send("STEP", list(cmds[i]))
        time.sleep(1/cmd_rate)

def send_step(rc,n=3,step_cmd=[0,0,5,0,0,0,0]):
    for i in range(n):
        rc.send("STEP", step_cmd)
        time.sleep(1/100.)
 
if __name__ == '__main__':

    # works when client/server are both on local machine (capilano) 127.0.0.1
    # success test case is:
    # >> python robot_server.py # on capilano
    # >> python robot_client.py 127.0.0.1 # on capilano
    # fails when client/server are on different machines on lab subnet
    # failure test case is:
    # >> python robot_server.py # on capilano
    # >> python robot_client.py 132.206.73.29 # on rhys
    import sys
    #server_ip = sys.argv[1]
    #print('attempting to message server on %s - ensure it is running'%server_ip)
    #rc = RobotCommunicator(robot_ip=server_ip)
    #rc.send('RESET', 'True')
    try:
        rc = RobotCommunicator()
    except KeyboardInterrupt:
        pass
    embed()
    rc.disconnect()

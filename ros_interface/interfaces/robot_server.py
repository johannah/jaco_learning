import socket
import rospy
from ros_interface.srv import reset, step, home, get_state
import time

class RobotServer():
    def __init__(self, port=9100):
        # robot actually talks to the robot function
        self.port = port
        self.endseq = '|>'
        self.startseq = '<|'
        # between function call and data
        self.midseq = '**'
        self.prefix = 'demo'
        self.setup_ros()
        self.create_server()

    def setup_ros(self):
        print('setting up ros')
        rospy.wait_for_service('/reset')
        self.service_reset = rospy.ServiceProxy('/reset', reset)
        print('setup service: reset')
        rospy.wait_for_service('/home')
        self.service_home = rospy.ServiceProxy('/home', home)
        print('setup service: home')
        rospy.wait_for_service('/get_state')
        self.service_get_state = rospy.ServiceProxy('/get_state', get_state)
        print('setup service: get_state')
        rospy.wait_for_service('/step')
        self.service_step = rospy.ServiceProxy('/step', step)
        print('setup service: step')
        print('finished setting up ros')

    def state_callback(self, msg):
        self.state = msg.state

    def create_server(self):
        print('starting server')
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(('localhost', self.port))
        self.server_socket.listen(1)
        self.connected = False
        self.listen()

    def disconnect(self):
        if self.connected:
            self.server_socket.close()
            self.connected = False

    def handle_msg(self, fn, cmd):
        # todo decode the ros messges here with relevant info
        msg = 'NOTIMP'
        if fn == 'RESET':
            response = self.service_reset()
            msg = str(response.success)
        elif fn == 'GET_STATE':
            response = self.service_get_state()
            msg = str(response)
        elif fn == 'HOME':
            response = self.service_home()
            msg = str(response)
        elif fn == 'STEP':
            # cmd should be list of floats
            # check cmd
            cvars = [x for x in cmd[1:-1].strip().split(',')]
            # str
            ctype = cvars[0]
            relative = bool(cvars[1])
            unit = str(cvars[2])
            data = [float(x) for x in cvars[3:]]
            response = self.service_step(ctype, relative, unit, data)
            msg = str(response)
 
        elif fn == 'END':
            self.disconnect()
            self.create_server()
            msg = 'END'
        else:
            msg = 'NOTIMP'
        ret_msg = self.startseq+'ACK'+fn+self.midseq+msg+self.endseq
        print("SERVER RESPONDS WITH", ret_msg)
        return ret_msg

    def listen(self):
        while not self.connected:
            connection, client_address = self.server_socket.accept()
            print('connected to', client_address)
            self.connected = True
            while self.connected:
                try:
                    # every message needs a response before it will send a new
                    # message
                    # TODO - will need to handle large messages eventually, but
                    # leave this for now
                    rx_data = connection.recv(1024)
                    if rx_data:
                        rx_data = rx_data.decode()
                        if rx_data.endswith(self.endseq):
                            fn, cmd = rx_data[len(self.startseq):-len(self.endseq):].split(self.midseq)
                            ret_msg = self.handle_msg(fn, cmd)
                            connection.sendall(ret_msg.encode())
                    else:
                        time.sleep(.1)
                except KeyboardInterrupt as e:
                    self.server_socket.close()
                    break

if __name__ == '__main__':
    from IPython import embed
    rs = RobotServer()


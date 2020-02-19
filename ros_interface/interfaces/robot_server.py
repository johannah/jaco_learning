import socket
from srv import reset, step, home
from msg import

class RobotServer():
    def __init__(self, port=9100, robot):
        # robot actually talks to the robot function
        self.robot = robot
        self.port = port
        self.endseq = '|>'
        self.startseq = '<|'
        # between function call and data
        self.midseq = '**'
        self.prefix = 'demo'
        self.setup_ros()
        self.create_server()

    def setup_ros(self):
        self.service_reset = rospy.ServiceProxy('reset', reset)
        self.service_step = rospy.ServiceProxy('step', step)
        self.state_subscriber = rospy.Subscriber(self.prefix + "_driver/out/state", JointState,
                                                 self.receive_joint_state, queue_size=50)

    def state_callback(self, msg):
        self.state = msg.state

    def create_server(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(('localhost', self.port))
        self.server_socket.listen(1)
        self.connected = False
        self.listen()

    def handle_msg(self, fn, cmd):
        # todo decode the ros messges here with relevant info
        if fn == 'RESET':
            msg = self.service_reset()
        elif fn == 'HOME:
            msg = self.service_home(cmd)
        elif fn == 'STEP':
            msg = self.service_step(cmd)
        elif fn == 'GETSTATE':
            msg = self.state
        else:
            msg = 'NOTIMP'
        ret_msg = self.startseq+'ACK'+fn+self.midseq+msg+self.endseq
        return ret_msg

    def listen(self):
        while not self.connected:
            connection, client_address = self.server_socket.accept()
            print('connected to', client_address)
            self.connected = True
            while self.connected:
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

rs = RobotServer()


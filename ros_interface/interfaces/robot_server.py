import socket
import rospy
from ros_interface.srv import initialize, reset, step, home, get_state
import time

class RobotServer():
    def __init__(self, port=9030):
        # robot actually talks to the robot function
        self.port = port
        self.endseq = '|>'
        self.startseq = '<|'
        # between function call and data
        self.midseq = '**'
        rospy.init_node('robot_server')
        self.setup_ros()
        self.create_server()

    def setup_ros(self):
        print('setting up ros')
        rospy.wait_for_service('/initialize')
        self.service_init = rospy.ServiceProxy('/initialize', initialize)
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

    def handle_msg(self, fn, cmd):
        # todo decode the ros messges here with relevant info
        fn = fn.upper()
        msg = 'NOTIMP'
        print("handling fn: {}".format(fn))
        print("cmd is:{}".format(cmd))
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
            cvars = [x for x in cmd.strip().split(',')]
            ctype = cvars[0]
            relative = bool(cvars[1])
            unit = str(cvars[2])
            data = cvars[3:]
            data = [float(x) for x in data]
            response = self.service_step(ctype, relative, unit, data)
            msg = str(response)
        elif fn == 'INIT':
            fence_vars = [x for x in cmd.strip().split(',')]
            fence_vars = [float(x) for x in fence_vars]
            # min/max fence for xyz
            assert(len(fence_vars) == 6)
            print("setting fence", fence_vars)
            response = self.service_init(*fence_vars)
            msg = str(response)
        elif fn == 'END':
            # this is just used for the local server and won't be sent to jaco
            # TODO maybe it should be used to stop ros processes and shutdown ....
            self.disconnect()
            self.create_server()
            msg = 'ENDED'
        else:
            msg = 'NOTIMP'
        ret_msg = self.startseq+'ACK'+fn+self.midseq+msg+self.endseq
        print("SERVER RESPONDS WITH", ret_msg)
        return ret_msg

    def create_server(self):
        print('starting server at %s'%self.port)
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # 0.0.0.0 will accept from any address - makes this work on docker 
        self.server_socket.bind(('0.0.0.0', self.port))
        self.server_socket.listen(5)
        self.connected = False
        self.listen()

    def disconnect(self):
        if self.connected:
            self.server_socket.close()
            self.connected = False


    def listen(self):
        try:
            while not self.connected:
                connection, client_address = self.server_socket.accept()
                print('connected to', client_address)
                self.connected = True
                while self.connected:
                    try:
                        print('-waiting for next cmd-')
                        # every message needs a response before it will send a new
                        # message
                        # TODO - will need to handle large messages eventually, but
                        # leave this for now
                        rx_data = connection.recv(1024)
                        if rx_data:
                            rx_data = rx_data.decode().strip()
                            print("rx", rx_data)
                            if rx_data.endswith(self.endseq):
                                fn, cmd = rx_data[len(self.startseq):-len(self.endseq):].split(self.midseq)
                                ret_msg = self.handle_msg(fn, cmd)
                                connection.sendall(ret_msg.encode())
                            else:
                                print(rx_data, 'does not end with', self.endseq)
  
                        else:
                            time.sleep(1)
                    except KeyboardInterrupt as e:
                          print("rcvd interrupt from client loop- closing")
                          self.disconnect()
                          sys.exit()
 
        except KeyboardInterrupt as e:
            print("rcvd interrupt - closing")
            self.disconnect()
            sys.exit()
                    

if __name__ == '__main__':
    from IPython import embed
    rs = RobotServer()


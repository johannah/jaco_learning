#! /usr/bin/env python
import os
import sys
import thread
import socket
import rospy
from sensor_msgs.msg import Image
from ros_interface.srv import initialize, reset, step, home, get_state
import time
import numpy as np
import threading 
import json
import zlib

class RobotServer():
    def __init__(self, port=9030):
        # robot actually talks to the robot function
        self.count = 0
        self.client_num = 0
        self.port = port
        self.endseq = '|>'
        self.startseq = '<|'
        # between function call and data
        self.midseq = '**'
        rospy.init_node('robot_server')
        self.image_lock = threading.Lock()
        self.image_string = 'none'
        self.image_height = 0
        self.image_width = 0
        self.image_encoding = 'none'
        self.setup_ros()
        self.create_server()
        #rospy.spin()

    def setup_ros(self):
        self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.image_callback)

        rospy.loginfo('setting up ros')
        rospy.wait_for_service('/initialize')
        self.service_init = rospy.ServiceProxy('/initialize', initialize)
        rospy.wait_for_service('/reset')
        self.service_reset = rospy.ServiceProxy('/reset', reset)
        rospy.loginfo('setup service: reset')
        rospy.wait_for_service('/home')
        self.service_home = rospy.ServiceProxy('/home', home)
        rospy.loginfo('setup service: home')
        rospy.wait_for_service('/get_state')
        self.service_get_state = rospy.ServiceProxy('/get_state', get_state)
        rospy.loginfo('setup service: get_state')
        rospy.wait_for_service('/step')
        self.service_step = rospy.ServiceProxy('/step', step)
        rospy.loginfo('setup service: step')
        rospy.loginfo('finished setting up ros')

    def get_image_string(self):
        return self.image_data

    def image_callback(self, msg):
        self.image_data = msg.data
        self.image_height = str(msg.height).encode('utf-8')
        self.image_width = str(msg.width).encode('utf-8')
        self.image_encoding = msg.encoding.encode('utf-8')

    def handle_msg(self, fn, cmd):
        fn = str(fn.upper())
        msg = 'NOTIMP'
        rospy.loginfo("handling fn: {}".format(fn))
        rospy.loginfo("cmd is:{}".format(cmd))

        if fn == 'RESET':
            response = self.service_reset()
            msg = str(response)
        elif fn == 'GET_STATE':
            response = self.service_get_state()
            msg = str(response)
        elif fn == 'STEP':
            # cmd should be list of floats
            cvars = [x for x in cmd.strip().split(',')]
            ctype = cvars[0]
            relative = int(cvars[1])
            print("SENDING STEP RELATIVE", relative)
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
            msg = 'ENDED'
        elif fn == 'HOME':
            response = self.service_home()
            msg = str(response)
        elif fn == 'RENDER':
            msg = self.get_image_string()
            return self.startseq+msg+self.endseq
        else:
            msg = 'NOTIMP'
        ret_msg = self.startseq+'ACK'+fn+self.midseq+msg+self.endseq
        ret_msg = ret_msg.encode()
        return ret_msg

    def create_server(self):
        print('starting server at %s'%self.port)
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # 0.0.0.0 will accept from any address - makes this work on docker 
        self.server_socket.bind(('0.0.0.0', self.port))
        self.server_socket.listen(5)
        self.server_socket.settimeout(180)
        self.connected = False
        while True:
            try:
                c, addr = self.server_socket.accept()
                thread.start_new_thread(self.chat_with_client, (c,addr))
                self.client_num +=1 
            except Exception as e:
                print(e)
                self.disconnect()
                sys.exit()

    def disconnect(self):
        if self.connected:
            self.server_socket.close()
            self.connected = False

    def chat_with_client(self, connection, client_address):
        print('connected to client:{} at {}'.format(self.client_num, client_address))
        connected = True
        while connected:
            try:
                # every message needs a response before it will send a new
                # message
                # TODO - will need to handle large messages eventually, but
                # leave this for now
                rx_data = connection.recv(100000)
                if rx_data:
                    rx_data = rx_data.decode().strip()
                    print("rx", rx_data)
                    if rx_data.endswith(self.endseq):
                        fn, cmd = rx_data[len(self.startseq):-len(self.endseq):].split(self.midseq)
                        ret_msg = self.handle_msg(fn, cmd)
                        connection.sendall(ret_msg)
                        if fn.upper() == 'END':
                            connected = False
                    else:
                        print(rx_data, 'does not end with', self.endseq)
  
                else:
                    time.sleep(.1)
            except Exception as e:
                print('EXCEPTION: {}'.format(e))
                self.disconnect()
                sys.exit()
        connection.close()
 
                    

if __name__ == '__main__':
    from IPython import embed
    rs = RobotServer()


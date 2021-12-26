import matplotlib
matplotlib.use("Agg")
import numpy as np
import os
import sys
import fence
import matplotlib.pyplot as plt
from IPython import embed
import rospy
from ros_interface.srv import reset, step, home, get_state

class JacoJointTest():
    def __init__(self):
        self.setup_ros()

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
 
        self.eef_pos = [] 
        self.joint_pos = []

    def move_joint(self, joint, offset_degrees):
        joints = np.zeros(7)
        joints[joint] = offset_degrees
        ss = self.service_step('ANGLE', True, 'mdeg', joints)
        self.joint_pos.append(ss.joint_pos)
        self.eef_pos.append(ss.tool_pos)

 
if __name__ == '__main__':
    jtest = JacoJointTest()
    n_joints = 7
    joint_offset = 10
    jtest.move_joints(0, 10)
          
            
    
    
    

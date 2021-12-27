import matplotlib
matplotlib.use("Agg")
import numpy as np
import os
import sys
import matplotlib.pyplot as plt
from IPython import embed
import rospy
from ros_interface.srv import reset, step, home, get_state, initialize

class JacoJointTest():
    def __init__(self):
        self.setup_ros()
        # max joint step size is 10 degrees
        # otherwise robosuite steps dont work well
        self.max_joint_step = 10

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
        self.service_init()
        self.reset_data()
 
    def reset_data(self):
        self.eef_pos = [] 
        self.joint_pos = []
        self.actions = []

    def add_state(self):
        ss = self.service_get_state()
        self.joint_pos.append(ss.joint_pos)
        self.eef_pos.append(ss.tool_pos)
        self.actions.append(np.zeros(7))
  
    def save_data(self, filename):
        np.savez(filename, joint_pos=jtest.joint_pos, eef_pos=self.eef_pos, actions=self.actions)

    def move_joint(self, joint, offset_degrees):
        print('starting', offset_degrees)
        while np.abs(offset_degrees) > 0:
            step = np.sign(offset_degrees) * np.min([np.abs(self.max_joint_step), np.abs(offset_degrees)])
            offset_degrees = np.sign(offset_degrees) * (np.abs(offset_degrees)-np.abs(step))
            print(step, offset_degrees)
            joints = np.zeros(8)
            joints[joint] = step
            ss = self.service_step('ANGLE', True, 'mdeg', joints)
            self.joint_pos.append(ss.joint_pos)
            self.eef_pos.append(ss.tool_pos)
            self.actions.append(joints)
            print(ss.success, ss.joint_pos[joint])

 
def joint_0_full_revolution():
    """
     turn all around base axis
    """
    jtest.service_reset()
    jtest.reset_data()
    jtest.add_state()
    for x in np.arange(0, 400, jtest.max_joint_step):
        jtest.move_joint(0, jtest.max_joint_step)
    jtest.add_state()
    jtest.save_data('datasets/joint_0_full_revolution')

 
def all_joints_move():
    """
     go back and forth for each joint
    """
    jtest.service_reset()
    jtest.reset_data()
    jtest.add_state()
    # for joint 3 (big elbow, go up first! (positive))
    jtest.move_joint(3, 20)
    offset = 20
    for jt in range(7):
        jtest.move_joint(jt, offset)
        jtest.move_joint(jt, -offset)
    jtest.add_state()
    jtest.save_data('datasets/all_joints_move')

 
def move_tool_orientation():
    """
     move over center axis with tool orientation change
    """
    jtest.service_reset()
    jtest.reset_data()
    jtest.add_state()
    # for joint 3 (big elbow, go up first! (positive))
    #for jt3 in [150]:
    jtest.move_joint(3, 60)
    jtest.move_joint(4, 15)
    jtest.move_joint(1, 20)
    jtest.move_joint(4, 15)
    jtest.move_joint(3, 60)
    jtest.move_joint(4, 15)
    jtest.move_joint(1, 20)
    jtest.move_joint(4, 15)
    jtest.move_joint(3, 60)
    jtest.move_joint(4, 15)
    jtest.move_joint(1, 20)
    jtest.move_joint(4, 15)
    jtest.move_joint(3, 60)
    jtest.move_joint(4, 15)
    jtest.move_joint(1, -20)
    jtest.move_joint(3, 30)
    jtest.save_data('datasets/tool_orientation')

 
def finger_joints_move():
    """
    FINGERS DON"T REALLY WORK CORRECTLY YET
    """
    #jtest.service_reset()
    jtest.reset_data()
    jtest.add_state()
    # for joint 3 (big elbow, go up first! (positive))
    # open -> 0
    for ii in np.linspace(-1, 1, 10):
        print(ii)
        jtest.move_joint(7,10) 
    for ii in np.linspace(-1, 1, 10):
        print(ii)
        jtest.move_joint(7,-10) 
 
    jtest.save_data('datasets/finger_joints_move')




if __name__ == '__main__':
    jtest = JacoJointTest()
    data_dir = 'datasets'
    if not os.path.exists(data_dir):
        os.makedir(data_dir)
    
    #joint_0_full_revolution()
    all_joints_move()
    move_tool_orientation()
    ##finger_joints_move()
          
            
    
    
    

#! /usr/bin/env python

# *******************************************************************
# Modified with permission from :
# Author: Sahand Rezaei-Shoshtari
# Oct. 2019
# Copyright 2019, Sahand Rezaei-Shoshtari, All rights reserved.
# *******************************************************************
# https://github.com/sahandrez/jaco_control

import os
import numpy as np
import pid
#import trajectory
import threading

# ROS libs
import rospy
import rospkg
import actionlib
import tf
import tf.transformations
import tf2_ros
#import dynamic_reconfigure.server
#from jaco_control.cfg import controller_gainsConfig
from base import BaseConfig

# ROS messages and services
from std_msgs.msg import Float64, Header
from geometry_msgs.msg import Pose, PoseStamped, Twist, TwistStamped, Vector3, Point, Quaternion, Wrench, WrenchStamped
from sensor_msgs.msg import JointState
#from gazebo_msgs.msg import LinkState#, LinkStates
from kinova_msgs.msg import JointVelocity, JointTorque, JointAngles
from kinova_msgs.msg import ArmJointAnglesGoal, ArmJointAnglesAction, SetFingersPositionAction, SetFingersPositionGoal
from kinova_msgs.srv import HomeArm, SetTorqueControlMode, SetTorqueControlParameters
#from jaco_control.msg import InteractionParams

from ros_interface.srv import reset, step, home, get_state

# todo - force this to load configuration from file should have safety params
# torque, velocity limits in it

radians_to_deg = lambda angle: (180. * angle / np.pi)

class JacoConfig(BaseConfig):
    def __init__(self):
        pass

    def define_config_dependent_variables(self):
        self.robot_name = self.cfg['base']['robot_name']
        self.n_joints = int(self.robot_name[3])
        self.server_port = self.cfg['base']['server_port']
        # Robot parameters
        self.prefix = '/' + self.robot_name
        # from sahand
        self.MAX_FINGER_TURNS = 6800
        self.set_PID()

    def verify_config(self):
        """ check important parts of the config"""
       # assert num join_configs is the same as the n_joints

    def set_PID(self):
        # velocity controller gains
        self.P = np.diag([self.cfg['velocity_kp_gains']['joint_%d'%n] for n in range(self.n_joints)])
        self.D = np.diag([self.cfg['velocity_kd_gains']['joint_%d'%n] for n in range(self.n_joints)])
        self.I = 0.0 * np.eye(self.n_joints)

class JacoRobot():

    def init_ros(self):
        rospy.loginfo('starting init of ros')
        print('starting init of ros')
        self.prefix = '/j2n7s300'
        rospy.init_node('jaco_controller', anonymous=True)

        self.active_controller = 'velocity'
        # init services
        self.path_home_arm = self.prefix + '_driver/in/home_arm'
        rospy.loginfo('waiting for service ---> %s'%self.path_home_arm)
        rospy.wait_for_service(self.path_home_arm)
        self.home_robot_service = rospy.ServiceProxy(self.path_home_arm, HomeArm)
        # Joint velocity command publisher - send commands to the kinova driver
        self.path_joint_vel = self.prefix + '_driver/in/joint_velocity'
        self.joint_velocity_publisher = rospy.Publisher(self.path_joint_vel, 
                                                        JointVelocity, 
                                                        queue_size=50)
 
        # Callback data holders
        self.robot_joint_state = JointState()
        rospy.loginfo("Jaco controller init successful.")

    def connect_to_robot(self):
        """
        Connects to the robot.
        :return: None
        """
        topics = [name for (name, _) in rospy.get_published_topics()]
        if self.prefix + '_driver/out/joint_state' not in topics:
            rospy.logerr("COULD NOT connect to the robot.")
            raise

        self.state_lock = threading.Lock()
        self.path_joint_state = self.prefix + "_driver/out/joint_state"
        self.state_subscriber = rospy.Subscriber(self.path_joint_state, 
                                                 JointState, 
                                                 self.receive_joint_state, 
                                                 queue_size=50)

        rospy.loginfo("Connected to the robot")


    def receive_joint_state(self, robot_joint_state):
        """
        Callback for '/prefix_driver/out/joint_state'.
        :param robot_joint_state: data from topic
        :type robot_joint_state JointState
        :return None
        """
        self.state_lock.acquire()
        self.robot_joint_state = robot_joint_state
        self.state_lock.release()

    def create_joint_velocity_cmd(self, velocity):
        """
        Creates a joint velocity command with the target velocity for each joint.
        :param velocity: velocity of each joint in deg/s
        :type velocity: np.array
        :return: joint velocity command
        :rtype: JointVelocity
        """
        # init
        # TODO - convert to degrees if flagged
        # deg_vel = [radian_to_deg(x) for x in  velocity]
        print(velocity)
        
        velocity = velocity.reshape(-1)
        joint_cmd = JointVelocity()
        joint_cmd.joint1 = velocity[0]
        joint_cmd.joint2 = velocity[1]
        joint_cmd.joint3 = velocity[2]
        joint_cmd.joint4 = velocity[3]
        joint_cmd.joint5 = velocity[4]
        joint_cmd.joint6 = velocity[5]
        joint_cmd.joint7 = 0.0
        if self.n_joints == 7:
            joint_cmd.joint7 = velocity[6]
        return joint_cmd

#    def send_joint_velocity_cmd(self, joint_cmd):
#        """
#        Publishes the joint velocity command to the robot.
#        :param joint_cmd: desired joint velocities
#        :type joint_cmd: JointVelocity
#        :return: None
#        """
#        self.joint_velocity_publisher.publish(joint_cmd)

    def shutdown_controller():
        """
        Shuts down the controller.
        :return: None
        """
        rospy.loginfo("Shutting Down Controller.")
        rospy.signal_shutdown('Done')
        return exit(0)


class Jaco(JacoRobot):
    def __init__(self):
        #rospy.init_node('dm_jaco_controller', anonymous=True)
        # state passed in 6dof mujoco has 37 dimensions
        # our 7DOF has 39 dimensions
        self.n_joints = 7
        self.init_ros()
        self.empty_state = np.zeros((37))
        rospy.loginfo('initiating reset service')
        # instantiate services to be called by dm_wrapper
        self.server_reset = rospy.Service('/reset', reset, self.reset)
        self.server_get_state = rospy.Service('/get_state', get_state, self.get_state)
        self.server_home = rospy.Service('/home', home, self.home)
        self.server_step = rospy.Service('/step', step, self.step)


    def get_state(self, msg=None, success=True):
        """ :msg is not used - this returns state regardless of message passed in (for service calls)
            :success bool to indicate if a cmd was successfully executed
        
            in dm_control for 6dof robot - state is an OrderedDict([
                   'arm_pos' of shape (9,2) -> jaco_joint_1-6 and jaco_joint_finger1-6 
                   'arm_vel' of shape (9,1)
                   'hand_pos is shape (7,)
                   'target_pos' is shape (3,)
        """
        name = []
        joint_pos = []
        joint_vel = []
        joint_effort = []
        self.state_lock.acquire()
        for (idx, joint_name) in enumerate(self.robot_joint_state.name):
            name.append(joint_name)
            joint_pos.append(self.robot_joint_state.position[idx])
            joint_vel.append(self.robot_joint_state.velocity[idx])
            joint_effort.append(self.robot_joint_state.effort[idx])
        self.state_lock.release()
        return success, name, joint_pos, joint_vel, joint_effort

    def step(self, cmd):
        print(cmd)
        # TODO - should we not use an invalid command or should
        cmd_vel = np.array(cmd.velocity)
        print('rx', cmd_vel)
        if self.check_action_safety(cmd_vel):
            cmd_vel_msg = self.create_joint_velocity_cmd(cmd_vel)
            print("SENDING ROBOT")
            print(cmd_vel_msg)
            self.joint_velocity_publisher.publish(cmd_vel_msg)
            print("SUCCESS IN SENDING VELOCITY")
            return self.get_state(success=True)
        else:
            print("FAILURE IN SENDING VELOCITY")
            return self.get_state(success=False)

    def home(self, msg=None):
        print('calling home')
        self.home_robot_service()
 
    def check_action_safety(self, action):
        # action in deg/second
        if action.max() < 15:
            return True
        else:
            return False

    def reset(self, msg=None):
        print('calling reset')
        self.home_robot_service()
        # home position looks like
        #name: [ j2n7s300_joint_1, j2n7s300_joint_2, j2n7s300_joint_3, 
        #        j2n7s300_joint_4, j2n7s300_joint_5, j2n7s300_joint_6, j2n7s300_joint_7, 
        #      j2n7s300_joint_finger_1, j2n7s300_joint_finger_2, j2n7s300_joint_finger_3, 
        #      j2n7s300_joint_finger_tip_1, j2n7s300_joint_finger_tip_2, j2n7s300_joint_finger_tip_3]
        #      position: [4.708371868196414, 2.6192210626218024, 0.0009016398446646594, 
        #                 0.5213664806830808, -0.004190159922630479, 3.7136391231065957, 9.430082977660817, 
        #                 1.108797407149339, 1.108797407149339, 1.1100294042683936, 0.0, 0.0, 0.0]
        #      velocity: [-0.007791646811650356, -0.007791648892244958, -4.891454401121671e-47, 
        #          -0.007791648892244958, 4.891454401121671e-47, -0.011333306723049174, -0.011333306723049174, 
        #                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        #      effort: [0.38293781876564026, 1.250989317893982, 0.7754424810409546, 
        #            6.08735466003418, 0.2894153892993927, -1.0226423740386963, -0.01134837232530117, 
        #             0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # this function does not return until the arm has reached the home position
        return self.get_state()
    

if __name__ == '__main__':
    jaco = Jaco()
    jaco.connect_to_robot()
    try:
        rospy.spin()
    except KeyBoardInterrupt:
        sys.exit()


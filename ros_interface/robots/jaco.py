#! /usr/bin/env python

# *******************************************************************
# Modified with permission from :
# Author: Sahand Rezaei-Shoshtari
# Oct. 2019
# Copyright 2019, Sahand Rezaei-Shoshtari, All rights reserved.
# *******************************************************************
# https://github.com/sahandrez/jaco_control
"""
origin is the intersection point of the bottom plane of the base and cylinder center line.
+x axis is directing to the left when facing the base panel (where power switch and cable socket locate).
+y axis is towards to user when facing the base panel.
+z axis is upwards when robot is standing on a flat surface.

This is the current joint angle in degrees
/j2n7s300_driver/out/joint_command

"""

import os
from copy import copy
import numpy as np
import pid
import time
import math
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
from kinova_msgs.msg import JointVelocity, JointTorque, JointAngles, KinovaPose
from kinova_msgs.msg import ArmJointAnglesGoal, ArmJointAnglesAction, SetFingersPositionAction, SetFingersPositionGoal, ArmPoseAction, ArmPoseGoal
from kinova_msgs.srv import HomeArm, SetTorqueControlMode, SetTorqueControlParameters

from utils import Quaternion2EulerXYZ, EulerXYZ2Quaternion, trim_target_pose_safety
from utils import convert_tool_pose, convert_joint_angles, convert_to_degrees
#from jaco_control.msg import InteractionParams
from ros_interface.srv import initialize, reset, step, home, get_state

# todo - force this to load configuration from file should have safety params
# torque, velocity limits in it


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

class JacoRobot(object):
    def __init__(self, robot_type='j2n7s300', cfg=JacoConfig()):
        self.state_lock = threading.Lock()
        self.reset_state_trace()
        self.tool_pose_lock = threading.Lock()

        self.n_states = 0
        self.request_timeout_secs = 10
        rospy.loginfo('starting init of ros')
        self.robot_type = robot_type
        self.prefix = '/{}'.format(robot_type)
        rospy.init_node('jaco_controller', anonymous=True)

        # init services
        self.path_home_arm = self.prefix + '_driver/in/home_arm'
        rospy.loginfo('waiting for service ---> %s'%self.path_home_arm)
        rospy.wait_for_service(self.path_home_arm)
        self.home_robot_service = rospy.ServiceProxy(self.path_home_arm, HomeArm)
        ## Joint velocity command publisher - send commands to the kinova driver
        self.path_joint_vel = self.prefix + '_driver/in/joint_velocity'
        self.joint_velocity_publisher = rospy.Publisher(self.path_joint_vel, 
                                                        JointVelocity, 
                                                        queue_size=50)
 
        # Callback data holders
        self.robot_joint_state = JointState()

        self.joint_angle_requester_path = self.prefix + '_driver/joints_action/joint_angles'
        self.joint_angle_requester = actionlib.SimpleActionClient(self.joint_angle_requester_path, 
                                                               ArmJointAnglesAction)
        self.tool_pose_requester_path = self.prefix+'_driver/pose_action/tool_pose'
        self.tool_pose_requester =  actionlib.SimpleActionClient(self.tool_pose_requester_path, 
                                                                  ArmPoseAction)

        rospy.loginfo("Jaco controller init successful.")

    def add_fake_n_states(self):
        self.n_states +=1

    def connect_to_robot(self):
        """
        Connects to the robot.
        :return: None
        """
        print('connecting to robot')
        topics = [name for (name, _) in rospy.get_published_topics()]
        if self.prefix + '_driver/out/joint_state' not in topics:
            rospy.logerr("COULD NOT connect to the robot.")
            raise

        # tool pose for end effector
        self.tool_pose_rcvd = False
        self.tool_pose_out_address = self.prefix + '_driver/out/tool_pose'
        self.tool_pose_subscriber = rospy.Subscriber(self.tool_pose_out_address, 
                                                     PoseStamped, 
                                                     self.receive_tool_pose, 
                                                     queue_size=10)

        while not self.tool_pose_rcvd:
            rospy.loginfo("waiting on tool pose... ")
            try:
                time.sleep(.5)
            except KeyboardInterrupt as e:
                sys.exit()
 
        self.joint_state_rcvd = False
        self.path_joint_state = self.prefix + "_driver/out/joint_state"
        self.state_subscriber = rospy.Subscriber(self.path_joint_state, 
                                                 JointState, 
                                                 self.receive_joint_state, 
                                                 queue_size=10)
        while not self.joint_state_rcvd:
            rospy.loginfo("waiting on joint state...")
            try:
                time.sleep(.5)
            except KeyboardInterrupt as e:
                sys.exit()
        ################################################
        rospy.loginfo("Connected to the robot")

    def reset_state_trace(self):
        self.state_lock.acquire()
        self.n_states = 0
        self.state_start = time.time()
        self.state_trace = {'n_states':0,
                             'time_offset':[], 
                             'joint_pos':[], 
                             'joint_vel':[], 
                             'joint_effort':[], 
                             'tool_pose':[]
                             }
        self.state_lock.release()

    def receive_joint_state(self, robot_joint_state_msg):
        """
        Callback for '/prefix_driver/out/joint_state'.
        :param robot_joint_state: data from topic
        :type robot_joint_state JointState
        :return None
        """
        self.state_lock.acquire()
        robot_joint_state = copy(robot_joint_state_msg)
        self.joint_angles = robot_joint_state_msg.position
        self.state_lock.release()

        self.state_trace['n_states']+=1
        self.state_trace['time_offset'].append(time.time()-self.state_start)
        self.state_trace['joint_pos'].extend(robot_joint_state.position)
        self.state_trace['joint_vel'].extend(robot_joint_state.velocity)
        self.state_trace['joint_effort'].extend(robot_joint_state.effort)
        robot_tool_pose = self.get_tool_pose()
        tool_pose  = [robot_tool_pose.pose.position.x, 
                      robot_tool_pose.pose.position.y, 
                      robot_tool_pose.pose.position.z, 
                      robot_tool_pose.pose.orientation.x, robot_tool_pose.pose.orientation.y, 
                      robot_tool_pose.pose.orientation.z, robot_tool_pose.pose.orientation.w]
        self.state_trace['tool_pose'].extend(tool_pose)

        self.joint_state_rcvd = True

    def get_state_trace(self):
        self.state_lock.acquire()
        st = copy(self.state_trace)
        self.state_lock.release()
        return st

    def get_joint_angles(self):
        self.state_lock.acquire()
        ja = self.joint_angles
        self.state_lock.release()
        return ja

            
    def get_tool_pose(self):
        self.tool_pose_lock.acquire()
        robot_tool_pose = copy(self.robot_tool_pose)
        self.tool_pose_lock.release()
        return robot_tool_pose

    def receive_tool_pose(self, robot_tool_pose):
        """
        tool_pose is constantly updated
        Callback for '/prefix_driver/out/tool_pose'
        # in quaternians
        :param robot_tool_pose: data from topic
        :type robot_tool_pose kinova_msgs.msg.KinovaPose
        :return None
        """
        self.tool_pose_lock.acquire()
        self.robot_tool_pose = robot_tool_pose 
        self.tool_pose_lock.release()
        self.tool_pose_rcvd = True

    def send_tool_pose_cmd(self, position, orientation_q):
        #robot_tool_pose = self.get_tool_pose()
 
        print("REQUESTING POSE before fence of:", position)
        position, result = trim_target_pose_safety(position, self.fence_min_x, self.fence_max_x, 
                                                             self.fence_min_y, self.fence_max_y, 
                                                             self.fence_min_z, self.fence_max_z)
        # based on pose_action_client.py
        print("REQUESTING POSE after fence of:", position)
        # TODO - does wait_for_server belong here or when it is defined?
        self.tool_pose_requester.wait_for_server()
        goal = ArmPoseGoal()
        goal.pose.header = Header(frame_id=(self.prefix+'_link_base'))
        goal.pose.pose.position = Point(x=position[0], y=position[1], z=position[2])
        goal.pose.pose.orientation = Quaternion(x=orientation_q[0], y=orientation_q[1], z=orientation_q[2], w=orientation_q[3])
        self.tool_pose_requester.send_goal(goal)
        if self.tool_pose_requester.wait_for_result(rospy.Duration(self.request_timeout_secs)):
            self.tool_pose_requester.get_result()
            result+='+TOOL_POSE_FINISHED'
            robot_tool_pose = self.get_tool_pose()
            this_position = [robot_tool_pose.pose.position.x, robot_tool_pose.pose.position.y, robot_tool_pose.pose.position.z]
            success = True
        else:
            self.tool_pose_requester.cancel_all_goals()
            result += '+TIMEOUT' 
            success = False
        return result, success


    def send_joint_angle_cmd(self, joint_angles_degrees):
        """
        create joint target pose to send to the controller
        Sends the joint angle command to the action server and waits for its execution. 
        Note that the planning is done in the robot base.
        """
        # TODO add safety planner
        #joint_angles_degrees, result = self.check_target_pose_safety(joint_angles_degrees)
        joint_cmd = ArmJointAnglesGoal()
        joint_cmd.angles.joint1 = joint_angles_degrees[0] 
        joint_cmd.angles.joint2 = joint_angles_degrees[1] 
        joint_cmd.angles.joint3 = joint_angles_degrees[2] 
        joint_cmd.angles.joint4 = joint_angles_degrees[3] 
        joint_cmd.angles.joint5 = joint_angles_degrees[4] 
        joint_cmd.angles.joint6 = joint_angles_degrees[5] 
        if len(joint_angles_degrees) == 7:
            joint_cmd.angles.joint7 = joint_angles_degrees[6] 
        self.joint_angle_requester.send_goal(joint_cmd)

        result = ''
        if self.joint_angle_requester.wait_for_result(rospy.Duration(self.request_timeout_secs)):
            self.joint_angle_requester.get_result()
            result+='+JOINT_ANGLE_FINISHED'
            robot_joint_angles = self.get_joint_angles()
            #this_position = [robot_tool_pose.pose.position.x, robot_tool_pose.pose.position.y, robot_tool_pose.pose.position.z]
            success = True
        else:
            self.joint_angle_requester.cancel_all_goals()
            result += '+TIMEOUT' 
            success = False
        return result, success


        goal = ArmPoseGoal()
        goal.pose.header = Header(frame_id=(self.prefix+'_link_base'))
        goal.pose.pose.position = Point(x=position[0], y=position[1], z=position[2])
        goal.pose.pose.orientation = Quaternion(x=orientation_q[0], y=orientation_q[1], z=orientation_q[2], w=orientation_q[3])
        self.tool_pose_requester.send_goal(goal)
        if self.tool_pose_requester.wait_for_result(rospy.Duration(self.request_timeout_secs)):
            self.tool_pose_requester.get_result()
            result+='+TOOL_POSE_FINISHED'
            robot_tool_pose = self.get_tool_pose()
            this_position = [robot_tool_pose.pose.position.x, robot_tool_pose.pose.position.y, robot_tool_pose.pose.position.z]
            success = True
        else:
            self.tool_pose_requester.cancel_all_goals()
            result += '+TIMEOUT' 
            success = False
        return result, success

    def send_joint_velocity_cmd(self, velocity):
        """
        Creates a joint velocity command with the target velocity for each joint.
        :param velocity: velocity of each joint in deg/s
        :type velocity: np.array
        :return: joint velocity command
        :rtype: JointVelocity
        Velocity control seems like it is too slow to be useful
        """
        # TODO check safety
        success = True
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
        print('publishing', joint_cmd)
        self.joint_velocity_publisher.publish(joint_cmd)
        return 'sent', success

    def shutdown_controller():
        """
        Shuts down the controller.
        :return: None
        """
        rospy.loginfo("Shutting Down Controller.")
        rospy.signal_shutdown('Done')
        return exit(0)


class JacoInterface(JacoRobot):
    def __init__(self, robot_type='j2n7s300'):
        #rospy.init_node('dm_jaco_controller', anonymous=True)
        # state passed in 6dof mujoco has 37 dimensions
        # our 7DOF 7 major joints and 6 fingerjoints
        self.n_joints = int(robot_type[3])
        super(JacoInterface, self).__init__(robot_type=robot_type, cfg=JacoConfig())
        self.connect_to_robot()
        rospy.loginfo('initiating reset service')
        # instantiate services to be called by dm_wrapper
        self.connect = rospy.Service('/initialize', initialize, self.initialize)
        self.server_reset = rospy.Service('/reset', reset, self.reset)
        self.server_get_state = rospy.Service('/get_state', get_state, self.get_state)
        self.server_home = rospy.Service('/home', home, self.home)
        self.server_step = rospy.Service('/step', step, self.step)
        print('waiting for client initialization')
        self.initialized = False
        rospy.spin()
        

    def initialize(self, cmd):
        self.fence_min_x = cmd.fence_min_x
        self.fence_max_x = cmd.fence_max_x
        self.fence_min_y = cmd.fence_min_y
        self.fence_max_y = cmd.fence_max_y
        self.fence_min_z = cmd.fence_min_z
        self.fence_max_z = cmd.fence_max_z
        self.initialized = True
        rospy.loginfo('initialized --->')
        return True

    def get_state(self, success=True, msg=''):
        """ 
            :msg is not used - this returns state regardless of message passed in (for service calls)
            :success bool to indicate if a cmd was successfully executed
        
            in dm_control for 6dof robot - state is an OrderedDict([
                   'arm_pos' of shape (9,2) -> jaco_joint_1-6 and jaco_joint_finger1-6 
                   'arm_vel' of shape (9,1)
                   'hand_pos is shape (7,)
                   'target_pos' is shape (3,)
        """
        st = self.get_state_trace()
        return success, msg, [], st['n_states'], st['time_offset'], st['joint_pos'], st['joint_vel'], st['joint_effort'], st['tool_pose']

    def step(self, cmd):
        if self.initialized:
            if cmd.type == 'VEL':
                # velocity command for each joint in deg/sec
                # vel command dont have time to actually get results
                # only reset state trace when commanded
                self.reset_state_trace()
                n = int(cmd.data[0])
                cmd_vel_deg = convert_to_degrees(cmd.unit, np.array(cmd.data[1:]))
                for i in range(n):
                    msg, success = self.send_joint_velocity_cmd(cmd_vel_deg)
                    time.sleep(1/100.)
                return self.get_state(success=success, msg=str(msg))
            if cmd.type == 'ANGLE':
                # command joint position angle 
                self.reset_state_trace()
                current_joint_angles_radians = self.get_joint_angles()
                joint_angles_degrees, joint_angles_radians = convert_joint_angles(current_joint_angles_radians, cmd.unit, cmd.relative, cmd.data)
                msg, success = self.send_joint_angle_cmd(joint_angles_degrees)
                success = True
                return self.get_state(success=success, msg=msg)
            elif cmd.type == 'TOOL':
                self.reset_state_trace()
                # command end effector pose in cartesian space
                current_tool_pose = self.get_tool_pose()
                position, orientation_q, orientation_rad, orientation_deg = convert_tool_pose(current_tool_pose, cmd.unit, cmd.relative, cmd.data[:3], cmd.data[3:])
                msg, success = self.send_tool_pose_cmd(position, orientation_q)
                return self.get_state(success=success, msg=msg)
 
            else:
                raise(NotImplemented)
        else:
            return self.get_state(success=False, msg='not initialized')

    def home(self, msg=None):
        print('calling home')
        self.home_robot_service()
        return True
 
    def reset(self, msg=None):
        print('calling reset')
        self.home_robot_service()
        # TODO - reset should take a goto message and use the controller to go to a particular position
        # this function does not return until the arm has reached the home position
        return self.get_state()
    

if __name__ == '__main__':
    jaco = JacoInterface()
    #jaco.connect_to_robot()
    #jaco.reset()
    #print(jaco.get_state_trace())
    #try:
    #    rospy.spin()
    #except KeyBoardInterrupt:
    #    sys.exit()


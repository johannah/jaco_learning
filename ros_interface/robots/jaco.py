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
/j2n7s300_driver/out/joint_commdnd

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

from utils import Quaternion2EulerXYZ, EulerXYZ2Quaternion
#from jaco_control.msg import InteractionParams
import fence
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
#
#    def set_PID(self):
#        # velocity controller gains
#        self.P = np.diag([self.cfg['velocity_kp_gains']['joint_%d'%n] for n in range(self.n_joints)])
#        self.D = np.diag([self.cfg['velocity_kd_gains']['joint_%d'%n] for n in range(self.n_joints)])
#        self.I = 0.0 * np.eye(self.n_joints)
#
class JacoRobot():

    def init_ros(self, robot_type='j2n7s300', cfg=JacoConfig()):
        self.state_lock = threading.Lock()
        self.tool_pose_lock = threading.Lock()

        self.request_timeout_secs = 10
        rospy.loginfo('starting init of ros')
        print('starting init of ros')
        self.robot_type = robot_type
        self.prefix = '/{}'.format(robot_type)
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
        self.path_joint_ang = self.prefix + '_driver/joints_action/joint_angles'
        self.joint_angle_client = actionlib.SimpleActionClient(self.path_joint_ang, 
                                                               ArmJointAnglesAction)
        #self.joint_angle_client.wait_for_server()
        #self.path_position_publisher = '//desired_joint_position'
        #self.desired_joint_position_publisher = rospy.Publisher(self.path_position_publisher, 
        #                                        JointState, queue_size=50)
        self.tool_pose_requester_address = self.prefix+'_driver/pose_action/tool_pose'
        self.tool_pose_requester =  actionlib.SimpleActionClient(self.tool_pose_requester_address, 
                                                                  ArmPoseAction)


        rospy.loginfo("Jaco controller init successful.")

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


    def receive_joint_state(self, robot_joint_state):
        """
        Callback for '/prefix_driver/out/joint_state'.
        :param robot_joint_state: data from topic
        :type robot_joint_state JointState
        :return None
        """
        self.state_lock.acquire()
        # Hacky limit of states
        self.robot_joint_state = copy(robot_joint_state)

        self.state_trace['n_states']+=1
        self.state_trace['time_offset'].append(time.time()-self.state_start)
        self.state_trace['joint_pos'].extend(self.robot_joint_state.position)
        self.state_trace['joint_vel'].extend(self.robot_joint_state.velocity)
        self.state_trace['joint_effort'].extend(self.robot_joint_state.effort)
        robot_tool_pose = self.get_tool_pose()
        tool_pose  = [robot_tool_pose.pose.position.x, robot_tool_pose.pose.position.y, robot_tool_pose.pose.position.z, 
                      robot_tool_pose.pose.orientation.x, robot_tool_pose.pose.orientation.y, 
                      robot_tool_pose.pose.orientation.z, robot_tool_pose.pose.orientation.w]
        self.state_trace['tool_pose'].extend(tool_pose)

        self.state_lock.release()
        self.joint_state_rcvd = True
            
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

    def get_pose(self, unit, relative, position, orientation):
        """
        unit: describes the unit of the command - mq:quaternian, mrad:radians, or mdeg:degrees. If mq, 3 position+4 quaternians data are required, otherwise, 3 position + 3 orientation data are required
        is_relative: bool indicative whether or not the pose command is relative to the current position or absolute
        position: relative or absolute position and orientation values for XYZ 
        """
        # failed: 
        # when sent testup
        #('last', [0.25247281789779663, -0.29990264773368835, 0.5781479477882385])
        #('this', [0.18161669373512268, 0.2771887183189392, 0.7448583841323853])

        assert unit in ['mq', 'mrad', 'mdeg']
        if unit == 'mq':
            assert(len(orientation) == 4); "end effector pose in quaternions requires 3 position & 4 orientation values - received {}".format(len(orientation))
        else:
            assert(len(orientation) == 3); "end effector pose in rad/deg requires 3 position & 3 orientation values - received {}".format(len(orientation))

        self.tool_pose_lock.acquire()
        robot_tool_pose = self.robot_tool_pose 
        self.tool_pose_lock.release()
        last_position = [robot_tool_pose.pose.position.x, robot_tool_pose.pose.position.y, robot_tool_pose.pose.position.z]
        last_orientation_q = [robot_tool_pose.pose.orientation.x, robot_tool_pose.pose.orientation.y, 
                            robot_tool_pose.pose.orientation.z, robot_tool_pose.pose.orientation.w]
 
        if relative:
            # get current orientation
            # note - in the example, they reference /driver/out/cartesian_command - so it is the last command, not the last position as we are doing here
            # last_orientation is in radians
            print("rev relative position", position)
            last_orientation_rad = Quaternion2EulerXYZ(last_orientation_q)
            position = [last_position[i]+position[i] for i in range(3)]
        if unit == 'mq':
            if relative:
                # current orientation is mq
                orientation_rad = [last_orientation_rad[i] + Quaternion2EulerXYZ(orientation)[i] for i in range(3)]
                orientation_q = EulerXYZ2Quaternion(orientation_rad)
            else:
                orientation_q = orientation
            orientation_rad = Quaternion2EulerXYZ(orientation_q)
            orientation_deg = [math.degrees(orientation_rad[i]) for i in range(3)]
        if unit == 'mrad':
            if relative:
                orientation_rad = [last_orientation_rad[i] + orientation[i] for i in range(3)]
            else:
                orientation_rad = orientation
            orientation_q = EulerXYZ2Quaternion(orientation_rad)
            orientation_deg = [math.degrees(orientation_rad[i]) for i in range(3)]
        if unit == 'mdeg':
            if relative:
                orientation_deg = [math.degrees(last_orientation_rad[i]) + orientation[i] for i in range(3)]
            else:
                orientation_deg = orientation
            orientation_rad = [math.radians(orientation_deg[i]) for i in range(3)]
            orientation_q = EulerXYZ2Quaternion(orientation_rad)
        print('last position', last_position)
        print('req position', position)
        return position, orientation_q, orientation_rad, orientation_deg

    def check_target_pose_safety(self, position):
        print('position', position)
        x,y,z = position
        fence_result = ''
        if fence.maxx < x:
            rospy.logwarn('HIT FENCE: maxx of {} is < x of {}'.format(fence.maxx, x))
            x = fence.maxx
            fence_result+='+MAXFENCEX'
        if x < fence.minx:
            rospy.logwarn('HIT FENCE: x of {} < miny {}'.format(x, fence.minx))
            x = fence.minx
            fence_result+='+MINFENCEX'
        if fence.maxy < y:
            rospy.logwarn('HIT FENCE: maxy of {} is < y {}'.format(fence.maxy, y))
            y = fence.maxy
            fence_result+='+MAXFENCEY'
        if y < fence.miny:
            rospy.logwarn('HIT FENCE: y of {} is  miny of {}'.format(y, fence.miny))
            y = fence.miny
            fence_result+='MINFENCEY'
        if fence.maxz < z:
            rospy.logwarn('HIT FENCE: maxz of {} is < z of {}'.format(fence.maxz, z))
            z = fence.maxz
            fence_result+='MAXFENCEZ'
        if z < fence.minz:
            rospy.logwarn('HIT FENCE: z of {} < minz of {}'.format(z, fence.minz))
            z = fence.minz
            fence_result+='MINFENCEZ'
        return [x,y,z], fence_result

    def send_tool_pose_cmd(self, position, orientation_q):
        #robot_tool_pose = self.get_tool_pose()
 
        print("REQUESTING POSE before fence of:", position)
        position, result = self.check_target_pose_safety(position)
        # based on pose_action_client.py
        print("REQUESTING POSE after fence of:", position)
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


    def send_joint_velocity_cmd(self, velocity):
        """
        Creates a joint velocity command with the target velocity for each joint.
        :param velocity: velocity of each joint in deg/s
        :type velocity: np.array
        :return: joint velocity command
        :rtype: JointVelocity
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
        self.joint_velocity_publisher.publish(joint_cmd)
        return 'sent', success

    def create_joint_angle_cmd(self, angle):
        """
        Creates a joint angle command with the target joint angles. 
        :param angle: goal position of the waypoint, angles are in radians
        :type angle: list
        :return: joint angle command
        :rtype: ArmJointAnglesGoal
        """
        # initialize the command
        joint_cmd = ArmJointAnglesGoal()

        joint_cmd.angles.joint1 = self.convert_to_degree(angle[0])
        joint_cmd.angles.joint2 = self.convert_to_degree(angle[1])
        joint_cmd.angles.joint3 = self.convert_to_degree(angle[2])
        joint_cmd.angles.joint4 = self.convert_to_degree(angle[3])
        joint_cmd.angles.joint5 = self.convert_to_degree(angle[4])
        joint_cmd.angles.joint6 = self.convert_to_degree(angle[5])
        if self.n_joints == 6:
            joint_cmd.angles.joint7 = 0.
        else:
            joint_cmd.angles.joint7 = self.convert_to_degree(angle[6])

        return joint_cmd

    def send_joint_angle_cmd(self, joint_cmd):
        """
        Sends the joint angle command to the action server and waits for its execution. Note that the planning is done
        in the robot base.
        :param joint_cmd: joint angle command
        :type joint_cmd: ArmJointAnglesGoal
        :return: None
        """
        self.joint_angle_client.send_goal(joint_cmd)
        self.joint_angle_client.wait_for_result()


    def publish_desired_joint_position(self, q_desired):
        """
        Publishes the desired joint position. Useful for gain tuning or debugging.
        :param q_desired: desired joint position, in a form of [1 x n_joints] 2D array.
        :type: np.array
        :return: None
        """
        msg = JointState()
        msg.header = self.robot_joint_states.header
        msg.name = self.robot_joint_states.name
        msg.position = q_desired.tolist()
        self.desired_joint_position_publisher.publish(msg)


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
        self.reset_state_trace()
        self.empty_state = np.zeros((37))
        rospy.loginfo('initiating reset service')
        # instantiate services to be called by dm_wrapper
        self.server_reset = rospy.Service('/reset', reset, self.reset)
        self.server_get_state = rospy.Service('/get_state', get_state, self.get_state)
        self.server_home = rospy.Service('/home', home, self.home)
        self.server_step = rospy.Service('/step', step, self.step)

    def reset_state_trace(self):
        self.state_lock.acquire()
        self.state_start = time.time()
        self. state_trace = {'n_states':0,
                             'time_offset':[], 
                             'joint_pos':[], 
                             'joint_vel':[], 
                             'joint_effort':[], 
                             'tool_pose':[]
                             }
        self.state_lock.release()

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
        self.state_lock.acquire()
        st = copy(self.state_trace)
        self.state_lock.release()
        return success, msg, [], self.state_trace['n_states'], self.state_trace['time_offset'], self.state_trace['joint_pos'], self.state_trace['joint_vel'], self.state_trace['joint_effort'], self.state_trace['tool_pose']

    def step(self, cmd):
        # TODO - should we not use an invalid command or should
        if cmd.type == 'VEL':
            # velocity command for each joint in deg/sec
            self.reset_state_trace()
            msg, success = self.send_joint_velocity_cmd(cmd.data)
            return self.get_state()
        if cmd.type == 'POSE':
            position, orientation_q, orientation_rad, orientation_deg = self.get_pose(cmd.unit, cmd.relative, cmd.data[:3], cmd.data[3:])
            print("SENDING POSITION", position)
            # need to store all states
            self.reset_state_trace()
            msg, success = self.send_tool_pose_cmd(position, orientation_q)
            print("FINISHED")
            return self.get_state(msg=str(position+orientation_q))
        else:
            raise(NotImplemented)

    def home(self, msg=None):
        print('calling home')
        self.home_robot_service()
        return True
 
    def check_vel_action_safety(self, action):
        # action in deg/second
        if np.abs(action).max() < 200:
            return True
        else:
            return False

    def reset(self, msg=None):
        print('calling reset')
        self.home_robot_service()
        # TODO - reset should take a goto message and use the controller to go to a particular position
        # this function does not return until the arm has reached the home position
        return self.get_state()
    

if __name__ == '__main__':
    jaco = Jaco()
    jaco.connect_to_robot()
    try:
        rospy.spin()
    except KeyBoardInterrupt:
        sys.exit()


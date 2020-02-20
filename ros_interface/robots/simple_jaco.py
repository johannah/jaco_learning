#! /usr/bin/env python

# *******************************************************************
# Modified with permission from :
# Author: Sahand Rezaei-Shoshtari
# Oct. 2019
# Copyright 2019, Sahand Rezaei-Shoshtari, All rights reserved.
# *******************************************************************
# https://github.com/sahandrez/jaco_control

import os
import sys
import numpy as np
import pid

# ROS libs
import rospy
import rospkg
import actionlib
import tf
import tf.transformations
import tf2_ros
import dynamic_reconfigure.server
from base import BaseRobot, BaseConfig

# ROS messages and services
from std_msgs.msg import Float64, Header
from geometry_msgs.msg import Pose, PoseStamped, Twist, TwistStamped, Vector3, Point, Quaternion, Wrench, WrenchStamped
from sensor_msgs.msg import JointState
#from gazebo_msgs.msg import LinkState#, LinkStates
from kinova_msgs.msg import JointVelocity, JointTorque, JointAngles
from kinova_msgs.msg import ArmJointAnglesGoal, ArmJointAnglesAction, SetFingersPositionAction, SetFingersPositionGoal
from kinova_msgs.srv import HomeArm, SetTorqueControlMode, SetTorqueControlParameters

from ros_interface.srv import reset, step, home
# todo - force this to load configuration from file should have safety params
# torque, velocity limits in it

class SimpleJacoRobot(BaseRobot):
    """
    This class handles the control of the robot.
    """

    def __init__(self):
        """
        Initializes the robot class.
        config_instance
        """
        self.prefix = '/j2n7s300'
        rospy.init_node('jaco_controller', anonymous=True)
        self.init_ros()
        # TODO state should be configured
        self.empty_state = np.zeros((37))

    def step(self, action):
        action = self.check_action_safety(action)
        self.create_joint_velocity_cmd(action)
        return self.get_state()

    def reset(self, req):
        print("RX RESET")
        self.home_robot_service()
        # TODO JRH - wait till robot goes home ?
        return True
        #return self.get_state()

    def get_state(self):
        # TODO JRH collate state here as described in mujoco and return
        state = self.empty_state
        return state

    def init_ros(self):
        print('init ros')
        rospy.wait_for_service(self.prefix + '_driver/in/home_arm')
        self.home_robot_service = rospy.ServiceProxy(self.prefix + '_driver/in/home_arm', HomeArm)

        # make default services 
        self.server_reset = rospy.Service('reset', reset, self.reset)
        # FLAGS and STATE DESCRIBERS
        self.active_controller = None
        # SUBSCRIBERS
        # State subscribers
        self.state_subscriber = None
        # Torque subscribers
        self.actual_joint_torque_subscriber = None
        self.compensated_joint_torque_subscriber = None
        # End-effector state subscriber (used on the real robot)
        self.end_effector_state_subscriber = None
        # End-effector wrench
        self.end_effector_wrench_subscriber = None

        # PUBLISHERS
        # Joint velocity command publisher
        self.joint_velocity_publisher = None
        # Joint torque command publisher
        self.joint_torque_publisher = None
        # Desired joint position publisher (useful for gain tuning and debugging)
        self.desired_joint_position_publisher = None
        # Send interaction params to the force interaction node
        self.force_interaction_params_publisher = None
        # TF Buffer and Listener
        self.tf_Buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_Buffer)

        # Controllers
        # TODO put these in from the config file
        #self.velocity_controller = pid.PID(0., 0., 0., self.n_joints)
        #self.P_task, self.I_task, self.D_task = None, None, None

        # Callback data holders
        self.robot_joint_states = JointState()
        #self.end_effector_state = LinkState()
        self.actual_joint_torques = JointAngles()
        self.compensated_joint_torques = JointAngles()
        self.end_effector_wrench = Wrench()

        # JRH not sure if this is going to be needed
        ## PyBullet object
        #self.pybullet_robot = None

        # Directories
        self.rospack = rospkg.RosPack()
        self.rospack_path = self.rospack.get_path('ros_interface')
        self.description_directory = 'description'
        rospy.loginfo("Robot init successful.")
        print('finished init ros')

    def connect_to_robot(self):
        """
        Connects to the robot.
        :return: None
        """
        topics = [name for (name, _) in rospy.get_published_topics()]
        joint_state_topic = self.prefix + '_driver/out/joint_state'
        if joint_state_topic not in topics:
            rospy.logerr("COULD NOT connect to the robot.")
            emsg = 'ERROR: %s not in topics'%joint_state_topic
            rospy.logerr(emsg)
            raise ValueError(emsg)

        self.state_subscriber = rospy.Subscriber(self.prefix + "_driver/out/joint_state", JointState, self.receive_joint_state, queue_size=50)
        #self.end_effector_state_subscriber = rospy.Subscriber(self.prefix + "_driver/out/tool_pose", PoseStamped,
        #                                                      self.receive_end_effector_state, queue_size=50)
        #self.end_effector_wrench_subscriber = rospy.Subscriber(self.prefix + "_driver/out/tool_wrench",
       #                                                        WrenchStamped, self.receive_end_effector_wrench,
       #                                                        queue_size=50)
        rospy.loginfo("Connected to the robot")


    def receive_joint_state(self, robot_joint_state):
        """
        Callback for '/prefix_driver/out/joint_state'.
        :param robot_joint_state: data from topic
        :type robot_joint_state JointState
        :return None
        """
        self.robot_joint_states = robot_joint_state

    #def receive_end_effector_state(self, pose):
    #    """
    #    Callback for '/j2n6s300_driver/out/tool_pose'. Sets the pose of the end effector of the real robot. This method
    #    does not compute the twist.
    #    :param pose: end-effector pose
    #    :type pose: PoseStamped
    #    :return: None
    #    """
    #    self.end_effector_state.link_name = self.prefix[1:] + '_end_effector'
    #    self.end_effector_state.pose = pose
    #    self.end_effector_state.reference_frame = 'world'

    def home_robot(self):
        """
        Homes the robot by calling the home robot service
        :return: None
        """
        # call the homing service
        self.home_robot_service()

    def shutdown_controller():
        """
        Shuts down the controller.
        :return: None
        """
        rospy.loginfo("Shutting Down Controller.")
        rospy.signal_shutdown('Done')
        return exit(0)

if __name__ == '__main__':
    simple_jaco = SimpleJacoRobot() 
    simple_jaco.connect_to_robot()
    try:
        rospy.spin()
    except KeyBoardInterrupt:
        sys.exit()
    

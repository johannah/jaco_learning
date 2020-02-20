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

        ## TF Buffer and Listener
        #self.tf_Buffer = tf2_ros.Buffer()
        #self.tf_listener = tf2_ros.TransformListener(self.tf_Buffer)

        # Controllers
        # TODO put these in from the config file
        #self.velocity_controller = pid.PID(0., 0., 0., self.n_joints)
        #self.P_task, self.I_task, self.D_task = None, None, None

        # Callback data holders
        self.robot_joint_state = JointState()

        #self.end_effector_state = LinkState()
        #self.actual_joint_torques = JointAngles()
        #self.compensated_joint_torques = JointAngles()
        #self.end_effector_wrench = Wrench()

        # Directories
        #self.rospack = rospkg.RosPack()
        #self.rospack_path = self.rospack.get_path('jaco_control')
        #self.description_directory = 'description'

 
#        # init service clients
#        self.joint_angle_client = actionlib.SimpleActionClient(self.prefix + '_driver/joints_action/joint_angles', ArmJointAnglesAction)
#        self.joint_angle_client.wait_for_server()
#        self.gripper_client = actionlib.SimpleActionClient(self.prefix + '_driver/fingers_action/finger_positions', SetFingersPositionAction)
#        self.gripper_client.wait_for_server()

        # init publishers
        # Joint velocity command publisher - send commands to the kinova driver
#        self.joint_velocity_publisher = rospy.Publisher(self.prefix + '_driver/in/joint_velocity', JointVelocity, queue_size=50)
        # Desired joint position publisher (useful for gain tuning and debugging)
        #self.desired_joint_position_publisher = rospy.Publisher('/jaco_control/desired_joint_position', JointState, queue_size=50)
#
          # init the controllers
        #self.init_velocity_controller()
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

         #joint_torque_path = self.prefix + "_driver/out/actual_joint_torques" 
#        self.actual_joint_torque_subscriber = rospy.Subscriber(joint_torque_path,
#                                                               JointAngles, self.receive_actual_joint_torque,
#                                                               queue_size=50)
#        self.compensated_joint_torque_subscriber = rospy.Subscriber(self.prefix + "_driver/out/compensated_joint_torques",
#                                                                    JointAngles, self.receive_compensated_joint_torque,
#                                                                    queue_size=50)
#        self.end_effector_state_subscriber = rospy.Subscriber(self.prefix + "_driver/out/tool_pose", PoseStamped,
#                                                              self.receive_end_effector_state, queue_size=50)
#        self.end_effector_wrench_subscriber = rospy.Subscriber(self.prefix + "_driver/out/tool_wrench",
#                                                               WrenchStamped, self.receive_end_effector_wrench,
#                                                               queue_size=50)
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

#    def receive_end_effector_state(self, pose):
#        """
#        Callback for '/j2n6s300_driver/out/tool_pose'. Sets the pose of the end effector of the real robot. This method
#        does not compute the twist.
#        :param pose: end-effector pose
#        :type pose: PoseStamped
#        :return: None
#        """
#        self.end_effector_state.link_name = self.prefix[1:] + '_end_effector'
#        self.end_effector_state.pose = pose
#        self.end_effector_state.reference_frame = 'world'
#
#    def receive_end_effector_wrench(self, msg):
#        """
#        Callback for '/j2n6s300_driver/out/end/tool_wrench'.
#        :param msg: end-effector wrench
#        :return: None
#        """
#        self.end_effector_wrench = msg.wrench
#
#    def receive_actual_joint_torque(self, actual_trq):
#        """
#        Callback for '/prefix_driver/out/actual_joint_torques'.
#        :param actual_trq: data from the topic
#        :type actual_trq: JointAngles
#        :return: None
#        """
#        self.actual_joint_torques = actual_trq
#
#    def receive_compensated_joint_torque(self, compensated_trq):
#        """
#        Callback for '/prefix_driver/out/compensated_joint_torques'.
#        :param compensated_trq: data from the topic
#        :return: None
#        """
#        self.compensated_joint_torques = compensated_trq
#
#    def init_velocity_controller(self):
#        """
#        Initializes the velocity controller.
#        :return: None
#        """
#        # TODO JRH - shouldnt this read from our config?
#        p_gain = 5.0
#        i_gain = 0.0
#        d_gain = 1.0
#        P = p_gain * np.eye(self.n_joints)
#        I = i_gain * np.eye(self.n_joints)
#        D = d_gain * np.eye(self.n_joints)
#        self.velocity_controller = pid.PID(P, I, D, self.n_joints)
#
#    def velocity_control(self, traj, sleep_time=10.):
#        """
#        NOTE: Only works on the real robot.
#        Controls the robot through the waypoints with velocity controller. Note that this actually controls the joint
#        position but through velocity commands. The velocity commands are sent to low level controllers on the robot.
#        :param traj: trajectory generated by the Planner class
#        :type traj: trajectory.Trajectory
#        :param sleep_time: time to wait for the robot to reach the starting position of the trajectory
#        :type sleep_time: float or int
#        :return: None
#        """
#
#        # the publish rate MUST be 100 Hz (Kinova API documentation)
#        publish_rate = rospy.Rate(100)
#
#        # send the robot to starting position
#        rospy.loginfo("Sending robot to the starting position.")
#        print(traj.start_pos)
#        self.set_joint_angle(traj.start_pos)
#        rospy.sleep(sleep_time)
#
#        # tracking time
#        start_time = rospy.get_time()
#        elapsed_time = 0.0
#
#        rospy.loginfo("Starting velocity controller.")
#        while elapsed_time < rospy.Duration(traj.total_t).to_sec():
#            # get the index of the next waypoint
#            elapsed_time = rospy.get_time() - start_time
#            index = traj.get_next_waypoint(elapsed_time)
#            if index >= len(traj.waypoints):
#                break
#
#            # compute the error term and update the PID controller (also deals with angle wraparound problem)
#            pos = np.array(self.robot_joint_states.position[0:self.n_joints])
#            error = self.wrap_to_pi(pos) - self.wrap_to_pi(traj.waypoints[index][:])
#            error = error.reshape((-1, 1))
#
#            # only the diagonal elements of the control command matrix is required
#            cmd = -np.diag(self.velocity_controller.update_PID(error))
#
#            # send the joint velocity command to the robot
#            joint_command = self.create_joint_velocity_cmd(cmd)
#            self.send_joint_velocity_cmd(joint_command)
#
#            # publish desired joint position
#            self.publish_desired_joint_position(traj.waypoints[index][:])
#
#            # maintain the publish rate of 100 Hz
#            publish_rate.sleep()
#
#    def set_joint_angle(self, joint_angles):
#        """
#        Set the joint positions on the real robot. Planning is done in the robot base.
#        :param joint_angles: desired joint positions
#        :type joint_angles: list or np.array
#        :return: None
#        """
#        # create the joint command
#        joint_command = self.create_joint_angle_cmd(joint_angles)
#
#        # send the joint command to the real robot
#        self.send_joint_angle_cmd(joint_command)
#
#    def set_finger_position(self, finger_positions):
#        """
#        Sets the finger positions; the values are in percentage: Fully closed is 100 and fully open is 0.
#        :param finger_positions: list of the finger positions
#        :type finger_positions: list
#        :return: None
#        """
#        # convert percentage to thread turn
#        finger_turns = [x/100.0 * self.MAX_FINGER_TURNS for x in finger_positions]
#        turns_temp = [max(0.0, x) for x in finger_turns]
#        finger_turns = [min(x, self.MAX_FINGER_TURNS) for x in turns_temp]
#
#        # create and send the goal message
#        goal = SetFingersPositionGoal()
#        goal.fingers.finger1 = finger_turns[0]
#        goal.fingers.finger2 = finger_turns[1]
#        goal.fingers.finger3 = finger_turns[2]
#
#        self.gripper_client.send_goal(goal)
#
#        if self.gripper_client.wait_for_server(rospy.Duration(5)):
#            return self.gripper_client.get_result()
#        else:
#            self.gripper_client.cancel_all_goals()
#            rospy.logwarn("The gripper action time-out.")
#            return None
#

    def create_joint_angle_cmd(self, angle):
        """
        Creates a joint angle command with the target joint angles. Planning is done in the base of the robot.
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
        joint_cmd.angles.joint7 = 0.0
        if self.n_joints == 7:
            joint_cmd.angles.joint7 = self.convert_to_degree(angle[6])
        return joint_cmd

    def create_joint_velocity_cmd(self, velocity):
        """
        Creates a joint velocity command with the target velocity for each joint.
        :param velocity: velocity of each joint in radians/s
        :type velocity: np.array
        :return: joint velocity command
        :rtype: JointVelocity
        """
        # init
        velocity = velocity.reshape(-1)
        joint_cmd = JointVelocity()

        joint_cmd.joint1 = self.convert_to_degree(velocity[0])
        joint_cmd.joint2 = self.convert_to_degree(velocity[1])
        joint_cmd.joint3 = self.convert_to_degree(velocity[2])
        joint_cmd.joint4 = self.convert_to_degree(velocity[3])
        joint_cmd.joint5 = self.convert_to_degree(velocity[4])
        joint_cmd.joint6 = self.convert_to_degree(velocity[5])
        joint_cmd.joint7 = 0.0
        if self.n_joints == 7:
            joint_cmd.joint7 = self.convert_to_degree(velocity[6])
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

    def send_joint_velocity_cmd(self, joint_cmd):
        """
        Publishes the joint velocity command to the robot.
        :param joint_cmd: desired joint velocities
        :type joint_cmd: JointVelocity
        :return: None
        """
        self.joint_velocity_publisher.publish(joint_cmd)

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
        self.init_ros()
        self.empty_state = np.zeros((37))
        rospy.loginfo('initiating reset service')
        # instantiate services to be called by dm_wrapper
        self.server_reset = rospy.Service('/reset', reset, self.reset)
        self.server_get_state = rospy.Service('/get_state', get_state, self.get_state)
        self.server_home = rospy.Service('/home', home, self.home)


    def get_state(self, msg=None):
        """ in dm_control for 6dof robot - state is an OrderedDict([
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
        return True, name, joint_pos, joint_vel, joint_effort

    def step(self, cmd=None):
        action = self.check_action_safety(action)
        self.create_joint_velocity_cmd(action)
        return self.get_state()

    def home(self, msg=None):
        print('calling home')
        self.home_robot_service()
 
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


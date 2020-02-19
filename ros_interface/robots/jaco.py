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
#import pybullet
import pid
import trajectory

# ROS libs
import rospy
import rospkg
import actionlib
import tf
import tf.transformations
import tf2_ros
import dynamic_reconfigure.server
from jaco_control.cfg import controller_gainsConfig
from base import BaseRobot, BaseConfig

# ROS messages and services
from std_msgs.msg import Float64, Header
from geometry_msgs.msg import Pose, PoseStamped, Twist, TwistStamped, Vector3, Point, Quaternion, Wrench, WrenchStamped
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import LinkState#, LinkStates
from kinova_msgs.msg import JointVelocity, JointTorque, JointAngles
from kinova_msgs.msg import ArmJointAnglesGoal, ArmJointAnglesAction, SetFingersPositionAction, SetFingersPositionGoal
from kinova_msgs.srv import HomeArm, SetTorqueControlMode, SetTorqueControlParameters
from jaco_control.msg import InteractionParams

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

class JacoRobot(BaseRobot):
    """
    This class handles the control of the robot.
    """

    def __init__(self, config):
        """
        Initializes the robot class.
        config_instance
        """
        rospy.init_node('jaco_controller', anonymous=True)
        self.init_ros()
        self.connect_to_robot()
        # TODO state should be configured
        self.empty_state = np.zeros((37))

    def step(self, action):
        action = self.check_action_safety(action)
        self.create_joint_velocity_cmd(action)
        return self.get_state()

    def reset(self):
        self.home_robot_service()
        # TODO JRH - wait till robot goes home ?
        return self.get_state()

    def get_state(self):
        # TODO JRH collate state here as described in mujoco and return
        state = self.empty_state
        return state

    def init_ros(self):
        rospy.wait_for_service(self.prefix + '_driver/in/home_arm')
        self.home_robot_service = rospy.ServiceProxy(self.prefix + '_driver/in/home_arm', HomeArm)

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
        self.velocity_controller = pid.PID(0., 0., 0., self.n_joints)
        #self.P_task, self.I_task, self.D_task = None, None, None

        # Callback data holders
        self.robot_joint_states = JointState()
        self.end_effector_state = LinkState()
        self.actual_joint_torques = JointAngles()
        self.compensated_joint_torques = JointAngles()
        self.end_effector_wrench = Wrench()

        # JRH not sure if this is going to be needed
        ## PyBullet object
        #self.pybullet_robot = None

        # Directories
        self.rospack = rospkg.RosPack()
        self.rospack_path = self.rospack.get_path('jaco_control')
        self.description_directory = 'description'
        rospy.loginfo("Robot init successful.")

    def init_controller(self):
        """
        Initializes all the publishers and services to control the robot.
        :return: None
        """
        # init service clients
        self.joint_angle_client = actionlib.SimpleActionClient(self.prefix + '_driver/joints_action/joint_angles', ArmJointAnglesAction)
        self.joint_angle_client.wait_for_server()
        self.gripper_client = actionlib.SimpleActionClient(self.prefix + '_driver/fingers_action/finger_positions', SetFingersPositionAction)
        self.gripper_client.wait_for_server()

        # init publishers
        self.joint_velocity_publisher = rospy.Publisher(self.prefix + '_driver/in/joint_velocity', JointVelocity, queue_size=50)
        self.joint_torque_publisher = rospy.Publisher(self.prefix + '_driver/in/joint_torque', JointTorque, queue_size=50)
        self.desired_joint_position_publisher = rospy.Publisher('/jaco_control/desired_joint_position', JointState, queue_size=50)

        # init services
        rospy.wait_for_service(self.prefix + '_driver/in/set_torque_control_mode')
        self.set_torque_control_mode_service = rospy.ServiceProxy(self.prefix + '_driver/in/set_torque_control_mode', SetTorqueControlMode)

        rospy.wait_for_service(self.prefix + '_driver/in/set_torque_control_parameters')
        self.set_torque_control_parameters_service = rospy.ServiceProxy(self.prefix + '_driver/in/set_torque_control_parameters', SetTorqueControlParameters)

        rospy.wait_for_service(self.prefix + '_driver/in/home_arm')
        self.home_robot_service = rospy.ServiceProxy(self.prefix + '_driver/in/home_arm', HomeArm)

        # init the controllers
        self.init_velocity_controller()
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

        self.state_subscriber = rospy.Subscriber(self.prefix + "_driver/out/joint_state", JointState, self.receive_joint_state, queue_size=50)
        self.actual_joint_torque_subscriber = rospy.Subscriber(self.prefix + "_driver/out/actual_joint_torques",
                                                               JointAngles, self.receive_actual_joint_torque,
                                                               queue_size=50)
        self.compensated_joint_torque_subscriber = rospy.Subscriber(self.prefix + "_driver/out/compensated_joint_torques",
                                                                    JointAngles, self.receive_compensated_joint_torque,
                                                                    queue_size=50)
        self.end_effector_state_subscriber = rospy.Subscriber(self.prefix + "_driver/out/tool_pose", PoseStamped,
                                                              self.receive_end_effector_state, queue_size=50)
        self.end_effector_wrench_subscriber = rospy.Subscriber(self.prefix + "_driver/out/tool_wrench",
                                                               WrenchStamped, self.receive_end_effector_wrench,
                                                               queue_size=50)
        rospy.loginfo("Connected to the robot")


    def receive_joint_state(self, robot_joint_state):
        """
        Callback for '/prefix_driver/out/joint_state'.
        :param robot_joint_state: data from topic
        :type robot_joint_state JointState
        :return None
        """
        self.robot_joint_states = robot_joint_state

    def receive_end_effector_state(self, pose):
        """
        Callback for '/j2n6s300_driver/out/tool_pose'. Sets the pose of the end effector of the real robot. This method
        does not compute the twist.
        :param pose: end-effector pose
        :type pose: PoseStamped
        :return: None
        """
        self.end_effector_state.link_name = self.prefix[1:] + '_end_effector'
        self.end_effector_state.pose = pose
        self.end_effector_state.reference_frame = 'world'

    def receive_end_effector_wrench(self, msg):
        """
        Callback for '/j2n6s300_driver/out/end/tool_wrench'.
        :param msg: end-effector wrench
        :return: None
        """
        self.end_effector_wrench = msg.wrench

    def receive_actual_joint_torque(self, actual_trq):
        """
        Callback for '/prefix_driver/out/actual_joint_torques'.
        :param actual_trq: data from the topic
        :type actual_trq: JointAngles
        :return: None
        """
        self.actual_joint_torques = actual_trq

    def receive_compensated_joint_torque(self, compensated_trq):
        """
        Callback for '/prefix_driver/out/compensated_joint_torques'.
        :param compensated_trq: data from the topic
        :return: None
        """
        self.compensated_joint_torques = compensated_trq

    def set_active_controller(self, controller):
        """
        Sets the active controller
        :param controller: a string containing the name of the controller ('velocity', 'fftorque', 'impedance')
        :type controller: str
        :return: None
        """
        self.active_controller = controller

    def init_velocity_controller(self):
        """
        Initializes the velocity controller.
        :return: None
        """
        # TODO JRH - shouldnt this read from our config?
        p_gain = 5.0
        i_gain = 0.0
        d_gain = 1.0
        P = p_gain * np.eye(self.n_joints)
        I = i_gain * np.eye(self.n_joints)
        D = d_gain * np.eye(self.n_joints)
        self.velocity_controller = pid.PID(P, I, D, self.n_joints)

    def velocity_control(self, traj, sleep_time=10.):
        """
        NOTE: Only works on the real robot.
        Controls the robot through the waypoints with velocity controller. Note that this actually controls the joint
        position but through velocity commands. The velocity commands are sent to low level controllers on the robot.
        :param traj: trajectory generated by the Planner class
        :type traj: trajectory.Trajectory
        :param sleep_time: time to wait for the robot to reach the starting position of the trajectory
        :type sleep_time: float or int
        :return: None
        """
        self.set_active_controller('velocity')

        # the publish rate MUST be 100 Hz (Kinova API documentation)
        publish_rate = rospy.Rate(100)

        # send the robot to starting position
        rospy.loginfo("Sending robot to the starting position.")
        print(traj.start_pos)
        self.set_joint_angle(traj.start_pos)
        rospy.sleep(sleep_time)

        # tracking time
        start_time = rospy.get_time()
        elapsed_time = 0.0

        rospy.loginfo("Starting velocity controller.")
        while elapsed_time < rospy.Duration(traj.total_t).to_sec():
            # get the index of the next waypoint
            elapsed_time = rospy.get_time() - start_time
            index = traj.get_next_waypoint(elapsed_time)
            if index >= len(traj.waypoints):
                break

            # compute the error term and update the PID controller (also deals with angle wraparound problem)
            pos = np.array(self.robot_joint_states.position[0:self.n_joints])
            error = self.wrap_to_pi(pos) - self.wrap_to_pi(traj.waypoints[index][:])
            error = error.reshape((-1, 1))

            # only the diagonal elements of the control command matrix is required
            cmd = -np.diag(self.velocity_controller.update_PID(error))

            # send the joint velocity command to the robot
            joint_command = self.create_joint_velocity_cmd(cmd)
            self.send_joint_velocity_cmd(joint_command)

            # publish desired joint position
            self.publish_desired_joint_position(traj.waypoints[index][:])

            # maintain the publish rate of 100 Hz
            publish_rate.sleep()

    def set_joint_angle(self, joint_angles):
        """
        Set the joint positions on the real robot. Planning is done in the robot base.
        :param joint_angles: desired joint positions
        :type joint_angles: list or np.array
        :return: None
        """
        # create the joint command
        joint_command = self.create_joint_angle_cmd(joint_angles)

        # send the joint command to the real robot
        self.send_joint_angle_cmd(joint_command)

    def set_finger_position(self, finger_positions):
        """
        Sets the finger positions; the values are in percentage: Fully closed is 100 and fully open is 0.
        :param finger_positions: list of the finger positions
        :type finger_positions: list
        :return: None
        """
        # convert percentage to thread turn
        finger_turns = [x/100.0 * self.MAX_FINGER_TURNS for x in finger_positions]
        turns_temp = [max(0.0, x) for x in finger_turns]
        finger_turns = [min(x, self.MAX_FINGER_TURNS) for x in turns_temp]

        # create and send the goal message
        goal = SetFingersPositionGoal()
        goal.fingers.finger1 = finger_turns[0]
        goal.fingers.finger2 = finger_turns[1]
        goal.fingers.finger3 = finger_turns[2]

        self.gripper_client.send_goal(goal)

        if self.gripper_client.wait_for_server(rospy.Duration(5)):
            return self.gripper_client.get_result()
        else:
            self.gripper_client.cancel_all_goals()
            rospy.logwarn("The gripper action time-out.")
            return None

    def home_robot(self):
        """
        Homes the robot by calling the home robot service
        :return: None
        """
        # call the homing service
        self.home_robot_service()

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

    def publish_interaction_params(self, traj):
        """
        Publishes the interaction parameters to '/jaco_control/force_interaction' topic.
        :param traj: the trajectory that is being run (it also contains information abot interaction params)
        :type traj: trajectory.Trajectory
        :return: None
        """
        center = Point(x=traj.rod_center[0], y=traj.rod_center[1], z=traj.rod_center[2])
        radius = traj.rod_radius
        cut_force_k = traj.cut_force_k
        cut_force_d = traj.cut_force_d
        direction = Vector3(x=traj.cut_direction[0], y=traj.cut_direction[1], z=traj.cut_direction[2])
        plane = Vector3(x=traj.cut_plane[0], y=traj.cut_plane[1], z=traj.cut_plane[2])

        msg = InteractionParams(center=center, radius=radius, cut_force_k=cut_force_k, cut_force_d=cut_force_d,
                                direction=direction, plane=plane)

        self.force_interaction_params_publisher.publish(msg)

    def publish_end_effector_pose(self):
        """
        Publishes the end-effector pose. Useful for Gazebo.
        :return: None
        """
        header = Header(stamp=rospy.Time.now(), frame_id='world')
        pose = Pose(position=self.end_effector_state.pose.position, orientation=self.end_effector_state.pose.orientation)
        msg = PoseStamped(header=header, pose=pose)
        self.end_effector_pose_publisher.publish(msg)

    def publish_end_effector_twist(self):
        """
        Publishes the end-effector twist. Useful for Gazebo.
        :return: None
        """
        header = Header(stamp=rospy.Time.now(), frame_id='world')
        twist = Twist(linear=self.end_effector_state.twist.linear, angular=self.end_effector_state.twist.angular)
        msg = TwistStamped(header=header, twist=twist)

        self.end_effector_twist_publisher.publish(msg)

    def shutdown_controller():
        """
        Shuts down the controller.
        :return: None
        """
        rospy.loginfo("Shutting Down Controller.")
        rospy.signal_shutdown('Done')
        return exit(0)



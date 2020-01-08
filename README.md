# jaco_learning
 
From sahand:

Running the Code on Jaco 2 Robot
Bring up the robot:
roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=j2n6s300

Launch MoveIt! and RViz if you want to use MoveIt! for planning:
roslaunch j2n6s300_moveit_config j2n6s300_jaco_lfd.launch

To use the software E-stop in torque control mode, run the following node. Use ctrl-C to stop the motion whenever the robot was approaching a dangerous position. Shutting down this node switches the robot from torque control mode to position control mode:
rosrun kinova_driver kinova_emergency_stop

# Tasks

build com protocal between dm_control and ros

## I think this is for the 6dof jaco arm
state_dim = env.observation_spec()['observations'].shape[0]
action_dim = env.action_spec().shape[0]
min_action = float(env.action_spec().minimum[0])
max_action = float(env.action_spec().maximum[0])
action_shape = env.action_spec().shape

state return in mujoco - 

---------------
after reset:

state_type, reward, discount, state = eval_env.reset()

state_type is :  <StepType.FIRST: 0>

reward, discount are None

state is: 
collections.OrderedDict['observations'].shape -> (37,)
collections.OrderedDict['observations'].dtype -> np.float64
max() is 1, min() -1 -1

action was  of shape (6,) float32

-------------
during steps

step_type, reward, discount, state = eval_env.step(action)
step_type is:  <StepType.MID: 1>
done is bool
discount was 1.0
reward was 0.0 - (obv this will change)
observations has same params as above


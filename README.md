# ros_interface

Convert dm_control commands to ros for taking actions and returning state on real robots. 

## Exploring the Jaco 6DOF arm
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


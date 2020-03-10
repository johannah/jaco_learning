# ros_interface

Convert dm_control commands to ros for taking actions and returning state on real robots. 

Standard functions for rl control (such as step() and reset()) are implemented here. They can be called via 
TCP connection and will interface with the appropriate ros services for a particular robot. 

Jaco components of this repo are based off of [sahandres's jaco_control repo](https://github.com/sahandrez/jaco_control/blob/master/jaco_control/utils/robot.py)

TODO: 
- handle lab ports  
- forward images   
- test reconnect cases  
- export screen from docker image  
- ensure that ros log messages are same as mujoco  
- integrate moveit (so far have only used kinova_demo pose_action_client



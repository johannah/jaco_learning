# ros_interface

Convert dm_control commands to ros for taking actions and returning state on real robots. 

Standard functions for rl control (such as step() and reset()) are implemented here. They can be called via 
TCP connection and will interface with the appropriate ros services for a particular robot. 

# Running Jaco Experiments
The physical arm is attached via a usb to a machine running jaco_docker. The robot_server and robot must be run from this machine. Currently, this is done by within our jaco_docker to resolve dependencies easily. 

After creating jaco_robot using the instructions on github.com/johannah/jaco_docker, you must launch or attach to a running container. 

If running a new docker instance:

`docker run -it --name jaco_robot --privileged -v /dev/bus/usb:/dev/bus/       usb  jaco_control`

To resume a recent docker container:   
`docker ps`  
`docker container start <container_id>`  

To attach to a running session:   
`docker exec -it jaco_robot /bin/bash`

1) Ensure usb connection from robot to computer running docker. 

2) Launch kinova drivers. 
`roscd ros_interface  
sh launch/kinova_launch.sh
`
Robot should open hand. 

3) 


### Position Info

From Kinova: The origin of the robot is at the intersection point of the bottom plane of the base and cylinder center line.
+x axis is directing to the left when facing the base panel (where power switch and cable socket locate).
+y axis is towards to user when facing the base panel.
+z axis is upwards when robot is standing on a flat surface.

Position of the hand can be found at /j2n7s300_driver/out/tool_pose

Current joint angle in degrees /j2n7s300_driver/out/joint_commdnd

This position seems to be a good starting place for working in the air:

joint1: 270.736328125
joint2: 190.922332764
joint3: 0.156117007136
joint4: 63.7591400146
joint5: 359.618255615
joint6: 213.480560303
joint7: 179.727416992

Desk working limits facing robot: 

left down:
  position:
    x: -0.168833673
    y: -0.468905031681
    z: 0.100024700165
  orientation:
    x: 0.722196280956
    y: -0.0425987318158
    z: 0.0991511568427
    w: 0.683218061924

right up:
pose:
  position:
    x: 0.415617853403
    y: -0.347220003605
    z: 0.844722270966
  orientation:
    x: 0.611751139164
    y: 0.380540639162
    z: 0.475444734097
    w: 0.504877865314

right down:
pose:
  position:
    x: 0.412587165833
    y: -0.341769188643
    z: 0.0996225997806
  orientation:
    x: 0.610928654671
    y: 0.381375432014
    z: 0.474779188633
    w: 0.505869209766

forward:
pose:
  position:
    x: 0.508611798286
    y: -0.710586607456
    z: 0.139719769359
  orientation:
    x: 0.712905526161
    y: 0.131039366126
    z: 0.254024744034
    w: 0.640363872051
### To run rl experiment: 

`sh launch/kinova_launch.sh`  
`python ros_interface/robots/jaco.py`  
`python ros_interface/interfaces/robot_server.py`  

Send tcp cmds and recieve observations as in robot_client.py


# Comments
Jaco components of this repo are based off of [sahandres's jaco_control repo](https://github.com/sahandrez/jaco_control/blob/master/jaco_control/utils/robot.py)

TODO: 
- handle lab ports  
- forward images   
- test reconnect cases  
- export screen from docker image  
- ensure that ros log messages are same as mujoco  
- integrate moveit (so far have only used kinova_demo pose_action_client



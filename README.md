# ros_interface

The purpose of this library is to provide a method of converting standard rl state update functions into ros commands. Experiments have been performed on the Jaco2 robot & dm_control, though the interface was designed to be general enough for use in other robots or rl environments. 

A TCP connection exposes init, step, reset, get_state, home to the robot through interfaces/robot_server.py. 

TODO describe input/output of these functions. 
TODO describe fence
TODO describe kinds of steps

# Running Jaco Experiments

The physical arm is attached via a usb to a machine running jaco_docker. The robot_server and robot must be run from this machine. 

After creating jaco_robot using the instructions on github.com/johannah/jaco_docker, you must launch or attach to a running container. 

1) Ensure usb connection from robot to computer running docker. 

2) Start docker:

If running a new docker instance:

`docker run -it --name jaco_robot --privileged -v /dev/bus/usb:/dev/bus/usb  jaco_control`

To resume a recent docker container:   
`docker ps`  
`docker container start <container_id>`  

To attach to a running session:   
`docker exec -it jaco_robot /bin/bash`

2) Turn on robot and use remote to set home (light should turn blue on robot). Launch kinova drivers. 
`roscd ros_interface  
sh launch/kinova_launch.sh
`
Robot should open hand. 

3) Run jaco.py

4) run robot_server.py

---

5) After experiments. Return robot to home position, then power off.


### Position Info

The origin of the robot is at the intersection point of the bottom plane of the base and cylinder center line.
+x axis is directing to the left when facing the base panel (where power switch and cable socket are located).
+y axis is towards to user when facing the base panel.
+z axis is upwards when robot is standing on a flat surface.

Position of the hand can be found at /j2n7s300_driver/out/tool_pose
Current joint angle in degrees /j2n7s300_driver/out/joint_commdnd

# Comments
Most Jaco components of this repo are based off of [sahandres's jaco_control repo](https://github.com/sahandrez/jaco_control/blob/master/jaco_control/utils/robot.py)

TODO: 
- forward images   
- test reconnect cases  



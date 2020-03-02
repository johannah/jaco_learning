# ros_interface

Convert dm_control commands to ros for taking actions and returning state on real robots. 

Standard functions for rl control (such as step() and reset()) are implemented here. They can be called via 
TCP connection and will interface with the appropriate ros services for a particular robot. 

## Repo Tour:
-- ros_interface   
   -- robots    
     -- jaco.py    
   -- interfaces  
     -- robot_server.py   
     -- robot_client.py   

Robot client serves as an example of how to communicate via standard RL commands to the robot server. To run rl experiment: 

`sh launch/kinova_launch.sh`  
`python ros_interface/robots/jaco.py`  
`python ros_interface/interfaces/robot_server.py`  

Send tcp cmds and recieve observations as in robot_client.py

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

# Comments
Jaco components of this repo are based off of [sahandres's jaco_control repo](https://github.com/sahandrez/jaco_control/blob/master/jaco_control/utils/robot.py)


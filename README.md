# ros_interface

The purpose of this library is to provide a method of converting standard rl state update functions into ros commands. Experiments have been performed on the Jaco2 7DOF robot & dm_control, though the interface was designed to be general enough for use in other robots or rl environments. 

A TCP connection exposes init, step, reset, get_state, home to the robot through interfaces/robot_server.py. 

The physical arm is attached via a usb to a machine running jaco_docker. The robot_server and robot must be run from this machine. 

# Running Jaco Experiments

1) After creating jaco_robot_net docker using the instructions on github.com/johannah/jaco_docker, you must launch or attach to a running container. 

    - Ensure usb connection from robot to computer running docker. 

    - Start docker:

      - If running a new docker instance:
        - `docker run -it --name jaco_robot --privileged -v /dev/bus/usb:/dev/bus/usb  jaco_control`
      
      - To resume a recent docker container:   
        - `docker ps`  
        - `docker container start <container_id>`  
      
      - To attach to a running session:   
        - `docker exec -it jaco_robot /bin/bash`

3) Turn on robot and use remote to set home (light should turn blue on robot). Launch kinova drivers. 

    - `roslaunch jaco_remote.launch`
  
    - Robot should open hand. 

4) Running remote experiments.

    - Remote experiments can be run by declaring `physics_type='robot'` in the `task_kwargs` of a `dm_control` env from our [dm_control](http://github.com/johannah/dm_control). Data is sent via tcp through port 9030 by default. 
  
    - Be sure to configure the `fence` in `task_kwargs` to be within the bounds of your workspace. For instance, a `fence={'x':(-.5,.5), 'y':(-1,.4), 'z':(0.05, 3)}` will limit the robot to within .5 meter from side to side, allow a reach of 1m to the front (towards kinova label), and only allow the robot to reach above the base in z. 
  
    - Each step command returns the robot position, velocity, and effort for each joint as well as the tool pose.  
  
    - See [jaco_rl](http://github.com/johannah/jaco_rl) for an example of an rl agent trained to work with this repo.

---

5) After experiments. Return robot to sleep position with the remote (press home until the remote returns to home, then press home again to move it to the sleep position), then power off.


### Position Info

The origin of the robot is at the intersection point of the bottom plane of the base and cylinder center line.
+x axis is directing to the left when facing the base panel (where power switch and cable socket are located).
+y axis is towards to user when facing the base panel.
+z axis is upwards when robot is standing on a flat surface.

Position of the hand can be found at /j2n7s300_driver/out/tool_pose
Current joint angle in degrees /j2n7s300_driver/out/joint_commdnd

# Comments
Most Jaco components of this repo are based off of [sahandres's jaco_control repo](https://github.com/sahandrez/jaco_control/blob/master/jaco_control/utils/robot.py)


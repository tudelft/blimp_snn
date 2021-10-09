# Connecting to multiple machines + OptiTrack

In this document, the procedure will be illustrated with an example connecting the computer (master, server) and the Raspberry Pi (remote).

## How to connect to multiple machines with ROS?

1. Modify the */etc/hosts* file on all machines, adding the lines:
```
    <IP-of-master> mycomp
    <IP-of-pi>     mypi
```

2. Modify the *~/.ssh/config* file on the master machine (computer), adding the lines
```
Host mypi
        Hostname 192.168.0.100
        User pi
```

3. Modify the *~/.bashrc* file on all machines, adding lines:
    * In the computer:
        ```
        export ROS_MASTER_URI=http://mycomp:11311
        export ROS_IP=mycomp
        ```

    * In the pi:
        ```
        export ROS_MASTER_URI=http://mycomp:11311
        export ROS_IP=mypi
        ```

4. Create a `pi_ros_env.sh` file in the `catkin_ws` of the pi, like this:
```
#!/bin/bash

export ROS_IP=mypi
export ROS_MASTER_URI=http://mycomp:11311
#export ROSLAUNCH_SSH_UNKNOWN=1

source /home/pi/Pi_Git/ros_radar_mine/catkin_ws/devel/setup.bash

exec "$@"
```

5. Give execution permissions for this `pi_ros_env.sh` file:
```
chmod +x pi_ros_env.sh
```

6. Start the `ROS launch file` adding the following lines:
```
<machine name="pi_client" timeout="20" address="mypi" user="pi" password="raspberry"  env-loader="/home/pi/Pi_Git/ros_radar_mine/catkin_ws/pi_ros_env.sh"/>
<machine name="comp_server" address="mycomp"/>
```

7. On each `<node>` on the `ROS launch file`, specify on which machine it will be executed:
```
<node machine="pi_client" ... >
            or
<node machine="comp_server" ... >
```

## How to connect to OptiTrack with ROS?

### Procedure:

1. Git clone the `mocap_optitrack` package in your workspace (`catkin_ws/src`):
```
git clone https://github.com/h2r/mocap_optitrack
```

2. Connect to the *OptiTrack - Motive* computer through an ethernet cable/connection

3. Run `ifconfig` + enable *multicast* on your computer:
```
ifconfig <name_of_ethernet_connection> multicast
```

4. Add the following `launch` file in the `mocap_optitrack/launch` package directory:
```
<launch> 
  <node machine="comp_server"
    pkg="mocap_optitrack" 
	type="pose_stamper_1" 
	name="pose_stamper_1" />
  
  <node machine="comp_server"
    pkg="mocap_optitrack" 
	type="mocap_node" 
	name="mocap_node"
	respawn="false"
	launch-prefix=""
	required="true">

    <rosparam file="$(find mocap_optitrack)/config/mocap.yaml" command="load" />
  </node>
</launch>
```

5. Generate a *rigid body* in *Motive* and five it a name

6. Modify the `config/mocap.yaml` file in the `mocap_optitrack` with the proper rigid body name

7. Add the following line to the main `launch` file on the master computer:
```
<include file="$(find mocap_optitrack)/launch/<new_launch_file_name>.launch" if="$(arg record_optitrack)"/>
```

### Debugging problems:

One common error could be that the connection with OptiTrack is not properly configured.

* To check that it is properly working: 
    1. Follow all the previous steps. If not getting data:
    2. Download *NatNet SDK*
    3. Carefully follow the `ReadMe.txt` instructions within the `sample/SampleClient` folder
    4. On step 8, remember to press key `[1]` or the corresponding one that appears on the screen

* Another approach:
    1. Launch the `mocap_optitrack` node
    2. Do a `rostopic echo /<rigid_body>/pose` and check that the messages are properly being published
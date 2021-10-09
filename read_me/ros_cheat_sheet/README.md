# ROS little summary

## Getting started
### Packages

`rospack list`: list of all installed ROS packages\
`rospack find package-name`: find the directory of a single package\
`roscd package-name`: move to package directory\
`rosls package-name`: inspecting a package

Important features: **tab completion**
Executables and include files from packages installed by *apt-get* are stored in: `/opt/ros/melodic`

### Nodes

`rosrun package-name executable-name`: command to execute one node\
`rosrun package-name executable-name __name:=node-name`: explicitly set the node's name (overwriting its default name)\
`rosnode list`: list of running nodes at a particular time\
`rosnode info node-name`: information about a node (topics to which it is publisher/subscriber + connections to other nodes)\
`rosnode kill node-name`: to kill a node\
`rosnode cleanup`: remove dead node from the list

**Note1:** Every node should have a different name.\
**Note2:** Node names are not necessarily the same as the names of the exectuables underlying those nodes.

### Topics and messages

`rqt_graph`: visualize the publish-subscribe relationships between ROS nodes\
`rostopic list`: to get a list of active topics\
`rostopic echo topic-name`: see actual messages that are being published on a single topic (example: rostopic echo /turtle1/cmd_vel)\
`rostopic hz/bw topic-name`: measuring publication rates\
`rostopic info topic-name`: inspecting a topic\
`rosmsg show message-type-name`: inspecting a message type\
`rostopic pub -r rate-in-hz topic-name message-type message-content`: publishing messages from the command line (example: rostopic pub -r 1 /turtle1/cmd_vel\geometry_msgs/Twist '[2,0,0]' '[0,0,0]')

Understanding message types: `package-name/type-name` (example: turtlesim/Color)

`roswtf`: checking for problems

### Writing ROS programs
```
mkdir -p ~/catkin_ws/src       # creating a workspace
cd ~/catkin_ws/src             # moving to the src folder
catkin_create_pkg package-name # creating a package
cd package-name
mkdir src
mkdir include

Tree structure:

catkin_ws
└── src
    └── package-name
        ├── CMakeLists.txt
        ├── package.xml
        ├── src
        └── include

```

TO **COMPILE**:

Declaring dependencies:
```
find_package(catkin REQUIRED COMPONENTS package-names) [CMakeLists.txt]
<build_depend>package-name</build_depend>  [package.xml]
<run_depend>package-name</run_depend>  [package.xml]
```

Declaring an executable:
```
add_executable(executable-name source-files)
target_link_libraries(executable-name ${catkin_LIBRARIES})
```

Uncomment:
```
add_executable(executable-name src/cpp-name.cpp) # This will create the executable at the devel/lib directory in the workspace
add_dependencies(hello ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hello
  ${catkin_LIBRARIES}
)
include_directories(
include
${catkin_INCLUDE_DIRS}
)
```
Building the workspace:
```
cd catkin_ws
catkin_make # Always compile from the workspace directory
source devel/setup.bash
```

### Publisher/Subscriber tips
```
ros::Publisher pub = nh.advertise<message-type>("topic-name",queue_size);
```
**Note1:** To publish messages on multiple topics from the same node, it is necessary to create separate ros::Publisher objects for each topic
```
void function-name(const package-name::type-name &msg){
    ...
}

ros::Subscriber sub = node-handle.subscribe(topic-name, queue-size, pointer-to-callback-function);
```

Giving control to ROS:
```
ros::spin();          # Only use this if you only work with a subscriber
OR
while(ros::ok()){     # Better use this normally (if you work with publishers and subscribers simultaneously)
    ros::spinOnce();
}
```

### Log messages
```
ROS_DEBUG_STREAM_ONCE(message);
ROS_INFO_STREAM_ONCE(message);
ROS_WARN_STREAM_ONCE(message);
ROS_ERROR_STREAM_ONCE(message);
ROS_FATAL_STREAM_ONCE(message);

rqt_console

· To see logs:
∼ /.ros/log/run_id/rosout.log

setting /run_id to run_id
rosparam get /run_id
rosclean check
rosclean purge

· To set the logger level:
rosservice call /node-name/set_logger_level ros.package-name level
rqt_logger_level
```

### Graph resource names

* Global names: start with `/` and make sense anywhere they are used
* Relative names: lackes leading slash `/` (page 89)
    * Resolving relative names: /default-namespace + relative-name => global-name
* Setting the default namespace (which is tracked individually for each node, rather than being a system-wide setting): `ns` in launch files.
* Private names: node-name + private-name => global-name (often used for parameters, never for topics)

### Launch files
To start several nodes at once -> define a .launch file (within the launch directory)\
Nodes from launch files are initialized roughly at the same time
```
roslaunch package-name launch-file-name
roslaunch -v package-name launch-file-name # Verbose option
<launch>
<node
pkg="package-name"
type="executable-name"
name="node-name"            # Relative name for a node
respawn="true"              # Restart a node when it terminates
required="true"             # Close evertything when a specific node terminates
launch−prefix="xterm −e"    # Launch a node in its own window
output="screen"             # Directing output to the console
ns="namespace"              # Launching node inside a certain namespace (INTERESTING! SEE PAGE 101 and 102)
<remap from="original-name" to="new-name" /> # Page 104
<include file="path-to-launch-file" />
<include file="$(find package-name)/launch-file-name" />
/>
<node pkg=" ... " type=" ... " name=" ... "></node>
</launch>
```
There are also `arguments` (see page 111)
There are also `groups` (see page 112) [To push several nodes into the same workspace, for example, or to conditionally enable or disable nodes]

### Parameters

Parameters must be actively queried by the ndes that are interested in their values.\
Therefore, they are most suitable for configuration information that will not change (much) over time.
```
rosparam list                                # List of all existing parameters
rosparam get parameter_name                  # Retrieve the value of a certain parameter
rosparam get namespace                       # Retrieve the value of all parameters inside a namespace
rosparam get /                               # Retrieve all parameters at once
rosparam set parameter_name parameter_value  # Assign a value to a parameter
rosservice call /clear
```
Accessing parameters from C++:
```
void ros::param::set(parameter_name, input_value);  # The input value for a set can be a std::string, a bool, an int, or a double
bool ros::param::get(parameter_name, output_value); # The output value for get should be a variable (which is passed by reference) of one of those types
```
See page 121 and example set_bg_color.cpp and pubvel_with_max.cpp for more details on parameters from C++


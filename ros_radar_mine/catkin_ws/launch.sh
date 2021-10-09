roslaunch motor_control radar.launch & 
sleep 3s 
roslaunch motor_control controller.launch & 
#roslaunch motor_control optitrack.launch & 
sleep 5s 
xterm -e roslaunch motor_control plotjuggler.launch & 
sleep 2s 
roslaunch motor_control record.launch
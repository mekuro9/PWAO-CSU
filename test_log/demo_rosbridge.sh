#!/bin/bash

# Source ROS and run ROS package in a new terminal
gnome-terminal -- /bin/bash -c 'source /opt/ros/humble/setup.bash; 
cd /home/moro/humble_ws;
colcon build --packages-select sequence_controller;
source install/local_setup.bash;
echo "Launching ROS Package..."; 
script -c "ros2 run sequence_controller latency_test" /home/moro/test_log/rosbridge/reliable_3_pub_phone.txt;
exec /bin/bash'

# Launch Flask Server in a new terminal
gnome-terminal -- /bin/bash -c 'cd /home/moro/ros2web/flask-app; 
echo "Starting Flask Server..."; 
flask --app latency_test run --host 0.0.0.0; 
exec /bin/bash'

# Source ROS and run ROSBridge server in a new terminal
gnome-terminal -- /bin/bash -c 'source /opt/ros/humble/setup.bash; 
echo "Starting ROSBridge Server..."; 
ros2 launch rosbridge_server rosbridge_websocket_launch.xml; 
exec /bin/bash'


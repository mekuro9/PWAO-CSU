#!/bin/bash

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

# Source ROS and run ROS package in a new terminal
gnome-terminal -- /bin/bash -c 'source /opt/ros/humble/setup.bash; 
cd /home/moro/humble_ws;
colcon build --packages-select obstacle_detection;
source install/local_setup.bash;
script -c "ros2 run obstacle_detection obstacle_detector" /home/moro/test_log/lidar/test2.txt;
echo "Launching ROS Package..."; 
exec /bin/bash'

# Source ROS and run ROS package in a new terminal
gnome-terminal -- /bin/bash -c 'sudo chmod 777 /dev/ttyUSB0;
source /opt/ros/humble/setup.bash; 
cd /home/moro/humble_ws;
source install/local_setup.bash;
ros2 launch sllidar_ros2 view_sllidar_s2_launch.py;
echo "Launching Lidar Package..."; 
exec /bin/bash'



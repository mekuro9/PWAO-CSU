#!/bin/bash

# Source ROS and run ROS package in a new terminal
gnome-terminal -- /bin/bash -c 'source /opt/ros/humble/setup.bash; 
cd /home/cafelatte/humble_ws;
colcon build --packages-select test_package;
source install/local_setup.bash;
echo "Launching ROS Package..."; 
ros2 run test_package is_test;
exec /bin/bash'

# Launch Flask Server in a new terminal
gnome-terminal -- /bin/bash -c 'cd /home/cafelatte/ros2web/flask-app; 
echo "Starting Flask Server..."; 
flask --app latency_test run --host 0.0.0.0; 
exec /bin/bash'

# Source ROS and run integration serive server in a new terminal
gnome-terminal -- /bin/bash -c 'source /opt/ros/humble/setup.bash;
cd /home/cafelatte/is-workspace;
source install/local_setup.bash
echo "Starting integration service Server..."; 
integration-service src/Integration-Service/wheelchair_test/is_test1_be.yaml; 
exec /bin/bash'


When you install micro-ros-arduino library from github https://github.com/micro-ROS/micro_ros_arduino/tree/humble (for humble version) and add it to arduino libraries, the example code does not compile for teensy 4.0
The solution to a similar issue was with a different board was found here: https://github.com/micro-ROS/micro_ros_arduino/issues/315 and here: https://github.com/micro-ROS/micro_ros_arduino/issues/1114

However as it is not for Teensy, the following steps were taken to solve the issue


1. Download the latest version of the micro-ros-arduino library(for humble) from the release section: https://github.com/micro-ROS/micro_ros_arduino/releases
2. Include it in your project using ``` <Sketch -> Include library -> Add .ZIP Library...> ```
3. Since we are not using docker but native Ubuntu 22.04 installed ROS 2 Humble, we will skip the micro-ros Agent docker command


4. Open a terminal in your home 
5. Source the ROS 2 installation
   ```source /opt/ros/humble/setup.bash```
6. Create a workspace and download the micro-ROS tools (also mentioned in https://micro.ros.org/docs/tutorials/core/teensy_with_arduino/)
```
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
# Update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y
```

7 Build micro-ROS tools and source them
```
colcon build
source install/local_setup.bash
```

8. Clone the micro_ros_arduino repository in the microros_ws/src folder
9. Following the steps of https://micro.ros.org/docs/tutorials/advanced/create_custom_static_library/, create firmware workspace by:
```
#Open a new terminal in home
source /opt/ros/humble/setup.bash
cd microros_ws
source install/local_setup.bash
ros2 run micro_ros_setup create_firmware_ws.sh generate_lib

```
10. Next edit the teensy4_toolchain.cmake file in /home/[your username]/microros_ws/src/micro_ros_arduino-humble/extras/library_generation
```
SET(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_CROSSCOMPILING 1)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

#set(CMAKE_C_COMPILER $ENV{TOOLCHAIN_PREFIX}gcc) # This line has been commented out
#set(CMAKE_CXX_COMPILER $ENV{TOOLCHAIN_PREFIX}g++) # This line has been commented out

# The next two lines are included instead
set(CMAKE_C_COMPILER /usr/bin/arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER /usr/bin/arm-none-eabi-g++) 

SET(CMAKE_C_COMPILER_WORKS 1 CACHE INTERNAL "")
SET(CMAKE_CXX_COMPILER_WORKS 1 CACHE INTERNAL "")

set(FLAGS "-O2 -mfloat-abi=hard -mfpu=fpv5-d16 -ffunction-sections -fdata-sections -fno-exceptions -nostdlib -mcpu=cortex-m7 -mthumb -D'RCUTILS_LOG_MIN_SEVERITY=RCUTILS_LOG_MIN_SEVERITY_NONE'" CACHE STRING "" FORCE)

set(CMAKE_C_FLAGS_INIT "-std=c11 ${FLAGS} -DCLOCK_MONOTONIC=0 -D'__attribute__(x)='" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_INIT "-std=c++14 ${FLAGS} -fno-rtti -DCLOCK_MONOTONIC=0 -D'__attribute__(x)='" CACHE STRING "" FORCE)

set(__BIG_ENDIAN__ 0)

```
11. Open another terminal in home and install the gcc-arm-none-eabi 
```
sudo apt-get update
sudo apt-get -y install gcc-arm-none-eabi

```
12. In the terminal you opened in step 9 (if closed then open one and source ros2 and local_setup ) run the following command (make sure you are in microros_ws)
```
ros2 run micro_ros_setup build_firmware.sh /home/[your username]/microros_ws/src/micro_ros_arduino-humble/extras/library_generation/teensy4_toolchain.cmake /home/[your username]/microros_ws/src/micro_ros_arduino-humble/extras/library_generation/colcon.meta

```
13. Now go to the folder ```/home/microros_ws/firmware/build``` and copy the libmicroros.a file
14. Go to the Arduino micro_ros_arduino libary which you added in step 1 (most likely in /home/Arduino/libraries/)
15. Go to ```/home/moro/Arduino/libraries/micro_ros_arduino/src/imxrt1062/fpv5-d16-hard``` and replace the libmicroros.a file with the one copied earlier from ```/home/microros_ws/firmware/build```

Now when you run the example code(for e.g micro-ros_subscriber) from micro-ros library on Arduino IDE (selecting Teensy 4.0 board) and compile, it should work!

This should also work for Teensy4.1

Here: Arduino IDE 2.2.1 is used along with Teensyduino 1.58.1 

When you install micro-ros-arduino library from github https://github.com/micro-ROS/micro_ros_arduino/tree/humble (for humble version) and add it to arduino libraries, the example code does not compile for teensy 4.0
The solution to a similar issue was with a different board was found here: https://github.com/micro-ROS/micro_ros_arduino/issues/315 and here: https://github.com/micro-ROS/micro_ros_arduino/issues/1114

However as it is not for Teensy, the following steps were taken to solve the issue


1. Download the latest version of the micro-ros-arduino library(for humble) from the release section: https://github.com/micro-ROS/micro_ros_arduino/releases
2. Include it in your project using ``` <Sketch -> Include library -> Add .ZIP Library...> ```
3. Since we are not using docker but native Ubuntu 22.04 installed ROS 2 Humble, we will skip the micro-ros Agent docker command and go to the section on patch for teensy
4. Go to your arduino + teensy path and open terminal, which is actually a hidden folder in your home .arduino15. /home/[yourusername]/.arduino15/packages/teensy/hardware/avr/1.58.1 (You can cd to this path or open terminal at this location) then run the following command
```
curl https://raw.githubusercontent.com/micro-ROS/micro_ros_arduino/main/extras/patching_boards/platform_teensy.txt > platform.txt

```
The contents of the platform.txt file are
```
# https://www.pjrc.com/teensy/td_download.html
# https://arduino.github.io/arduino-cli/latest/platform-specification/
# https://arduino.github.io/arduino-cli/0.30/platform-specification/

name=Teensyduino
version=1.8.5
rewriting=disabled

# Teensyduino Installer
#compiler.path={runtime.hardware.path}/../tools/
#teensytools.path={runtime.hardware.path}/../tools/

# Arduino Boards Manager
compiler.path={runtime.tools.teensy-compile.path}/
teensytools.path={runtime.tools.teensy-tools.path}/


## Debugger - Experimental
#debug.executable={build.path}/{build.project_name}.elf
#debug.toolchain=gcc
#debug.toolchain.path={compiler.path}{build.toolchain}
#debug.toolchain.prefix=arm-none-eabi-
#debug.server=openocd
#debug.server.openocd.path={teensytools.path}tdebug
#debug.server.openocd.scripts_dir=unused
#debug.server.openocd.script=toolspath:{teensytools.path}:buildpath:{build.path}:name:{build.project_name}:board:{build.board}

## EEPROM Data
compiler.objcopy.eep.flags=-O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0
compiler.elf2hex.flags=-O ihex -R .eeprom
compiler.libraries.ldflags=

## Preprocessor Includes
recipe.preproc.includes="{compiler.path}{build.toolchain}{build.command.g++}" -M -MG -MP -x c++ -w {build.flags.cpp} {build.flags.cpu} {build.flags.defs} -DARDUINO={runtime.ide.version} -DARDUINO_{build.board} -DF_CPU={build.fcpu} -D{build.usbtype} -DLAYOUT_{build.keylayout} {includes} "{source_file}"

## Preprocessor Macros
recipe.preproc.macros="{compiler.path}{build.toolchain}{build.command.g++}" -E -CC -x c++ -w {compiler.cpp.flags} {build.flags.common} {build.flags.cpp} {build.flags.cpu} {build.flags.defs} -DARDUINO={runtime.ide.version} -DARDUINO_{build.board} -DF_CPU={build.fcpu} -D{build.usbtype} -DLAYOUT_{build.keylayout} {includes} "{source_file}" -o "{preprocessed_file_path}"

## New Preprocessor for Arduino 1.9
tools.arduino-preprocessor.path={runtime.tools.arduino-preprocessor.path}
tools.arduino-preprocessor.cmd.path={path}/arduino-preprocessor
tools.arduino-preprocessor.pattern="{cmd.path}" "{source_file}" "{codecomplete}" -- -std=gnu++14

## Precompile Arduino.h header
recipe.hooks.sketch.prebuild.1.pattern="{teensytools.path}precompile_helper" "{runtime.platform.path}/cores/{build.core}" "{build.path}" "{compiler.path}{build.toolchain}{build.command.g++}" -x c++-header {build.flags.optimize} {build.flags.common} {build.flags.dep} {build.flags.cpp} {build.flags.cpu} {build.flags.defs} -DARDUINO={runtime.ide.version} -DARDUINO_{build.board} -DF_CPU={build.fcpu} -D{build.usbtype} -DLAYOUT_{build.keylayout} "-I{runtime.platform.path}/cores/{build.core}" "{build.path}/pch/Arduino.h" -o "{build.path}/pch/Arduino.h.gch"

## Compile c++ files
recipe.cpp.o.pattern="{compiler.path}{build.toolchain}{build.command.g++}" -c {build.flags.optimize} {build.flags.common} {build.flags.dep} {build.flags.cpp} {build.flags.cpu} {build.flags.defs} -DARDUINO={runtime.ide.version} -DARDUINO_{build.board} -DF_CPU={build.fcpu} -D{build.usbtype} -DLAYOUT_{build.keylayout} "-I{build.path}/pch" {includes} "{source_file}" -o "{object_file}"

## Compile c files
recipe.c.o.pattern="{compiler.path}{build.toolchain}{build.command.gcc}" -c {build.flags.optimize} {build.flags.common} {build.flags.dep} {build.flags.c} {build.flags.cpu} {build.flags.defs} -DARDUINO={runtime.ide.version} -DARDUINO_{build.board} -DF_CPU={build.fcpu} -D{build.usbtype} -DLAYOUT_{build.keylayout} {includes} "{source_file}" -o "{object_file}"

## Compile S files
recipe.S.o.pattern="{compiler.path}{build.toolchain}{build.command.gcc}" -c {build.flags.optimize} {build.flags.common} {build.flags.dep} {build.flags.S} {build.flags.cpu} {build.flags.defs} -DARDUINO={runtime.ide.version} -DARDUINO_{build.board} -DF_CPU={build.fcpu} -D{build.usbtype} -DLAYOUT_{build.keylayout} {includes} "{source_file}" -o "{object_file}"

## Create archives
recipe.ar.pattern="{compiler.path}{build.toolchain}{build.command.ar}" rcs "{archive_file_path}" "{object_file}"

## Link
recipe.c.combine.pattern="{compiler.path}{build.toolchain}{build.command.linker}" {build.flags.optimize} {build.flags.ld} {build.flags.ldspecs} {build.flags.cpu} -o "{build.path}/{build.project_name}.elf" {object_files} {compiler.libraries.ldflags} "{build.path}/{archive_file}" "-L{build.path}" {build.flags.libs}

## Patch ELF - TODO: not supported by modern Arduino...  :(
recipe.elfpatch.pattern="{teensytools.path}/{build.elfpatch}" -mmcu={build.mcu} "{build.path}/{build.project_name}.elf" "{sketch_path}/disk"

## Create eeprom
recipe.objcopy.eep.pattern="{compiler.path}{build.toolchain}{build.command.objcopy}" {compiler.objcopy.eep.flags} "{build.path}/{build.project_name}.elf" "{build.path}/{build.project_name}.eep"

## Create hex
recipe.objcopy.hex.pattern="{compiler.path}{build.toolchain}{build.command.objcopy}" {compiler.elf2hex.flags} "{build.path}/{build.project_name}.elf" "{build.path}/{build.project_name}.hex"

## EHEX file - for Teensy 4.x secure mode
recipe.hooks.objcopy.postobjcopy.1.pattern="{teensytools.path}teensy_secure" encrypthex {build.board} "{build.path}/{build.project_name}.hex"


## Post Build - inform Teensy Loader of new file
recipe.hooks.postbuild.1.pattern="{teensytools.path}teensy_post_compile" "-file={build.project_name}" "-path={build.path}" "-tools={teensytools.path}" "-board={build.board}"
recipe.hooks.postbuild.2.pattern="{teensytools.path}stdout_redirect" "{build.path}/{build.project_name}.sym" "{compiler.path}{build.toolchain}{build.command.objdump}" -t -C "{build.path}/{build.project_name}.elf"
recipe.hooks.postbuild.3.pattern="{teensytools.path}teensy_size" "{build.path}/{build.project_name}.elf"
#
# objdump to create .lst file is VERY SLOW for huge files
# https://forum.pjrc.com/threads/68121?p=288306&viewfull=1#post288306
#
recipe.hooks.postbuild.4.pattern="{teensytools.path}stdout_redirect" "{build.path}/{build.project_name}.lst" "{compiler.path}{build.toolchain}{build.command.objdump}" -d -S -C "{build.path}/{build.project_name}.elf"


## Compute size
recipe.size.pattern="{compiler.path}{build.toolchain}{build.command.size}" -A "{build.path}/{build.project_name}.elf"
recipe.size.regex=^(?:\.text|\.text\.progmem|\.text\.itcm|\.data|\.text\.csf)\s+([0-9]+).*
recipe.size.regex.data=^(?:\.usbdescriptortable|\.dmabuffers|\.usbbuffers|\.data|\.bss|\.noinit|\.text\.itcm|\.text\.itcm\.padding)\s+([0-9]+).*
recipe.size.regex.eeprom=^(?:\.eeprom)\s+([0-9]+).*

## Teensy Ports Discovery
##  Arduino 1.8.9 requires pathPrefs patch
##  discovery patters have only limited support for substitution macros,
##  so we can not use {teensytools.path} or {compiler.path} here

# Teensyduino Installer
#discovery.teensy.pattern="{runtime.hardware.path}/../tools/teensy_ports" -J2

# Arduino Boards Manager
discovery.teensy.pattern="{runtime.tools.teensy-tools.path}/teensy_ports" -J2
pluggable_discovery.required=teensy:teensy-discovery
pluggable_monitor.required.teensy=teensy:teensy-monitor


## Teensy Loader

# Teensyduino Installer
#tools.teensyloader.cmd.path={runtime.hardware.path}/../tools

# Arduino Boards Manager
tools.teensyloader.cmd.path={runtime.tools.teensy-tools.path}

tools.teensyloader.upload.params.quiet=
tools.teensyloader.upload.params.verbose=-verbose
tools.teensyloader.upload.pattern="{cmd.path}/teensy_post_compile" "-file={build.project_name}" "-path={build.path}" "-tools={cmd.path}" "-board={build.board}" -reboot "-port={serial.port}" "-portlabel={serial.port.label}" "-portprotocol={serial.port.protocol}"




## Export hex
recipe.output.tmp_file={build.project_name}.hex
recipe.output.save_file={build.project_name}.{build.board}.hex
recipe.hooks.savehex.postsavehex.1.pattern="{teensytools.path}teensy_secure" encrypthex {build.board} "{sketch_path}/{build.project_name}.{build.board}.hex"

# TODO: missing patch in 1.6.6...
recipe.output.tmp_file2={build.project_name}.elf
recipe.output.save_file2={build.project_name}.elf


# documentation on this file's format
# https://arduino.github.io/arduino-cli/latest/platform-specification/
```

5. Open a terminal in your home 
6. Source the ROS 2 installation
   ```source /opt/ros/humble/setup.bash```
7. Create a workspace and download the micro-ROS tools (also mentioned in https://micro.ros.org/docs/tutorials/core/teensy_with_arduino/)
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

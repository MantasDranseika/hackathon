Matom.ai Hackaton 08 22

Task 5
Create a program that turns on the fan when temperature reaches the limit, use microROS to read temperature measurements and control the fan. Use an emulator. Note: the main program makes decisions while microROS should only be used for reading data and sending control signals.
To enable screen reader support, press Ctrl+Alt+Z To learn about keyboard shortcuts, press Ctrl+slash

requirements microROS and Zephyr RTOS
installation instructions
https://micro.ros.org/docs/tutorials/core/zephyr_emulator/
fan_control package must be placed in fimware/zephyr_apps/apps 

to configure firmware before build:
ros2 run micro_ros_setup configure_firmware.sh fan_control --transport udp --ip 192.168.0.106 --port 8888
build fimware:
ros2 run micro_ros_setup build_firmware.sh
create agent:
ros2 run micro_ros_setup create_agent_ws.sh
build agent:
ros2 run micro_ros_setup build_agent.sh
source build directory:
. install/setup.bash
launch transport agent:
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

in new terminal 
source build directory:
. install/setup.bash
launch Zephyr RTOS and microROS:
ros2 run micro_ros_setup flash_firmware.sh

in directory containing fan_brain source build directory:
. install/setup.bash
launch ros2 program:
ros2 run fan_brain fan_brain 


implementation consists of 2 modules fan_control and fan_brain where fan_control is microROS package 
runing on Zephyr RTOS emulator on local host machine with simulated connection via UDP 8888 port 
using micro_ros_agent.
fan_control posts integers as simulated temperature readings on topic zephyr_fan_control and 
subscribes to topic fan_brain in order to simulate command received from computing unit to execute 
fan on and off commands (prints out 1 or 0 when signal received) 

fan_brain running ROS2 subscribes to zephyr_fan_control to get simulated temperature readings
once temperature reaches set value (in this case 20) it will publish signal 1 to fan_brain topic, 
once temperature will be lower than set value (20) it will publish signal 0 to fan_brain topic.



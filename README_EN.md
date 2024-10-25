
Language：[English](README_EN.md) / [简体中文](README.md)

## Project Description:
This project adopts the method of controlling the P10 motor from Benmo Technology through ROS2; it implements the use of the ROS2_Control framework to control the motor hardware HAL layer, achieving better robot motor control effects, and includes tutorials for learning and communication. You can visit the GitHub link below to learn or refer to this tutorial for hands-on practice.
**GitHub Address**: https://github.com/DDTRobot/p10-ros2-node
## Related Configuration:
### Hardware:
- Category: Model
  - USB2CAN: MKS CANable V2.0
  - Joint Motor: DDT P10 48V
### Software Environment:
- Main Environment: Version
  - Ubuntu: 20.04
  - ROS2: foxy
  - ros2_control: foxy
### Project Introduction:
- Utilize the ros2_control framework https://github.com/ros-controls/ros2_control/tree/foxy to implement the Hardware Abstraction Layer (HAL).
- Utilize Linux socketcan to implement CAN signal transmission and reception.
## Installation:
### ros2_control
```
sudo apt-get install ros-foxy-ros2-control
sudo apt-get install ros-foxy-ros2-controller
```
### Source Code:
```
cd ~
mkdir ddt2_hw
cd ddt2_hw
git clone git@github.com:L-SY/ddt2_hw.git ~/ddt2_hw/src
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja
```
### Related Dependencies:
```
sudo apt-get install ros-foxy-yaml-cpp-vendor
sudo apt-get install ros-foxy-plotjuggler*cd ~
mkdir ddt2_hw
cd ddt2_hw
git clone git@github.com:L-SY/ddt2_hw.git ~/ddt2_hw/src
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja
sudo apt-get install ros-foxy-rqt*
```
## Source Code Introduction:
- ddt2_assets: Stores robot (motor) related configurations, such as controller configurations, urdf, launch.
  - config: Controller configurations under the ros2_control framework.
  - description:
    - ros2_control: The <ros2_control>-tag xacro tag.
    - urdf: The urdf.xacro file.
  - launch: Uses launch.py to load related ROS nodes.
- ddt2_controller: Stores custom ros2_control controllers, provided to ros2_control in the form of plugins.
- ddt2_hw: Implements the underlying CAN data transmission and reception.
- ddt2_msgs: Used to publish motor-related information in ddt2_controller.
## Usage:
### Initialize MKS CANable V2.0 settings

```
ls /dev | grep ttyACM                         Check the device created by the USB2CAN tool, for example, ttyACM0
sudo slcand -o -c -s8 /dev/ttyACMx can0       Note that the baud rate should be 1M to adapt to the motor (-s8), replace x with the local machine
sudo ifconfig can0 up
```
### Start the Controller:
```
source ~ddt2_ws/install/local_setup.bash
ros2 launch ddt2_assets load_hw.launch.py
```
**If normal, the following topics will be generated:**
- /actuator_states
- /ddt2_controller/commands
- /dynamic_joint_states
- /gains/joint1/pid_state
- /joint_states
- /parameter_events
- /robot_description
- /rosout
- /tf
- /tf_static
## Control the Motor:
Publish commands to `/ddt2_controller/commands` to control (unit is circles, relative to the position zero point), for example:
` ros2 topic pub /ddt2_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.5]}"`

If you need to reach a specified position at a specified speed, you need to send the maximum speed limit (unit is RPM, the maximum value is 100) before sending the above control command, for example:
`ros2 topic pub /ddt2_position_controller/velocity_limits std_msgs/msg/Float64MultiArray "{data: [3]}"`

### Motor Information Visualization:
>Use plotjuggler to view ROS2 topic information
```
ros2 run plotjuggler plotjuggler 1
```
- `/joint_states`: Motor information's effort, position, velocity.
- `/actuator_states`: General motor information plus raw information from the motor CAN signal.
![ddt2_hw_plotjuggler.png](docs%2Fddt2_hw_plotjuggler.png)

### Motor Parameter Modification:
The default motor CAN is connected to can0, and the CAN id is 51. If you need to modify it,
then modify it in the actuator.yaml of ddt2_assets.
![ddt2_hw_config.png](docs%2Fddt2_hw_config.png)
### Motor Zero Point Modification:
1. When the motor is powered on and disabled (disabled by default after power-on), manually turn the motor to the desired zero point position.
2. Read the current position
```
cansend can0 035#0400000000000000 1
```
You will receive the following feedback on can0:
```
can0 071 [8] 16 F2 FF FF FF FF FF FF 1
```
The first 4 bits of the data bits (16F2) are the motor position.
```
It is recommended to use candump can0 | grep 071 to get this information.
```
3. Modify the pos_offset in the ninth line of ddt2_assets/one_actuator/config/actuator_coefficient.yaml to the value obtained in step 2 (note: do not include spaces).
![PositionOffset.png](docs%2FPositionOffset.png)

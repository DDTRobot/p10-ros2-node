## DDT2_HW

Language：[English](README_EN.md) / [简体中文](README.md)

#### 相关配置

##### 硬件

| 类别      | 型号               |
|---------|------------------|
| USB2CAN | MKS CANable V2.0 |
| 关节电机    | DDT P10 48V      |

##### 软件环境

| 主要环境         | 版本    |
|--------------|-------|
| Ubuntu       | 20.04 |
| ROS2         | foxy  |
| ros2_control | foxy  |

#### 项目介绍

- 利用ros2_control框架https://github.com/ros-controls/ros2_control/tree/foxy 实现硬件抽象层HAL。
- 利用Linux socketcan实现can信号的收发

#### 安装

##### ros2_control

```
sudo apt-get install ros-foxy-ros2-control
sudo apt-get install ros-foxy-ros2-controller
```

##### 源码

```
cd ~
mkdir ddt2_hw
cd ddt2_hw
git clone git@github.com:L-SY/ddt2_hw.git ~/ddt2_hw/src
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja
```

##### 相关依赖

```
sudo apt-get install ros-foxy-yaml-cpp-vendor
sudo apt-get install ros-foxy-plotjuggler*
sudo apt-get install ros-foxy-rqt*
```

#### 源码介绍

- ddt2_assets：存放机器人（电机）相关配置，如控制器配置，urdf，launch
    - config：ros2_control框架下controller的配置
    - description：
        - ros2_control：urdf的<ros2_control>-tag xacro标签
        - urdf：urdf.xacro文件
    - launch：用launch.py来加载相关ros节点
- ddt2_controller：存放自定义的ros2_control的控制器，通过plugins以插件的形式给ros2_control使用
- ddt2_hw：实现底层的can数据的收发
- ddt2_msgs：用于在ddt2_controller中发布电机的相关信息

#### 使用

##### 初始化MKS CANable V2.0的设置

```
ls /dev | grep ttyACM                         查看USB2CAN工具生成的设备,例如ttyACM0
sudo slcand -o -c -s8 /dev/ttyACMx can0       注意波特率要1M适配电机（-s8),换ttyACM中x为本机
sudo ifconfig can0 up
```

##### 开启控制器

```c
source ~ddt2_ws/install/local_setup.bash
ros2 launch ddt2_assets load_hw.launch.py
```

如果正常则会生成以下topic

```
/actuator_states
/ddt2_controller/commands
/dynamic_joint_states
/gains/joint1/pid_state
/joint_states
/parameter_events
/robot_description
/rosout
/tf
/tf_static
```

##### 控制电机

往`/ddt2_controller/commands`上发命令则可以控制(单位为圈，相对于位置零点），例如：

```
ros2 topic pub /ddt2_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.5]}"
```

如果需要以指定速度到达指定位置的效果，则需在发上述控制命令前发送最大速度限制（单位为RPM，最大值为100），例如：

```
ros2 topic pub /ddt2_position_controller/velocity_limits std_msgs/msg/Float64MultiArray "{data: [3]}"
```

##### 电机信息可视化

> 利用plotjuggler来看ros topic信息

```
ros2 run plotjuggler plotjuggler
```

`/joint_states`：电机信息的effort,position,velocity

`/actuator_states`：电机的一般信息加电机can信号中的原始信息

![ddt2_hw_plotjuggler.png](docs%2Fddt2_hw_plotjuggler.png)

##### 电机参数修改

目前代码中默认电机can接在can0上，can id为51，如果需要修改

则在ddt2_assets的actuator.yaml中修改
![ddt2_hw_config.png](docs%2Fddt2_hw_config.png)

##### 电机零点修改

1.在电机上电，失能时（上电默认失能），用手将电机转到期望零点位置

2.读取当前位置

```
cansend can0 035#0400000000000000
```

会在can0上时收到如下反馈

```
can0  071   [8]  16 F2 FF FF FF FF FF FF
```

其中数据位的前4位（16F2）位是电机位置

```
建议使用candump can0 | grep 071来获取该信息
```

3.在ddt2_assets/one_actuator/config/actuator_coefficient.yaml中修改第九行`pos_offset`为2步获取的值(注意不要带空格)
![PositionOffset.png](docs%2FPositionOffset.png)
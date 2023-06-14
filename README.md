# Project Dragonfly Documentation

## Overview 

Project Dragonfly serves as a way to consolidate several low-profile sensors into a single, peripheral device, which attaches via USB to an drone in order to provide an estimation of state while keeping the device as small as possible. In doing so, we hope to create a modular, more cost effective way of providing state estimation, reducing the individual sensor configuration workload for drone manufacturers and hobbyists

The files containing the dragonfly PCB driver code can be found in the [SensorStick_integrated](https://github.com/markoristicc/Sensor_Stick/tree/prod/SensorStick_integrated) folder. This code was developed using the STM32CubeIDE, so that may be most convenient for you, however any embedded C development environment should suffice for flashing the code onto the PCB.  

## Getting Data off the Dragonfly for your own use

If you want to use the Dragonfly simply to collect data, we have provided code to do so, located in [UAVFirmware](https://github.com/markoristicc/Sensor_Stick/tree/prod/UAVFirmware) . If you are not already familiar with ROS, visiting [this](https://docs.ros.org/en/humble/Tutorials.html) documentation may be helpful, specifically the tutorials on creating a workspace, creating a package, writing a simply publisher & subscriber (python), and creating custom msg files. The code we are using assumes that you are running using ROS Humble on Ubuntu 22.02 and are using Python 3 >. 

In essence, this data parses the packet sent by the dragonfly and composes a custom message composed of the following message types: sensor_msgs/Imu, sensor_msgs/MagneticField, sensor_msgs/NavSatTransform, rosflight_msgs/Barometer, and rosflight_msgs/Airspeed. This message is published to the topic 'topic' (which can be changed in UAVFirmware/src/talker_listener/talker_listener/talker.py). To run the publisher enter the following commands into your terminal:

```
source /opt/ros/humble/setup.bash
colcon build --packages-select talker_listener 
```
On a separate terminal run
```
source install/setup.bash
ros2 run talker_listener talkerNode
```

And from there your ROS pubisher will be publishing these custom messages to 'topic' at a rate of 100Hz!


## Performing Visualizations

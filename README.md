# Dragonfly

## Overview 

Project Dragonfly serves as a way to consolidate several low-profile sensors into a single, peripheral device, which attaches via USB to an drone in order to provide an estimation of state while keeping the device as small as possible. In doing so, we hope to create a modular, more cost effective way of providing state estimation, reducing the individual sensor configuration workload for drone manufacturers and hobbyists

The files containing the dragonfly PCB driver code can be found in the /IntegratedDriver folder. This code was developed using the STM32CubeIDE, so that may be most convenient for you, however any embedded C development environment should suffice for flashing the PCB.  

## Getting Data off the Dragonfly for your own use

If you want to use the Dragonfly simply to collect data, we have provided code to do so, located in /DataRetreival . If you are not already familiar with ROS, visiting this documentation may be helpful. This code is assuming that you are running using ROS Humble on Ubuntu 22.02 and are using Python 3 >.

## Performing Visualizations

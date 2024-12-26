# Quadrotors-controller-using-LattePanda
 This repository provides instructions for the real-world implementation of Aggregated Hierarchical Sliding Mode Control (AHSMC) based on EKF for a drone, using the LattePanda Alpha as the flight controller and the Pixhawk FMUv2 for data collection.
 
**Before setting up the robot, please read the instructions:**   
 [Setup_Custom_Quad_Instruction.pdf](https://github.com/user-attachments/files/18249279/Setup_Custom_Quad_Instruction.pdf)

## Software Requirements & Setup

The custom controller is designed in:

- Ubuntu 22.04
- ROS2 Humble
- Mission Planner
- Mavlink 1.0

Follow these commands to install the package:

```shell
# Step 1: Create and build a colcon workspace:
$ mkdir -p ~/dev_ws/src
$ cd ~/dev_ws/
$ colcon build
$ echo "source ~/dev_ws/devel/setup.bash" >> ~/.bashrc

# Step 2: Clone this repo into your workspace
$ cd ~/dev_ws/src
Download the ahsmcquad folder

# Step 3: Build the colcon workspace for this package
$ cd ~/dev_ws
$ colcon build
```
# Contents
Please follow the PDF instructions to set up the drone and after that, run this code:

## a. **PID controller:**   

Follow these commands in order to fly the drone:

```
# get the data
ros2 run ahsmcquad getdata
# start the EKF
ros2 run ahsmcquad cascadedpositionpid
```


## b. **AHSMC controller:**   

Follow these commands in order to fly the drone:

```
# get the data
ros2 run ahsmcquad getdata
# start the EKF
ros2 run ahsmcquad hsmcposition
```


# Contact
- [Van Chung Nguyen](mailto:vanchungn@.unr.edu)
- [Hung La](mailto:hla@unr.edu)

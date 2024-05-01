# Toy Cleaning Robot

## Introduction

This is the repo for the Toy Cleaning team in WPI's RBE594 - Capstone Experience with Prof. Markus Nemitz (2024 Spring). The goal of this project is to implement a demonstrative simulation of a toy and room organizing robot. We chose a differential drive robot with an attached manipulator arm (ClearPath Jackal and Franka Panda respectively) to that end.

You can find the presentation for this project [here](https://github.com/anshuljindal876/toy_cleaner/tree/readme_updating/Presentation) and the demonstration [here](https://youtu.be/KUQGUB5UZtM?si=NhdE4qspo7lZLT3j).

See the image below for a PARI breakdown of the project:
![PARI Breakdown](https://github.com/anshuljindal876/toy_cleaner/blob/readme_updating/readme_utils/Screenshot%20from%202024-05-01%2012-32-49.png)

[Source for survey](https://swnsdigital.com/us/2017/06/parents-have-to-pick-up-after-their-kids-1500-times-a-year/)

## Dependencies

The simulation has been built and implemented in Nvidia Omniverse's IsaacSim, which is quickly becoming the industry standard for robot simulation.

Installation procedures for IsaacSim can be found [here](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/index.html).

**Minimum Specifications:**
- Ubuntu 20.04/22.04 or Windows 10/11
- Intel Core i7 (7th Generation)/AMD Ryzen 5
- 4 Cores
- 32GB RAM
- 50GB SSD
- 8GB VRAM
- RTX2070

We use ROS2 Humble for communication between nodes, and it can be installed from [here](https://docs.ros.org/en/humble/Installation.html).

Gazebo Classic is needed for the vision pipeline, and it can be installed from [here](https://gazebosim.org/docs/garden/ros_installation).

Additionally, you will also need PCL for the point cloud processing and its ROS2 interface:

For installing packages related to the Point Cloud Library (PCL) for ROS Humble.
```
sudo apt-get install ros-humble-pcl-*
```
For installing the Point Cloud Library (PCL) development files and libraries.
```
sudo apt install libpcl-dev
```


Finally, to plan the robot arm trajectories, we need MoveIt2. 
```
sudo apt install ros-humble-moveit
```

In case the binary install throws errors, please install from source: https://moveit.ros.org/install-moveit2/source/

## Understanding The Simulation

This simulation demonstrates the mobile manipulator identifying a bright red cone in a cluttered room, recording its coordinate using its point cloud, and then setting a point nearby as the goal. The differential drive then takes the robot there, after which, a hard-coded pick and lift sequence is carried out with the help of MoveIt2. The robot returns to the starting point and performs a similar place action. The actual picking could not be implemented due to problems with the physics engine.

Please take a look at the system diagram below:
![System Diagram](https://github.com/anshuljindal876/toy_cleaner/blob/readme_updating/readme_utils/Screenshot%20from%202024-05-01%2013-05-04.png)

The following is a more detailed look at the blocks of the system:

1. **Vision and PointCloud**
  The point cloud is generated from the stereo camera (Intel RealSense) and contains points corresponding to the whole environment. It is then transformed from the camera to the world frame, and the floor is removed using RANSAC-based plane fitting. Subsequently, we cluster the remaining point clouds using DBSCAN and isolate the different objects in the room. The RGB image then uses color keying to find which object is the red cone, and we project the point cloud back to the image frame and match it to the RGB image. Thus, the appropriate point cloud is selected. We find the centroid of the point cloud and set a point near it as the target.

2. **Differential Drive**
  **(ANSHUL ADD THE DIFF DRIVE STUFF)**

3. **MoveIt2**
  We use MoveIt2 to send position commands to the IsaacSim controller via the `/joint_states` topic. First, we perform inverse kinematics for the pose we wish to attain and then plan a path to that point. Once this path is executed, the simulator follows it.

  To run MoveIt2:
  ```
  ros2 run panda_moveit_config demo.launch.py
```

**(ANSHUL ADD THE Sim stuff)**

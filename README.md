# lidar_sim

A ROS 2 Humble simulation workspace for **LiDAR-only autonomous vehicle simulation**, including the `boot` and `pcl_merge` packages, with `citysim` as a submodule.  

This workspace is designed to run on **Ubuntu 22.04** with **ROS 2 Humble** and **Gazebo**.

It allows testing localization, obstacle detection, and LiDAR-based navigation in a simulated city environment.

---

## System Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo11

---

## Dependencies

### Basic Gazebo ROS 2 Packages

    sudo apt update

    curl -sSL http://get.gazebosim.org | sh

    sudo apt install ros-humble-gazebo-ros-pkgs
    sudo apt install ros-humble-gazebo-ros2-control

    sudo apt install ros-humble-velodyne-description
    sudo apt install ros-humble-velodyne-simulator
    sudo apt install ros-humble-pcl-ros

    sudo apt install ros-humble-navigation2
    sudo apt install ros-humble-nav2-bringup
    sudo apt install ros-humble-rviz2
    sudo apt install ros-humble-map-server
    sudo apt install ros-humble-tf2-tools





## Clone the Repository
### To clone this repository with the citysim submodule, run:

    mkdir ad_lidar_ws && cd ad_lidar_ws && mkdir src && cd src
    git clone --recurse-submodules https://github.com/tdvines/lidar_sim.git
    cd ..

## Build the Workspace
    colcon build --symlink-install

## Usage
### Launch the main simulation:
    source install/setup.bash
    ros2 launch boot/run.launch.py



### Author
Thomas D. Vines

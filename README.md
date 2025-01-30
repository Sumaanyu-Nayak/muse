# :compass: MUSE

This repository contains the ROS package `muse_ros`, part of the MUSE (Multi-sensor State Estimator) project of the DLS (Dynamic Legged Systems lab) state estimator.

The `muse_ros` package provides a ROS node and utilities for estimating the state of a quadruped robot using sensor data. It includes algorithms for state estimation, sensor fusion, and filtering.

The source code will be available after the paper's review process is finished.

<h2 align="center"> :warning: REPOSITORY UNDER DEVELOPMENT :warning:
    
The code will be available soon!
TODO list at the end of the page
</h2>



## :t-rex: Prerequisites
* [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu) (but it should be compatible with the other ROS versions)
* [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)

## :hammer_and_wrench: Building

To install the `muse_ros` package, follow these steps:

1. Clone this repository to your ROS workspace:
    ```
    git clone git@github.com:iit-DLSLab/MUSE.git
    ```

2. Build the package using `catkin_make`:
    ```
    cd muse_ros/muse_ws
    source devel/setup.bash
    catkin_make -j$(proc) install
    ```

## :scroll: TODO list
- [ ] Code
- [ ] Dockerization
- [ ] Support for ROS2

## :hugs: Contributing

Contributions to this repository are welcome.






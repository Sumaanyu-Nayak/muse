<h1 align="center"> MUSE: A Real-Time Multi-Sensor State Estimator for Quadruped Robots </h1>
<h3 align="center">Ylenia Nisticò, João Carlos Virgolino Soares, Lorenzo Amatucci, Geoff Fink and Claudio Semini</h3>	

## :paperclip: Paper
This paper has been accepted to IEEE Robotics and Automation Letter and it is available [here](https://arxiv.org/abs/2503.12101)!

If you like this work and would like to cite it (thanks):
```
@ARTICLE{10933515,
  author={Nisticò, Ylenia and Soares, João Carlos Virgolino and Amatucci, Lorenzo and Fink, Geoff and Semini, Claudio},
  journal={IEEE Robotics and Automation Letters}, 
  title={MUSE: A Real-Time Multi-Sensor State Estimator for Quadruped Robots}, 
  year={2025},
  volume={},
  number={},
  pages={1-8},
  keywords={Robots;Sensors;Robot sensing systems;Legged locomotion;Odometry;Cameras;Laser radar;Robot vision systems;Robot kinematics;Quadrupedal robots;state estimation;localization;sensor fusion;quadruped robots},
  doi={10.1109/LRA.2025.3553047}}
```

# :computer: Code

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






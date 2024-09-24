# :compass: MUSE

This repository contains the ROS package `muse_ros`, part of the MUSE (Multi-sensor State Estimator) project of the DLS (Dynamic Legged Systems lab) state estimator.

The `muse_ros` package provides a ROS node and utilities for estimating the state of the ANYmal B300 robot using sensor data. It includes algorithms for state estimation, sensor fusion, and filtering.

<h2 align="center"> :warning: REPOSITORY UNDER DEVELOPMENT :warning:

Check the TODO list at the end of the page
</h2>



## :t-rex: Prerequisites
* [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu) (but it should be compatible with the other ROS versions)
* [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)

## :hammer_and_wrench: Building

To install the `muse_anymal_ros` package, follow these steps:

1. Clone this repository to your ROS workspace:
    ```
    $ git clone git@github.com:ylenianistico/MUSE-dls_state_estimator.git
    ```

2. Build the package using `catkin_make`:
    ```
    $ cd muse_anymal_ros/muse_ws
    $ source devel/setup.bash
    $ catkin_make -j$(proc) install
    ```

## :play_or_pause_button: Usage

To use the `muse_anymal_ros` package, follow these steps:

1. Launch the main node:
    ```
    $ roslaunch state_estimator start.launch
    ```
    This will start a ROS node that will load some plugins. You should see the following in your terminal:
    ```
    Checking log directory for disk usage. This may take a while.
    Press Ctrl-C to interrupt
    Done checking log file disk usage. Usage is <1GB.

    started roslaunch server http://docker:37057/

    SUMMARY
    ========

    PARAMETERS
    * /rosdistro: noetic
    * /rosversion: 1.15.11
    * /state_estimator/plugin_blacklist: ['Timeout', 'Cons...
    * /state_estimator/plugin_whitelist: ['AttitudeEstimat...
    * /state_estimator/timeout_plugin: [{'name': 'Master...
    * /use_sim_time: True

    NODES
    /
        state_estimator (state_estimator/node)

    auto-starting new master
    process[master]: started with pid [6907]
    ROS_MASTER_URI=http://localhost:11311

    setting /run_id to a888b362-d124-11ee-b6c6-3005053ddcf4
    process[rosout-1]: started with pid [6917]
    started core service [/rosout]
    process[state_estimator-2]: started with pid [6924]
    [ INFO] [1708566587.683234240]: Starting state estimator
    [ INFO] [1708566587.706618104]: Plugin AttitudeEstimation loaded
    [ INFO] [1708566587.710962453]: Plugin AttitudeEstimation initialized
    [ INFO] [1708566587.710995173]: Plugin LegOdometry loaded
    [ INFO] [1708566587.715165404]: Plugin LegOdometry initialized
    [ INFO] [1708566587.715220503]: Plugin SensorFusion loaded
    [ INFO] [1708566587.720669214]: Plugin SensorFusion initialized

    ```

2. The `AttitudeEstimation` plugin estimates the orientation and angular velocity of your robot, using IMU measurements and possibly the orientation obtained with exteroceptive odometry (LiDAR or camera), if available. 

3. The `LegOdometry` plugin outputs the base velocity obtained using joint measurements. Currently, this is the only plugin dependent on the type of robot because it uses [Robcogen](https://github.com/ori-drs/quadruped_robcogen) to compute the robot kinematics and Jacobian.

4. The `Sensor Fusion` plugin fuses the estimated attitude, the measured base velocity, and the exteroceptive odometry in a Kalman Filter, and outputs the Pose and Twist of the robot.

## :octocat: Suggestions
You can use any kind of exteroceptive (e.g. camera or LiDAR) odometry. Possibly you will only have to modify the subscribers in `new_state_estimator/src/plugins/attitude_estimation_plugin.cpp` [here](https://github.com/ylenianistico/MUSE-dls_state_estimator/blob/main/muse_anymal_ros/muse_ws/src/new_state_estimator/state_estimator/src/plugins/attitude_estimation_plugin.cpp#L86) and `new_state_estimator/src/plugins/sensor_fusion_plugin.cpp` [here](https://github.com/ylenianistico/MUSE-dls_state_estimator/blob/main/muse_anymal_ros/muse_ws/src/new_state_estimator/state_estimator/src/plugins/sensor_fusion_plugin.cpp#L112).

If you have a LiDAR I strongly suggest using [KISS-ICP](https://github.com/PRBonn/kiss-icp)

## :scroll: TODO list
- [ ] Install KISS-ICP as a plugin library
- [ ] Install camera odometry as a plugin library

## :hugs: Contributing

Contributions to this repository are welcome.






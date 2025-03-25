# Install script for directory: /root/muse_ws/src/state_estimator_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/root/muse_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/state_estimator_msgs/msg" TYPE FILE FILES
    "/root/muse_ws/src/state_estimator_msgs/msg/jointState.msg"
    "/root/muse_ws/src/state_estimator_msgs/msg/actuatorForces.msg"
    "/root/muse_ws/src/state_estimator_msgs/msg/imu.msg"
    "/root/muse_ws/src/state_estimator_msgs/msg/timeSync.msg"
    "/root/muse_ws/src/state_estimator_msgs/msg/rawSensors.msg"
    "/root/muse_ws/src/state_estimator_msgs/msg/slip_detection.msg"
    "/root/muse_ws/src/state_estimator_msgs/msg/ForwardKinematics.msg"
    "/root/muse_ws/src/state_estimator_msgs/msg/FeetJacobians.msg"
    "/root/muse_ws/src/state_estimator_msgs/msg/attitude.msg"
    "/root/muse_ws/src/state_estimator_msgs/msg/leg_odometry.msg"
    "/root/muse_ws/src/state_estimator_msgs/msg/sensor_fusion.msg"
    "/root/muse_ws/src/state_estimator_msgs/msg/JointStateWithAcceleration.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/state_estimator_msgs/srv" TYPE FILE FILES
    "/root/muse_ws/src/state_estimator_msgs/srv/getActiveEstimators.srv"
    "/root/muse_ws/src/state_estimator_msgs/srv/getBlacklist.srv"
    "/root/muse_ws/src/state_estimator_msgs/srv/getEstimatorDescription.srv"
    "/root/muse_ws/src/state_estimator_msgs/srv/getWhitelist.srv"
    "/root/muse_ws/src/state_estimator_msgs/srv/listAllEstimators.srv"
    "/root/muse_ws/src/state_estimator_msgs/srv/pauseEstimator.srv"
    "/root/muse_ws/src/state_estimator_msgs/srv/resetEstimator.srv"
    "/root/muse_ws/src/state_estimator_msgs/srv/restartEstimator.srv"
    "/root/muse_ws/src/state_estimator_msgs/srv/resumeEstimator.srv"
    "/root/muse_ws/src/state_estimator_msgs/srv/startEstimator.srv"
    "/root/muse_ws/src/state_estimator_msgs/srv/stopEstimator.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/state_estimator_msgs/cmake" TYPE FILE FILES "/root/muse_ws/build/state_estimator_msgs/catkin_generated/installspace/state_estimator_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/root/muse_ws/devel/include/state_estimator_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/root/muse_ws/devel/share/roseus/ros/state_estimator_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/root/muse_ws/devel/share/common-lisp/ros/state_estimator_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/root/muse_ws/devel/share/gennodejs/ros/state_estimator_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/root/muse_ws/devel/lib/python3/dist-packages/state_estimator_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/root/muse_ws/devel/lib/python3/dist-packages/state_estimator_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/root/muse_ws/build/state_estimator_msgs/catkin_generated/installspace/state_estimator_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/state_estimator_msgs/cmake" TYPE FILE FILES "/root/muse_ws/build/state_estimator_msgs/catkin_generated/installspace/state_estimator_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/state_estimator_msgs/cmake" TYPE FILE FILES
    "/root/muse_ws/build/state_estimator_msgs/catkin_generated/installspace/state_estimator_msgsConfig.cmake"
    "/root/muse_ws/build/state_estimator_msgs/catkin_generated/installspace/state_estimator_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/state_estimator_msgs" TYPE FILE FILES "/root/muse_ws/src/state_estimator_msgs/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/state_estimator_msgs" TYPE DIRECTORY FILES "/root/muse_ws/src/state_estimator_msgs/include/state_estimator_msgs/" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()


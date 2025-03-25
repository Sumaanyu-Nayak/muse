# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "state_estimator_msgs: 12 messages, 11 services")

set(MSG_I_FLAGS "-Istate_estimator_msgs:/root/muse_ws/src/state_estimator_msgs/msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(state_estimator_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/jointState.msg" NAME_WE)
add_custom_target(_state_estimator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_estimator_msgs" "/root/muse_ws/src/state_estimator_msgs/msg/jointState.msg" ""
)

get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/actuatorForces.msg" NAME_WE)
add_custom_target(_state_estimator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_estimator_msgs" "/root/muse_ws/src/state_estimator_msgs/msg/actuatorForces.msg" ""
)

get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/imu.msg" NAME_WE)
add_custom_target(_state_estimator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_estimator_msgs" "/root/muse_ws/src/state_estimator_msgs/msg/imu.msg" ""
)

get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/timeSync.msg" NAME_WE)
add_custom_target(_state_estimator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_estimator_msgs" "/root/muse_ws/src/state_estimator_msgs/msg/timeSync.msg" ""
)

get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/rawSensors.msg" NAME_WE)
add_custom_target(_state_estimator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_estimator_msgs" "/root/muse_ws/src/state_estimator_msgs/msg/rawSensors.msg" "std_msgs/Header:state_estimator_msgs/actuatorForces:state_estimator_msgs/imu:state_estimator_msgs/timeSync"
)

get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/slip_detection.msg" NAME_WE)
add_custom_target(_state_estimator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_estimator_msgs" "/root/muse_ws/src/state_estimator_msgs/msg/slip_detection.msg" ""
)

get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/ForwardKinematics.msg" NAME_WE)
add_custom_target(_state_estimator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_estimator_msgs" "/root/muse_ws/src/state_estimator_msgs/msg/ForwardKinematics.msg" ""
)

get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/FeetJacobians.msg" NAME_WE)
add_custom_target(_state_estimator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_estimator_msgs" "/root/muse_ws/src/state_estimator_msgs/msg/FeetJacobians.msg" ""
)

get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/attitude.msg" NAME_WE)
add_custom_target(_state_estimator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_estimator_msgs" "/root/muse_ws/src/state_estimator_msgs/msg/attitude.msg" "std_msgs/Header"
)

get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/leg_odometry.msg" NAME_WE)
add_custom_target(_state_estimator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_estimator_msgs" "/root/muse_ws/src/state_estimator_msgs/msg/leg_odometry.msg" "std_msgs/Header"
)

get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/sensor_fusion.msg" NAME_WE)
add_custom_target(_state_estimator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_estimator_msgs" "/root/muse_ws/src/state_estimator_msgs/msg/sensor_fusion.msg" "std_msgs/Header"
)

get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/JointStateWithAcceleration.msg" NAME_WE)
add_custom_target(_state_estimator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_estimator_msgs" "/root/muse_ws/src/state_estimator_msgs/msg/JointStateWithAcceleration.msg" "std_msgs/Header"
)

get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/getActiveEstimators.srv" NAME_WE)
add_custom_target(_state_estimator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_estimator_msgs" "/root/muse_ws/src/state_estimator_msgs/srv/getActiveEstimators.srv" ""
)

get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/getBlacklist.srv" NAME_WE)
add_custom_target(_state_estimator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_estimator_msgs" "/root/muse_ws/src/state_estimator_msgs/srv/getBlacklist.srv" ""
)

get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/getEstimatorDescription.srv" NAME_WE)
add_custom_target(_state_estimator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_estimator_msgs" "/root/muse_ws/src/state_estimator_msgs/srv/getEstimatorDescription.srv" ""
)

get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/getWhitelist.srv" NAME_WE)
add_custom_target(_state_estimator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_estimator_msgs" "/root/muse_ws/src/state_estimator_msgs/srv/getWhitelist.srv" ""
)

get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/listAllEstimators.srv" NAME_WE)
add_custom_target(_state_estimator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_estimator_msgs" "/root/muse_ws/src/state_estimator_msgs/srv/listAllEstimators.srv" ""
)

get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/pauseEstimator.srv" NAME_WE)
add_custom_target(_state_estimator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_estimator_msgs" "/root/muse_ws/src/state_estimator_msgs/srv/pauseEstimator.srv" ""
)

get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/resetEstimator.srv" NAME_WE)
add_custom_target(_state_estimator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_estimator_msgs" "/root/muse_ws/src/state_estimator_msgs/srv/resetEstimator.srv" ""
)

get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/restartEstimator.srv" NAME_WE)
add_custom_target(_state_estimator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_estimator_msgs" "/root/muse_ws/src/state_estimator_msgs/srv/restartEstimator.srv" ""
)

get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/resumeEstimator.srv" NAME_WE)
add_custom_target(_state_estimator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_estimator_msgs" "/root/muse_ws/src/state_estimator_msgs/srv/resumeEstimator.srv" ""
)

get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/startEstimator.srv" NAME_WE)
add_custom_target(_state_estimator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_estimator_msgs" "/root/muse_ws/src/state_estimator_msgs/srv/startEstimator.srv" ""
)

get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/stopEstimator.srv" NAME_WE)
add_custom_target(_state_estimator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_estimator_msgs" "/root/muse_ws/src/state_estimator_msgs/srv/stopEstimator.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/jointState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_cpp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/actuatorForces.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_cpp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/imu.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_cpp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/timeSync.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_cpp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/rawSensors.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/muse_ws/src/state_estimator_msgs/msg/actuatorForces.msg;/root/muse_ws/src/state_estimator_msgs/msg/imu.msg;/root/muse_ws/src/state_estimator_msgs/msg/timeSync.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_cpp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/slip_detection.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_cpp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/ForwardKinematics.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_cpp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/FeetJacobians.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_cpp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/attitude.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_cpp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/leg_odometry.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_cpp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/sensor_fusion.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_cpp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/JointStateWithAcceleration.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
)

### Generating Services
_generate_srv_cpp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/getActiveEstimators.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_cpp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/getBlacklist.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_cpp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/getEstimatorDescription.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_cpp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/getWhitelist.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_cpp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/listAllEstimators.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_cpp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/pauseEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_cpp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/resetEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_cpp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/restartEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_cpp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/resumeEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_cpp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/startEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_cpp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/stopEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
)

### Generating Module File
_generate_module_cpp(state_estimator_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(state_estimator_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(state_estimator_msgs_generate_messages state_estimator_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/jointState.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_cpp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/actuatorForces.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_cpp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/imu.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_cpp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/timeSync.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_cpp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/rawSensors.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_cpp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/slip_detection.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_cpp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/ForwardKinematics.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_cpp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/FeetJacobians.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_cpp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/attitude.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_cpp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/leg_odometry.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_cpp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/sensor_fusion.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_cpp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/JointStateWithAcceleration.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_cpp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/getActiveEstimators.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_cpp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/getBlacklist.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_cpp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/getEstimatorDescription.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_cpp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/getWhitelist.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_cpp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/listAllEstimators.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_cpp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/pauseEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_cpp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/resetEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_cpp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/restartEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_cpp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/resumeEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_cpp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/startEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_cpp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/stopEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_cpp _state_estimator_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(state_estimator_msgs_gencpp)
add_dependencies(state_estimator_msgs_gencpp state_estimator_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS state_estimator_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/jointState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_eus(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/actuatorForces.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_eus(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/imu.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_eus(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/timeSync.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_eus(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/rawSensors.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/muse_ws/src/state_estimator_msgs/msg/actuatorForces.msg;/root/muse_ws/src/state_estimator_msgs/msg/imu.msg;/root/muse_ws/src/state_estimator_msgs/msg/timeSync.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_eus(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/slip_detection.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_eus(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/ForwardKinematics.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_eus(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/FeetJacobians.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_eus(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/attitude.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_eus(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/leg_odometry.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_eus(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/sensor_fusion.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_eus(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/JointStateWithAcceleration.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
)

### Generating Services
_generate_srv_eus(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/getActiveEstimators.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_eus(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/getBlacklist.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_eus(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/getEstimatorDescription.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_eus(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/getWhitelist.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_eus(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/listAllEstimators.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_eus(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/pauseEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_eus(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/resetEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_eus(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/restartEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_eus(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/resumeEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_eus(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/startEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_eus(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/stopEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
)

### Generating Module File
_generate_module_eus(state_estimator_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(state_estimator_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(state_estimator_msgs_generate_messages state_estimator_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/jointState.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_eus _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/actuatorForces.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_eus _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/imu.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_eus _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/timeSync.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_eus _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/rawSensors.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_eus _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/slip_detection.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_eus _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/ForwardKinematics.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_eus _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/FeetJacobians.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_eus _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/attitude.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_eus _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/leg_odometry.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_eus _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/sensor_fusion.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_eus _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/JointStateWithAcceleration.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_eus _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/getActiveEstimators.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_eus _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/getBlacklist.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_eus _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/getEstimatorDescription.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_eus _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/getWhitelist.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_eus _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/listAllEstimators.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_eus _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/pauseEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_eus _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/resetEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_eus _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/restartEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_eus _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/resumeEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_eus _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/startEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_eus _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/stopEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_eus _state_estimator_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(state_estimator_msgs_geneus)
add_dependencies(state_estimator_msgs_geneus state_estimator_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS state_estimator_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/jointState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_lisp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/actuatorForces.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_lisp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/imu.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_lisp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/timeSync.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_lisp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/rawSensors.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/muse_ws/src/state_estimator_msgs/msg/actuatorForces.msg;/root/muse_ws/src/state_estimator_msgs/msg/imu.msg;/root/muse_ws/src/state_estimator_msgs/msg/timeSync.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_lisp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/slip_detection.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_lisp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/ForwardKinematics.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_lisp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/FeetJacobians.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_lisp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/attitude.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_lisp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/leg_odometry.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_lisp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/sensor_fusion.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_lisp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/JointStateWithAcceleration.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
)

### Generating Services
_generate_srv_lisp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/getActiveEstimators.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_lisp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/getBlacklist.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_lisp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/getEstimatorDescription.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_lisp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/getWhitelist.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_lisp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/listAllEstimators.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_lisp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/pauseEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_lisp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/resetEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_lisp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/restartEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_lisp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/resumeEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_lisp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/startEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_lisp(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/stopEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
)

### Generating Module File
_generate_module_lisp(state_estimator_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(state_estimator_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(state_estimator_msgs_generate_messages state_estimator_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/jointState.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_lisp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/actuatorForces.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_lisp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/imu.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_lisp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/timeSync.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_lisp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/rawSensors.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_lisp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/slip_detection.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_lisp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/ForwardKinematics.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_lisp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/FeetJacobians.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_lisp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/attitude.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_lisp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/leg_odometry.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_lisp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/sensor_fusion.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_lisp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/JointStateWithAcceleration.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_lisp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/getActiveEstimators.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_lisp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/getBlacklist.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_lisp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/getEstimatorDescription.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_lisp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/getWhitelist.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_lisp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/listAllEstimators.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_lisp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/pauseEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_lisp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/resetEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_lisp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/restartEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_lisp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/resumeEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_lisp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/startEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_lisp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/stopEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_lisp _state_estimator_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(state_estimator_msgs_genlisp)
add_dependencies(state_estimator_msgs_genlisp state_estimator_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS state_estimator_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/jointState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_nodejs(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/actuatorForces.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_nodejs(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/imu.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_nodejs(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/timeSync.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_nodejs(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/rawSensors.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/muse_ws/src/state_estimator_msgs/msg/actuatorForces.msg;/root/muse_ws/src/state_estimator_msgs/msg/imu.msg;/root/muse_ws/src/state_estimator_msgs/msg/timeSync.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_nodejs(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/slip_detection.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_nodejs(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/ForwardKinematics.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_nodejs(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/FeetJacobians.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_nodejs(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/attitude.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_nodejs(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/leg_odometry.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_nodejs(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/sensor_fusion.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_nodejs(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/JointStateWithAcceleration.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
)

### Generating Services
_generate_srv_nodejs(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/getActiveEstimators.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_nodejs(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/getBlacklist.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_nodejs(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/getEstimatorDescription.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_nodejs(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/getWhitelist.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_nodejs(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/listAllEstimators.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_nodejs(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/pauseEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_nodejs(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/resetEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_nodejs(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/restartEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_nodejs(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/resumeEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_nodejs(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/startEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_nodejs(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/stopEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
)

### Generating Module File
_generate_module_nodejs(state_estimator_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(state_estimator_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(state_estimator_msgs_generate_messages state_estimator_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/jointState.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_nodejs _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/actuatorForces.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_nodejs _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/imu.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_nodejs _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/timeSync.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_nodejs _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/rawSensors.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_nodejs _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/slip_detection.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_nodejs _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/ForwardKinematics.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_nodejs _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/FeetJacobians.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_nodejs _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/attitude.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_nodejs _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/leg_odometry.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_nodejs _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/sensor_fusion.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_nodejs _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/JointStateWithAcceleration.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_nodejs _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/getActiveEstimators.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_nodejs _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/getBlacklist.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_nodejs _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/getEstimatorDescription.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_nodejs _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/getWhitelist.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_nodejs _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/listAllEstimators.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_nodejs _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/pauseEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_nodejs _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/resetEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_nodejs _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/restartEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_nodejs _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/resumeEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_nodejs _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/startEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_nodejs _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/stopEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_nodejs _state_estimator_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(state_estimator_msgs_gennodejs)
add_dependencies(state_estimator_msgs_gennodejs state_estimator_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS state_estimator_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/jointState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_py(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/actuatorForces.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_py(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/imu.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_py(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/timeSync.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_py(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/rawSensors.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/muse_ws/src/state_estimator_msgs/msg/actuatorForces.msg;/root/muse_ws/src/state_estimator_msgs/msg/imu.msg;/root/muse_ws/src/state_estimator_msgs/msg/timeSync.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_py(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/slip_detection.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_py(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/ForwardKinematics.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_py(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/FeetJacobians.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_py(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/attitude.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_py(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/leg_odometry.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_py(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/sensor_fusion.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_py(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/msg/JointStateWithAcceleration.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
)

### Generating Services
_generate_srv_py(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/getActiveEstimators.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_py(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/getBlacklist.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_py(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/getEstimatorDescription.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_py(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/getWhitelist.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_py(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/listAllEstimators.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_py(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/pauseEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_py(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/resetEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_py(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/restartEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_py(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/resumeEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_py(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/startEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_py(state_estimator_msgs
  "/root/muse_ws/src/state_estimator_msgs/srv/stopEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
)

### Generating Module File
_generate_module_py(state_estimator_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(state_estimator_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(state_estimator_msgs_generate_messages state_estimator_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/jointState.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_py _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/actuatorForces.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_py _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/imu.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_py _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/timeSync.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_py _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/rawSensors.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_py _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/slip_detection.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_py _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/ForwardKinematics.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_py _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/FeetJacobians.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_py _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/attitude.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_py _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/leg_odometry.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_py _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/sensor_fusion.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_py _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/msg/JointStateWithAcceleration.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_py _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/getActiveEstimators.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_py _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/getBlacklist.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_py _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/getEstimatorDescription.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_py _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/getWhitelist.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_py _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/listAllEstimators.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_py _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/pauseEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_py _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/resetEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_py _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/restartEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_py _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/resumeEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_py _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/startEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_py _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/muse_ws/src/state_estimator_msgs/srv/stopEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_py _state_estimator_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(state_estimator_msgs_genpy)
add_dependencies(state_estimator_msgs_genpy state_estimator_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS state_estimator_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(state_estimator_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(state_estimator_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(state_estimator_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(state_estimator_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(state_estimator_msgs_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(state_estimator_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(state_estimator_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(state_estimator_msgs_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(state_estimator_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(state_estimator_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(state_estimator_msgs_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(state_estimator_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(state_estimator_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(state_estimator_msgs_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(state_estimator_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()

# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "pluginlib;state_estimator_msgs;roscpp;iit_commons".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lstate_estimator_commons;-lstate_estimator_plugins;-lstate_estimator_models".split(';') if "-lstate_estimator_commons;-lstate_estimator_plugins;-lstate_estimator_models" != "" else []
PROJECT_NAME = "state_estimator"
PROJECT_SPACE_DIR = "/root/muse_ws/install"
PROJECT_VERSION = "0.0.1"

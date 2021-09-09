#!/usr/bin/env sh
# generated from catkin/cmake/template/local_setup.sh.in

# since this file is sourced either use the provided _CATKIN_SETUP_DIR
# or fall back to the destination set at configure time
<<<<<<< HEAD:build/custom_msgs/atomic_configure/local_setup.sh
: ${_CATKIN_SETUP_DIR:=/home/robot03/robot03/devel/.private/custom_msgs}
=======
: ${_CATKIN_SETUP_DIR:=/home/kento/robot03/devel/.private/joy_control}
>>>>>>> dev:build/joy_control/atomic_configure/local_setup.sh
CATKIN_SETUP_UTIL_ARGS="--extend --local"
. "$_CATKIN_SETUP_DIR/setup.sh"
unset CATKIN_SETUP_UTIL_ARGS

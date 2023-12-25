#!/bin/bash

echo "🐢 Sourcing ROS2 Humble..."
source /opt/ros/humble/setup.bash

#echo "🔗 Configuring the ROS DDS..."
# FASTRTPS_DEFAULT_PROFILES_FILE=/navigator/data/fastrtps.xml
# RMW_FASTRTPS_USE_QOS_FROM_XML=1
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# source /opt/cyclone_ws/install/setup.bash


echo "👍 Finished environment setup"

if [ -z ${@+x} ]; then
    exec bash
else 
    exec bash -c "$@"
fi
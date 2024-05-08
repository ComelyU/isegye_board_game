#! /bin/bash
echo "[ros_setup shell script start...]"

# setup.bash
source /opt/ros/foxy/setup.bash
source ~/turtlebot3_ws/install/setup.bash

# export
export ROS_DOMAIN_ID=30
export LDS_MODEL=LDS-02
export TURTLEBOT3_MODEL=burger
echo "ROS_DOMAIN_ID : $ROS_DOMAIN_ID, LDS_MODEL : $LDS_MODEL, TURTLEBOT3_MODEL : $TURTLEBOT3_MODEL"
echo "[Complete!!]"

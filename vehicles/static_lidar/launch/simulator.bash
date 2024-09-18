#! /usr/bin/env bash
if [ $# -eq 0 ];
then
    worldfile=turbine_playground.sdf
elif [ $# -eq 1 ];
then
    worldfile=$1
else
    echo "Too many arguments"
    exit 1
fi
(trap 'kill 0' SIGINT;
 ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:= $PWD/config/static_lidar.yaml &\
 ros2 run micro_ros_agent micro_ros_agent udp4 --middleware dds --verbose 4 --port 2019 --ros-args -r __node:=micro_ros_agent -r __ns:=/ &\
 gz sim -v4 -r $worldfile)
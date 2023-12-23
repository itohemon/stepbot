#!/bin/bash

maps_path=${HOME}/ros2_ws/src/stepbot/maps
map_path=${HOME}/ros2_ws/src/stepbot/maps/$1

if [ ! -e ${maps_path} ]; then
    mkdir -p $maps_path
fi

if [ ! -e ${map_path} ]; then
    mkdir -p $map_path
fi

ros2 run nav2_map_server map_saver_cli -f $map_path/map


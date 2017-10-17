#!/bin/bash

if [[ $# != 3 ]]; then
    echo "[Usage] : ./gen_grid.sh <dim(+-)> <size> <thickness>"
    echo "[IARC Config] ./gen_grid.sh 10 1.0 0.08"
    exit 0
fi

d=$1
s=$2
t=$3

echo "
<?xml version="1.0"?>
<sdf version="1.4">
<model name="IARCGrid">
    <static>true</static>
    "
for ((i=-$d;i<=$d;++i)); do

    echo "
    <link name="\"x$i\"">
    <pose>$i 0 0.01 0 0 0</pose>
    <visual name=\"visual\">
      <geometry>
        <box>
            <size>.08 $((d*2)) .01</size>
        </box>
      </geometry>
    </visual>
    </link>
    "

    echo "
    <link name="\"y$i\"">
    <pose>0 $i 0.01 0 0 0</pose>
    <visual name=\"visual\">
      <geometry>
        <box>
            <size>$((d*2)) .08 .01</size>
        </box>
      </geometry>
    </visual>
    </link>
    "
done

echo "
    </model>
</sdf>
"

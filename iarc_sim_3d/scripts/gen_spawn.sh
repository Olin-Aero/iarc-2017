#!/bin/bash

if [[ $# != 3 ]]; then
    echo "[Usage] : ./gen_spawn.sh <name> <num> <radius>"
    echo "[Obstacle Config] ./gen_spawn.sh obstacle 4 5.0"
    echo "[Target Config] ./gen_spawn.sh target 10 1.0"
    exit 0
fi

pi=$(echo "4*a(1)" | bc -l)
name=$1
num=$2
rad=$3

echo "<launch>"
for ((i=0;i<$num;++i)); do
    Y=$(echo "$i*2*${pi}/$num" | bc -l)
    x=$(echo "$rad*c(${Y})" | bc -l)
    y=$(echo "$rad*s(${Y})" | bc -l)

    if [ "$name" = "obstacle" ]; then
        # rotate by 90 deg. clockwise
        Y=$(echo "$Y-${pi}/2.0" | bc -l)
    fi

    echo "
    <include file=\"\$(find iarc_sim_3d)/launch/spawn_roomba.launch\">
        <arg name=\"ns\" value=\"$name$i\"/>
        <arg name=\"x\" value=\"$x\"/>
        <arg name=\"y\" value=\"$y\"/>
        <arg name=\"Y\" value=\"$Y\"/>
        <arg name=\"pole\" value=\"1.$((RANDOM % 1000))\"/>
    </include>"
    #echo "roslaunch iarc_sim_3d spawn_roomba.launch ns:=target$i x:=$x y:=$y Y:=$Y"
done
echo "</launch>"

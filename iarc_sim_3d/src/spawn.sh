#!/bin/bash
pi=$(echo "4*a(1)" | bc -l)
for i in {0..9}; do
    echo "[Spawn.sh] Spawning $i"
    Y=$(echo "$i*2*${pi}/10" | bc -l)
    x=$(echo "c(${Y})" | bc -l)
    y=$(echo "s(${Y})" | bc -l)
    roslaunch iarc_sim_3d spawn_roomba.launch ns:=target$i x:=$x y:=$y Y:=$Y &
done

#!/bin/bash

trap "exit" INT
declare -a arr=("V1_01_easy" "V1_02_medium" "V1_03_difficult" "V2_01_easy" "V2_02_medium" "V2_03_difficult") # 
if [ $# -ne 1 ]; then
    echo $0: usage: ./basalt-test.bash loop_number
    exit 1
fi

loop=$1
for i in "${arr[@]}"
do
  echo "running ${i}"
  echo $i >> stats_basalt.txt

  for ((j=0;j<${loop};j++));
  do
    echo "loop number $(($j+1))/${loop}"
    echo "loop number $(($j+1))/${loop}" >> stats_basalt.txt

    start_time="$(date -u +%s%3N)"
    ./../../basalt/build/basalt_vio --dataset-path ../src/VI-Stereo-DSO/dataset/${i}/ --cam-calib /usr/etc/basalt/euroc_ds_calib.json --dataset-type euroc --config-path /usr/etc/basalt/euroc_config.json --marg-data euroc_marg_data --show-gui 0 --num-threads 1 --save-trajectory tum > /dev/null &
    sleep 1
    pid=$(pidof -s basalt_vio)
    pidstat -p ${pid} 1 > cpu_basalt_${i}_${j}.txt &

    #wait $pid
    tail --pid=$pid -f /dev/null
    end_time="$(date -u +%s%3N)"
    killall -SIGINT pidstat
    elapsed="$(($end_time-$start_time))"
    echo "time elapsed in milliseconds: ${elapsed}" 
    echo "time elapsed in milliseconds: ${elapsed}" >> stats_basalt.txt 

    evo_ape tum truths/tum/${i}.tum trajectory.txt -as --save_plot ./basalt-${i}-${j}.pdf >> stats_basalt.txt
    mv trajectory.txt traj_${i}_${j}.txt

  done
done

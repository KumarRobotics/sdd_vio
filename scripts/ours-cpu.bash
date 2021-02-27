#!/bin/bash

trap "exit" INT
declare -a arr=("V1_02_medium") # "V1_02_medium" "V1_03_difficult" "V2_01_easy" "V2_02_medium" "V2_03_difficult")

if [ $# -ne 1 ]; then
    echo $0: usage: ./ours-cpu.bash loop_number
    exit 1
fi

loop=$1
mkdir cpus_ours 

for i in "${arr[@]}"
do
  echo "running ${i}.bag"

  for ((j=0;j<${loop};j++));
  do
    echo "loop number ${j+1}/${loop}"

    roslaunch sdd_vio vio_nodelet_euroc.launch > /dev/null &
    sleep 3

    pidstat -p $(pidof -s nodelet) 1 > cpu_ours_${i}_${j}.txt & 

    rosbag play ../src/sdd_vio/bagfiles/${i}.bag
    
    killall -SIGINT roslaunch
    killall -SIGINT pidstat
  done
done

# clean up
#mv cpu_ours_*.txt cpus_ours/

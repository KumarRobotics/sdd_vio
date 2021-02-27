#!/bin/bash

trap "exit" INT
declare -a arr=("V1_02_medium") # "V1_02_medium" "V1_03_difficult" "V2_01_easy" "V2_02_medium" "V2_03_difficult")

if [ $# -ne 1 ]; then
    echo $0: usage: ./rovio-test.bash loop_number
    exit 1
fi

loop=$1
echo "writing results to stats_rovio.txt"
mkdir trajbags_rovio

for i in "${arr[@]}"
do
  echo $i >> stats_rovio.txt
  echo "running ${i}.bag"

  for ((j=0;j<${loop};j++));
  do
    echo "loop number ${j+1}/${loop}"

    ./record.sh -O record &
    roslaunch rovio rovio_node.launch > /dev/null &
    sleep 3

    pidstat -p $(pidof -s rovio_node) 1 > cpu_rovio_${i}_${j}.txt &

    rosbag play ../src/sdd_vio/bagfiles/${i}.bag

    killall -SIGINT record
    killall -SIGINT roslaunch
    killall -SIGINT pidstat
    sleep 3

    # - merge vicon topic from original bag
    ./mergebag.py -o merge.bag -t /vicon/firefly_sbx/firefly_sbx record.bag ../src/sdd_vio/bagfiles/${i}.bag
    ./bag_transform.py merge.bag
    ./bag_times.py pose_stamped.bag

    evo_ape bag output.bag /vicon/pose_stamped /rovio/pose_stamped -as  >> stats_rovio.txt

    mv output.bag trajbags_rovio/traj_rovio_${i}_${j}.bag

  done
done

# clean up
rm *.bag


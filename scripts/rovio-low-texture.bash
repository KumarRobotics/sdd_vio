#!/bin/bash

trap "exit" INT
declare -a arr=("30hz-blob-1" "30hz-blob-2" "30hz-blob-3" "30hz-blob-4" "10hz-carried-1" "10hz-carried-2" "10hz-carried-3" "10hz-carried-4" "10hz-carried-5" "30hz-flying-1" "30hz-flying-2") #

if [ $# -ne 1 ]; then
    echo $0: usage: ./rovio-test.bash loop_number
    exit 1
fi

loop=$1
echo "writing results to stats_rovio_low.txt"
mkdir trajbags_rovio_low

for i in "${arr[@]}"
do
  echo $i >> stats_rovio_low.txt
  echo "running ${i}.bag"

  for ((j=0;j<${loop};j++));
  do

    ./record.sh -O record &
    roslaunch rovio rovio_node_snapdragon.launch > /dev/null &
    sleep 3

    rosbag play ../bagfiles/featureless/Dataset/${i}.bag

    killall -SIGINT record
    killall -SIGINT roslaunch
    sleep 3

    # - merge vicon topic from original bag
    ./mergebag.py -o merge.bag -t /vicon/dragonfly10/odom,/dragonfly10/odom record.bag ../bagfiles/featureless/Dataset/${i}.bag
    ./bag_transform.py merge.bag
    ./bag_times.py pose_stamped.bag

    evo_ape bag output.bag /vicon/pose_stamped /rovio/pose_stamped -as --save_plot ./rovio-low-${i}-${j} >> stats_rovio_low.txt

    mv output.bag trajbags_rovio_low/traj_rovio_${i}_${j}.bag

  done
done

# clean up
rm *.bag


#!/bin/bash

trap "exit" INT
declare -a arr=("V1_01_easy" "V1_02_medium" "V1_03_difficult" "V2_01_easy" "V2_02_medium" "V2_03_difficult") 
if [ $# -ne 1 ]; then
    echo $0: usage: ./ours-test.bash loop_number
    exit 1
fi

loop=$1
echo "writing results to stats_ours.txt"
#mkdir -p trajbags_ours

for i in "${arr[@]}"
do
  echo $i >> stats_ours.txt
  echo "running ${i}.bag"

  for ((j=0;j<${loop};j++));
  do
    echo "loop number ${j+1}/${loop}"

    ./record.sh -O record &
    roslaunch sdd_vio bag_reader.launch bag_name:=${i}.bag > /dev/null
    echo "finished recording"

    #pid=$(pidof -s sdd_vio_bag_reader)
    #wait $pid
    killall -SIGINT record
    sleep 3

    # merge vicon topic from original bag
    ./mergebag.py -o merge.bag -t /vicon/firefly_sbx/firefly_sbx,vicon/firefly_sbx/firefly_sbx record.bag ../src/sdd_vio/bagfiles/${i}.bag
    # transform recorded odom topics to pose_stamped
    ./bag_transform.py merge.bag
    # rewrite bag timestamp
    ./bag_times.py pose_stamped.bag

    evo_ape bag output.bag /vicon/pose_stamped /sdd_vio_node/pose_stamped -as  >> stats_ours.txt

#    mv output.bag trajbags_ours/traj_ours_${i}_${j}.bag
  # clean up
  rm *.bag

  done
done


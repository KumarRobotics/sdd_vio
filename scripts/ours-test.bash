#!/bin/bash

trap "exit" INT
declare -a arr=("V1_02_medium") # "V1_02_medium" "V1_03_difficult" "V2_01_easy" "V2_02_medium" "V2_03_difficult")

if [ $# -ne 1 ]; then
    echo $0: usage: ./ours-test.bash loop_number
    exit 1
fi

loop=$1
echo "writing results to stats_ours.txt"
mkdir trajbags_ours

for i in "${arr[@]}"
do
  echo $i >> stats_ours.txt
  echo "running ${i}.bag"

  for ((j=0;j<${loop};j++));
  do
    echo "loop number ${j+1}/${loop}"

    ./record.sh -O record &
    roslaunch sdd_vio bag_reader.launch bag_name:=${i}.bag > /dev/null

    killall -SIGINT record
    killall -SIGINT roslaunch
    sleep 3

    # merge vicon topic from original bag
    ./mergebag.py -o merge.bag -t /vicon/firefly_sbx/firefly_sbx -t vicon/firefly_sbx/firefly_sbx record.bag ../src/sdd_vio/bagfiles/${i}.bag
    # transform recorded odom topics to pose_stamped
    ./bag_transform.py merge.bag
    # rewrite bag timestamp
    ./bag_times.py pose_stamped.bag

    evo_ape bag output.bag /vicon/pose_stamped /sdd_vio_node/pose_stamped -as  >> stats_ours.txt

    mv output.bag trajbags_ours/traj_ours_${i}_${j}.bag

  done
done

# clean up
#rm *.bag

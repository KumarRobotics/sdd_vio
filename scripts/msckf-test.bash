#!/bin/bash

trap "exit" INT
declare -a arr=("V1_02_medium") # "V1_02_medium" "V1_03_difficult" "V2_01_easy" "V2_02_medium" "V2_03_difficult")

if [ $# -ne 1 ]; then
    echo $0: usage: ./msckf-test.bash loop_number
    exit 1
fi

loop=$1
echo "writing results to stats_msckf.txt"
mkdir trajbags_msckf

for i in "${arr[@]}"
do
  echo $i >> stats_msckf.txt
  echo "running ${i}.bag"

  for ((j=0;j<${loop};j++));
  do
    echo "loop number ${j+1}/${loop}"

    ./record.sh -O record &
    roslaunch msckf_vio msckf_vio_euroc.launch > /dev/null &
    sleep 3

    pidfirst=$(pidof -s nodelet)
    pidsecond=$(pidof -s nodelet -o ${pidfirst})
    pidstat -p ${pidfirst} 1 > cpu_msckf_1_${i}_${j}.txt &
    pidstat -p ${pidsecond} 1 > cpu_msckf_2_${i}_${j}.txt &

    rosbag play ../src/sdd_vio/bagfiles/${i}.bag

    killall -SIGINT record
    killall -SIGINT roslaunch
    killall -SIGINT pidstat
    sleep 3

    # - merge vicon topic from original bag
    ./mergebag.py -o merge.bag -t /vicon/firefly_sbx/firefly_sbx record.bag ../src/sdd_vio/bagfiles/${i}.bag
    ./bag_transform.py merge.bag
    ./bag_times.py pose_stamped.bag

    evo_ape bag output.bag /vicon/pose_stamped /msckf/pose_stamped -as  >> stats_msckf.txt

    mv output.bag trajbags_msckf/traj_msckf_${i}_${j}.bag

  done
done

# clean up
rm *.bag


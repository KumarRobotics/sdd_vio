#!/usr/bin/env bash

# Source our workspace directory to load ENV variables
source /home/wenxin/vio_ws/devel/setup.bash


#=============================================================
#=============================================================
#=============================================================


# estimator configurations
modes=(
    #"mono"
    #"binocular"
    "stereo"
)

# dataset locations
bagnames=(
     "V1_01_easy"
     "V1_02_medium"
     "V1_03_difficult"
     "V2_01_easy"
     "V2_02_medium"
     "V2_03_difficult"
#   "MH_01_easy"
#   "MH_02_easy"
#   "MH_03_medium"
#   "MH_04_difficult"
#   "MH_05_difficult"
)

# how far we should start into the dataset
# this can be used to skip the initial sections
bagstarttimes=(
     "0"
     "0"
     "0"
     "0"
     "0"
     "0"
    #"40"
    #"35"
    #"15"
    #"20"
    #"20"
)

# threshold for variance to detect if the unit has moved yet
imuthreshold=(
    "1.5"
    "1.5"
    "1.5"
    "1.5"
    "1.5"
    "1.5"
    #"1.5"
    #"1.5"
    #"1.5"
    #"1.5"
)

# location to save log files into
save_path1="/home/wenxin/vio_ws/scripts"
bag_path="/home/wenxin/vio_ws/src/sdd_vio/bagfiles"


#=============================================================
#=============================================================
#=============================================================


# Loop through all modes
for h in "${!modes[@]}"; do
# Loop through all datasets
for i in "${!bagnames[@]}"; do

# Monte Carlo runs for this dataset
# If you want more runs, change the below loop
for j in {00..02}; do

echo "${bagnames[i]} loop ${j}" >> stats_ov.txt
echo "running ${bagnames[i]} in loop ${j}"

# start timing
start_time="$(date -u +%s)"
filename_est="$save_path1/ov_2.3_${modes[h]}/${bagnames[i]}/${start_time}_estimate.txt"

# number of cameras
if [ "${modes[h]}" == "mono" ]
then
    temp1="1"
    temp2="true"
fi
if [ "${modes[h]}" == "binocular" ]
then
    temp1="2"
    temp2="false"
fi
if [ "${modes[h]}" == "stereo" ]
then
    temp1="2"
    temp2="true"
fi

# run our ROS launch file (note we send console output to terminator)
roslaunch ov_msckf pgeneva_ros_eth.launch max_cameras:="$temp1" use_stereo:="$temp2" bag:="$bag_path/${bagnames[i]}.bag" bag_start:="${bagstarttimes[i]}" init_imu_thresh:="${imuthreshold[i]}" dosave:="true" path_est:="$filename_est" > /dev/null &
sleep 3 

pid=$(pidof -s run_subscribe_msckf)
pidstat -p ${pid} 1 > cpu_open_vins_${bagnames[i]}_${j}.txt &

# print out the time elapsed
tail --pid=$pid -f /dev/null
killall -SIGINT pidstat
end_time="$(date -u +%s)"
elapsed="$(($end_time-$start_time))"
echo "BASH: ${modes[h]} - ${bagnames[i]} - run $j took $elapsed seconds";

rosrun ov_eval error_singlerun posyaw ${filename_est}  truths/rpe/${bagnames[i]}.txt >> stats_ov.txt

done


done
done



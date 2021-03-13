#!/bin/bash

trap "exit" INT
declare -a arr=("V1_01_easy" "V1_02_medium" "V1_03_difficult" "V2_01_easy" "V2_02_medium" "V2_03_difficult")  #  
if [ $# -ne 1 ]; then
    echo $0: usage: ./dso-test.bash loop_number
    exit 1
fi

loop=$1
echo "writing results to stats_dso.txt"

for i in "${arr[@]}"
do  
  echo $i >> statistics.txt
  echo $i >> stats_nt_dso.txt
  
  echo "running ${i} dataset"

  for ((j=0;j<$(($loop));j++));
  do
    echo "loop number ${j+1}/${loop}"

    ../src/VI-Stereo-DSO/build/bin/dso_dataset \
    files0=../src/VI-Stereo-DSO/dataset/${i}/mav0/cam0/data \
    files1=../src/VI-Stereo-DSO/dataset/${i}/mav0/cam1/data \
    calib0=../src/VI-Stereo-DSO/calib/euroc/cam0.txt \
    calib1=../src/VI-Stereo-DSO/calib/euroc/cam1.txt \
    T_stereo=../src/VI-Stereo-DSO/calib/euroc/T_C0C1.txt \
    imu_info=../src/VI-Stereo-DSO/calib/euroc/IMU_info.txt \
    groundtruth=../src/VI-Stereo-DSO/dataset/${i}/mav0/state_groundtruth_estimate0/data.csv \
    imudata=../src/VI-Stereo-DSO/dataset/${i}/mav0/imu0/data.csv \
    pic_timestamp=../src/VI-Stereo-DSO/dataset/${i}/mav0/cam0/data.csv \
    pic_timestamp1=../src/VI-Stereo-DSO/dataset/${i}/mav0/cam1/data.csv \
    preset=0\
    mode=1 quiet=0 nomt=1 \
    savefile_tail=nt_${i}_${j}\
    use_stereo=1 \
    imu_weight=6 \
    imu_weight_tracker=0.6 \
    stereo_weight=0.5 \
    nolog=1 nogui=1 > /dev/null &
    sleep 1

    pid=$(pidof -s dso_dataset)
    #pidstat -p ${pid} 1 > cpu_dso_${i}_${j}.txt & 

    wait $pid
    #killall -SIGINT pidstat 

    evo_ape tum ./data/nt_${i}_${j}_gt.txt ./data/nt_${i}_${j}.txt -as --save_plot ./nt-${i}-${j}.pdf >> stats_nt_dso.txt

  done
done


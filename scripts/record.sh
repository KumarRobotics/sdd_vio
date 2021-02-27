#!/bin/bash

rosbag record /vicon/firefly_sbx/firefly_sbx /firefly_sbx/vio/odom /rovio/odometry /sdd_vio_node/pose_stamped /vins_estimator/odometry $@

/*-------------------------------------------------------------
Copyright 2019 Wenxin Liu, Kartik Mohta, Giuseppe Loianno

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
--------------------------------------------------------------*/

#ifndef VISUALIZATION_H_
#define VISUALIZATION_H_

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include "sdd_vio/utils/math_utils.h"


namespace sdd_vio {

void visualize_points(ros::Publisher& pub_pts,
	const vector_aligned<Eigen::Vector3f>& feat_3D);

void visualize_cams(ros::Publisher& pub_cam, ros::Publisher &pub_cam_poseStamped, const Eigen::Isometry3f& T, const Eigen::Vector3f &v, ros::Time& timestamp, double seq);

}


#endif

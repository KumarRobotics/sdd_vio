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

#include "sdd_vio/visualization.h"
#include "sdd_vio/utils/math_utils.h"
#include <iostream>
#include <time.h>
#include <geometry_msgs/PoseStamped.h>

namespace sdd_vio {


void visualize_points(ros::Publisher& pub_pts,
    const vector_aligned<Eigen::Vector3f>& feat_3D)
{
    sensor_msgs::PointCloud pointCloud;
    int npts = feat_3D.size();

    std_msgs::Header header_msg;
    header_msg.frame_id = "world";
    header_msg.stamp = ros::Time::now();

    /* prepare pointCloud msg to publish */
    pointCloud.header = header_msg;

    for (int i=0;i<npts;++i)
    {
        geometry_msgs::Point32 p;
        p.x = feat_3D[i](0);
        p.y = feat_3D[i](1);
        p.z = feat_3D[i](2);
        pointCloud.points.push_back(p);
    }

    pub_pts.publish(pointCloud);
}


void visualize_cams(ros::Publisher& pub_cam, ros::Publisher& pub_cam_poseStamped, const Eigen::Isometry3f& T, const Eigen::Vector3f& v, ros::Time& timestamp, double seq)
{
    Eigen::Vector3f t = T.translation();
    Eigen::Quaternionf q(T.rotation());

    nav_msgs::Odometry cam;

    std_msgs::Header header_msg;
    header_msg.frame_id = "world";
    header_msg.stamp = timestamp;
    header_msg.seq = seq;

    /* prepare cam msg to publish */
    cam.header = header_msg;
    cam.pose.pose.position.x = t(0);
    cam.pose.pose.position.y = t(1);
    cam.pose.pose.position.z = t(2);
    cam.pose.pose.orientation.x = q.x();
    cam.pose.pose.orientation.y = q.y();
    cam.pose.pose.orientation.z = q.z();
    cam.pose.pose.orientation.w = q.w();
    cam.twist.twist.linear.x = v(0);
    cam.twist.twist.linear.y = v(1);
    cam.twist.twist.linear.z = v(2);

    pub_cam.publish(cam);

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = header_msg;
    pose_stamped.pose = cam.pose.pose;
    pub_cam_poseStamped.publish(pose_stamped);

}


}  // namespace sdd_vio

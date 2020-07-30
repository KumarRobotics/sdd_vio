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

#include <Eigen/Core>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <image_transport/image_transport.h>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <tuple>

namespace sdd_vio {

class PinholeCameraStereo;
class VoStereo;

class SddVioNodelet : public nodelet::Nodelet {
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SddVioNodelet();
  ~SddVioNodelet();

  void onInit();
  void loadParams(ros::NodeHandle &nh);
  void initRosIO(ros::NodeHandle &nh);
  void initVo(ros::NodeHandle &nh);

  /* Monocular callback function */
  void imgCb(const sensor_msgs::ImageConstPtr& msg);
  /* Stereo callback function */
  void imgCb(const sensor_msgs::ImageConstPtr& msg_0,
      const sensor_msgs::ImageConstPtr& msg_1);
  /* IMU callback function */
  void imuCb(const sensor_msgs::Imu::ConstPtr& msg);

  /* visualization publisher */
  image_transport::Publisher pub_rect_[2];

  /* visualization publisher */
  ros::Publisher pub_points_;
  ros::Publisher pub_cam_;
  ros::Publisher pub_cam_poseStamped_;

  /* ROS node image subscriber */
  message_filters::Subscriber<sensor_msgs::Image> image_sub_left_;
  message_filters::Subscriber<sensor_msgs::Image> image_sub_right_;
  message_filters::Subscriber<sensor_msgs::Imu> imu_sub_;
  std::unique_ptr<message_filters::Cache<sensor_msgs::Imu>> imu_cache_;
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
      sensor_msgs::Image>
      MySyncPolicy;
  std::unique_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

  private:
  bool initial_orientation_set = false;

  /* pointer to camera object */
  std::unique_ptr<sdd_vio::PinholeCameraStereo> stereoCam_;

  /* pointer to vo pipeline object */
  std::unique_ptr<sdd_vio::VoStereo> vo_;

  /* IMU Covariance */
  Eigen::Matrix<float, 6, 6> Cov_noise_;

  /* IMU integration */
  std::tuple<Eigen::Matrix3f, Eigen::Vector3f, Eigen::Vector3f, float,
      Eigen::Matrix<float, 9, 9>>
  imuIntegrate(std::vector<sensor_msgs::Imu::ConstPtr>& imu_vector) const;

  bool pub_image_debug_; // flag to enable publishing debug images
  bool input_rectified_;

  ros::Time time_last_; // keep track of time spent between frames
  ros::Time
      last_timestamp_; // keep the timestamp of last image - note difference
                       // between time_last, time_last uses ros::Time:now()
  double seq_last_;    // keep track of last frame sequence

}; // end class SddVioNodelet
}

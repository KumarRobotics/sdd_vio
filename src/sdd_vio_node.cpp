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

#include "sdd_vio/pinhole_camera_stereo.h"
#include "sdd_vio/utils/math_utils.h"
#include "sdd_vio/visualization.h"
#include "sdd_vio/vo_stereo.h"

#include "opencv2/core/version.hpp"

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <string>
#include <tuple>
#include <opencv2/ximgproc.hpp>

namespace sdd_vio {

using namespace message_filters;

class PinholeCameraStereo;
class VoStereo;

class SddVioNode {

  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SddVioNode();
  ~SddVioNode();

  void loadParams(ros::NodeHandle &nh);
  void initRosIO(ros::NodeHandle &nh);
  void initVo(ros::NodeHandle &nh);
  ros::NodeHandle nh_;

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

}; // end class SddVioNode



SddVioNode::SddVioNode()
    : nh_("~")
    , pub_image_debug_(false)
    , input_rectified_(true)
    , time_last_(ros::Time::now())
    , last_timestamp_(ros::Time::now())
    , seq_last_(0)
{
  std::cout << "--------------------------" << std::endl;

  std::cout << "onInit: Loading params." << std::endl;
  loadParams(nh_);

  std::cout << "onInit: Init ROS IO." << std::endl;
  initRosIO(nh_);

  std::cout << "onInit: Initializing VO object." << std::endl;
  initVo(nh_);

  std::cout << "onInit: vo pipeline initialized." << std::endl;
  std::cout << "--------------------------" << std::endl;
}

SddVioNode::~SddVioNode()
{
  std::cout << "Destroying SddVioNode object..." << std::endl;
  vo_->reset(); // get statistics
}

std::tuple<Eigen::Matrix3f, Eigen::Vector3f, Eigen::Vector3f, float,
    Eigen::Matrix<float, 9, 9>>
SddVioNode::imuIntegrate(
    std::vector<sensor_msgs::Imu::ConstPtr>& imu_vector) const
{

  int num_measurement = imu_vector.size();
  Eigen::Matrix3f R_meas;
  Eigen::Vector3f v_meas;
  Eigen::Vector3f p_meas;
  float t_meas = 0;
  R_meas << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  v_meas << 0, 0, 0;
  p_meas << 0, 0, 0;

  Eigen::Matrix<float, 9, 9> Cov_meas;
  if (imu_vector.size() != 0)
    Cov_meas.setZero();
  else
    Cov_meas.setIdentity();
  Eigen::Matrix<float, 9, 9> A;
  A.setIdentity();
  Eigen::Matrix<float, 9, 6> B;
  B.setZero();

  // not to use the last element
  for (int i = 0; i < num_measurement - 1; i++) {
    Eigen::Vector3f w; // angular velocity
    Eigen::Vector3f a; // linear acceleration
    w(0) = imu_vector[i]->angular_velocity.x;
    w(1) = imu_vector[i]->angular_velocity.y;
    w(2) = imu_vector[i]->angular_velocity.z;
    a(0) = imu_vector[i]->linear_acceleration.x;
    a(1) = imu_vector[i]->linear_acceleration.y;
    a(2) = imu_vector[i]->linear_acceleration.z;

    /* remove bias */
    w = w - vo_->getImuBiasGyro();
    a = a - vo_->getImuBiasAccel();

    /* obtain delta t */
    float delta_t
        = (imu_vector[i + 1]->header.stamp - imu_vector[i]->header.stamp)
              .toSec();

    /* update A, B */
    Eigen::Matrix<float, 3, 3> delta_R_jm1_j = exp_SO3<float>(w * delta_t);
    Eigen::Matrix<float, 3, 3> a_hat = hat(a);

    A.block<3, 3>(0, 0) = delta_R_jm1_j.transpose();
    A.block<3, 3>(3, 0) = -R_meas * a_hat * delta_t;
    A.block<3, 3>(6, 0) = -0.5 * R_meas * a_hat * delta_t * delta_t;
    A.block<3, 3>(6, 3) = delta_t * Eigen::MatrixXf::Identity(3, 3);

    B.block<3, 3>(0, 0) = exp_Jacobian<float>(w * delta_t) * delta_t;
    B.block<3, 3>(3, 3) = R_meas * delta_t;
    B.block<3, 3>(6, 3) = 0.5 * R_meas * delta_t * delta_t;

    /* update measurement covariance */
    Cov_meas = A * Cov_meas * A.transpose() + B * Cov_noise_ * B.transpose();
    //        std::cout<<"Cov :\n"<<Cov_meas<<"\n";
    /* update measurements */
    t_meas += delta_t;
    R_meas *= delta_R_jm1_j;
    v_meas += R_meas * a * delta_t;
    p_meas += v_meas * delta_t + 0.5 * R_meas * a * delta_t * delta_t;
  }

  return std::make_tuple(R_meas, v_meas, p_meas, t_meas, Cov_meas);
}

void SddVioNode::imgCb(const sensor_msgs::ImageConstPtr& msg_0,
    const sensor_msgs::ImageConstPtr& msg_1)
{

  if (!initial_orientation_set)
    return;

  ROS_INFO_STREAM("=========Entering imgCb function===========");

  /* terminal output to check if images are in sync */
  static int num_callbacks = 0; // number of times the callback function is entered
  static int total_frames_lost = 0;
  static double callback_timer = 0; // time for callback function

  double seq0 = msg_0->header.seq;           // seq for image
  ros::Time timestamp = msg_0->header.stamp; // timestamp for image

  /* evaluate against last callback */
  ros::Duration duration = ros::Time::now() - time_last_;
  double seq_jump = seq0 - seq_last_;
  time_last_ = ros::Time::now();

  ROS_INFO_STREAM("Time spent in the last callback: " << duration.toSec());

  if (seq_jump != 1 && num_callbacks != 0) {
    ROS_WARN("Frame lost!!");
    std::cout << "Number of frames lost: " << seq_jump - 1 << std::endl;
    total_frames_lost = total_frames_lost + seq_jump - 1;
    std::cout << "Total number of frames lost: " << total_frames_lost
              << std::endl;
  }

  ros::Time t1, t6;
  t1 = ros::Time::now();

  /* obtain IMU data */
  std::vector<sensor_msgs::Imu::ConstPtr> imu_vector
      = imu_cache_->getInterval(last_timestamp_, timestamp);
  if (imu_vector.size() == 0 && num_callbacks != 0)
    ROS_WARN_STREAM(
        "Fail to retrieve IMU data between current frame and last frame.");

  last_timestamp_ = timestamp;
  seq_last_ = seq0;

  /* IMU pre-integration */
  Eigen::Matrix3f R_meas;
  Eigen::Vector3f v_meas;
  Eigen::Vector3f p_meas;
  float t_meas;
  Eigen::Matrix<float, 9, 9> Cov_meas;
  std::tie(R_meas, v_meas, p_meas, t_meas, Cov_meas) = imuIntegrate(imu_vector);
  vo_->importImuMeas(R_meas, v_meas, p_meas, t_meas, Cov_meas);

  /* obtain original images */
  cv::Mat img0, img1;
  try {
    img0 = cv_bridge::toCvShare(msg_0, "mono8")->image;
    img1 = cv_bridge::toCvShare(msg_1, "mono8")->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  /* rectify images - not used */
  cv::Mat dst0, dst1;
  if (input_rectified_ == true) {
      dst0 = img0;
      dst1 = img1;
  }
  else {
      stereoCam_->undistortImage(img0, dst0, img1, dst1);
  }

  /* pass rectified images to vo pipeline */
  vo_->addImagePair(dst0, dst1, msg_0->header.stamp.toSec(), msg_0->header.seq);

  /* publish debug images */
  if (pub_image_debug_ == true) {
    cv_bridge::CvImage out_cv_img0(
        msg_0->header, msg_0->encoding, sdd_vio::adjustVis(vo_->visual1));
    cv_bridge::CvImage out_cv_img1(
        msg_1->header, msg_1->encoding, sdd_vio::adjustVis(vo_->visual2));
    pub_rect_[0].publish(out_cv_img0.toImageMsg());
    pub_rect_[1].publish(out_cv_img1.toImageMsg());
  }

  /* publish camera pose and pointcloud outputs */
  if (vo_->visCamUpdate()) {
    sdd_vio::visualize_cams(pub_cam_, pub_cam_poseStamped_, vo_->getVisPose(),
        vo_->getVisVel(), timestamp, msg_0->header.seq);
  }
  if (vo_->visPtsUpdate())
    sdd_vio::visualize_points(pub_points_, vo_->getVisPoints());

  t6 = ros::Time::now();
  duration = t6 - t1;
  callback_timer = callback_timer + duration.toSec();
  ROS_INFO_STREAM(
      "time for executing callback function in sec: " << duration.toSec());

  num_callbacks++;
}


static Eigen::Matrix<float, 3, 1> R_to_ypr(const Eigen::Matrix<float, 3, 3>& R)
{
  Eigen::Matrix<float, 3, 1> n = R.block(0, 0, 3, 1);
  Eigen::Matrix<float, 3, 1> o = R.block(0, 1, 3, 1);
  Eigen::Matrix<float, 3, 1> a = R.block(0, 2, 3, 1);

  Eigen::Matrix<float, 3, 1> ypr;
  float y = atan2(n(1, 0), n(0, 0));
  float p = atan2(-n(2, 0), n(0, 0) * cos(y) + n(1, 0) * sin(y));
  float r = atan2(a(0, 0) * sin(y) - a(1, 0) * cos(y),
      -o(0, 0) * sin(y) + o(1, 0) * cos(y));
  ypr(0, 0) = y;
  ypr(1, 0) = p;
  ypr(2, 0) = r;

  return ypr;
}

static Eigen::Matrix<float, 3, 3> ypr_to_R(
    const Eigen::Matrix<float, 3, 1>& ypr)
{
  float c, s;
  Eigen::Matrix<float, 3, 3> Rz;
  Rz.setZero();
  float y = ypr(0, 0);
  c = cos(y);
  s = sin(y);
  Rz(0, 0) = c;
  Rz(1, 0) = s;
  Rz(0, 1) = -s;
  Rz(1, 1) = c;
  Rz(2, 2) = 1;

  Eigen::Matrix<float, 3, 3> Ry;
  Ry.setZero();
  float p = ypr(1, 0);
  c = cos(p);
  s = sin(p);
  Ry(0, 0) = c;
  Ry(2, 0) = -s;
  Ry(0, 2) = s;
  Ry(2, 2) = c;
  Ry(1, 1) = 1;

  Eigen::Matrix<float, 3, 3> Rx;
  Rx.setZero();
  float r = ypr(2, 0);
  c = cos(r);
  s = sin(r);
  Rx(1, 1) = c;
  Rx(2, 1) = s;
  Rx(1, 2) = -s;
  Rx(2, 2) = c;
  Rx(0, 0) = 1;

  Eigen::Matrix<float, 3, 3> R = Rz * Ry * Rx;
  return R;
}

/* callback function for IMU data */
void SddVioNode::imuCb(const sensor_msgs::Imu::ConstPtr& msg)
{
  static const int total_num = 200; // the number of imus gathered from the ground for init
  static int data_i = 0; // the number of data processed
  static float w1 = 0, w2 = 0, w3 = 0, a1 = 0, a2 = 0, a3 = 0;
  if (data_i < total_num) // calibration on bias
  {
    ROS_INFO("Imu Seq: [%d]", msg->header.seq);
    w1 += msg->angular_velocity.x;
    w2 += msg->angular_velocity.y;
    w3 += msg->angular_velocity.z;
    a1 += msg->linear_acceleration.x;
    a2 += msg->linear_acceleration.y;
    a3 += msg->linear_acceleration.z;

    if (data_i == total_num - 1) // the last data collected for calibration
    {
      Eigen::Vector3f w_b(w1 / total_num, w2 / total_num, w3 / total_num);
      vo_->setImuBiasGyro(w_b);
      std::cout << "calibrated gyro bias: \n" << w_b << "\n";

      Eigen::Vector3f gravity_imu(
          a1 / total_num, a2 / total_num, a3 / total_num);
      float gravity_norm = gravity_imu.norm();
      std::cout << "gravity_imu: " << gravity_imu << std::endl;
      std::cout << "gravity_norm: " << gravity_norm << std::endl;
      const Eigen::Vector3f gravity = Eigen::Vector3f(0.0, 0.0, -gravity_norm);
      vo_->setGravity(gravity);

      // TODO: initialize orientation
      Eigen::Quaternionf initial_orientation
          = Eigen::Quaternionf::FromTwoVectors(gravity_imu, -gravity);
      std::cout << "initial_orientation: " << initial_orientation.coeffs()
                << "\n";
      Eigen::Matrix3f R(initial_orientation);
      auto ypr = R_to_ypr(R);
      ypr(0) = 0;
      R = ypr_to_R(ypr);
      initial_orientation = R;
      vo_->setInitPose(Eigen::Isometry3f(initial_orientation));

      initial_orientation_set = true;
    }
    data_i++;
  }
}

void SddVioNode::loadParams(ros::NodeHandle &nh)
{
  nh.param("vo/pub_image_debug", pub_image_debug_, false);
  nh.param("vo/input_rectified", input_rectified_, true);

  /* initialize stereo camera */
  stereoCam_.reset(new sdd_vio::PinholeCameraStereo(nh));
  if (!stereoCam_->isRectified())
    ROS_WARN("Rectification maps are not initialized!");
  std::cout << "onInit: Stereo camera object initialized." << std::endl;

  /* initialize IMU gyro and accel noise covariance */
  std::vector<double> var_w;
  if (!nh.getParam("vo/var_w", var_w))
    ROS_ERROR("Failed to get gyro noise variance from server.");
  std::vector<double> var_a;
  if (!nh.getParam("vo/var_a", var_a))
    ROS_ERROR("Failed to get accel noise variance from server.");
  Eigen::Matrix<float, 6, 1> var_d;
  var_d << var_w[0], var_w[1], var_w[2], var_a[0], var_a[1], var_a[2];
  Cov_noise_ = var_d.asDiagonal();
}

void SddVioNode::initRosIO(ros::NodeHandle &nh)
{
  /* initialize visualization - rectified image publisher */
  image_transport::ImageTransport it(nh);
  pub_rect_[0] = it.advertise("debug_img1", 1);
  pub_rect_[1] = it.advertise("debug_img2", 1);

  /* initialize visualization - 3D point publisher */
  pub_points_ = nh.advertise<sensor_msgs::PointCloud>("points", 10);
  pub_cam_ = nh.advertise<nav_msgs::Odometry>("cam", 10);
  pub_cam_poseStamped_
      = nh.advertise<geometry_msgs::PoseStamped>("pose_stamped", 10);

  /* initialize subscribers and callbacks */
  image_sub_left_.subscribe(
      nh, "cam_topic_left", 10, ros::TransportHints().tcpNoDelay());
  image_sub_right_.subscribe(
      nh, "cam_topic_right", 10, ros::TransportHints().tcpNoDelay());
  sync_.reset(new message_filters::Synchronizer<MySyncPolicy>(
      MySyncPolicy(100), image_sub_left_, image_sub_right_));
  sync_->registerCallback(boost::bind(&sdd_vio::SddVioNode::imgCb, this, _1, _2));

  imu_sub_.subscribe(nh, "imu_topic", 500, ros::TransportHints().tcpNoDelay());
  imu_cache_.reset(new message_filters::Cache<sensor_msgs::Imu>(imu_sub_, 100));
  imu_cache_->registerCallback(
      boost::bind(&sdd_vio::SddVioNode::imuCb, this, _1));
}

void SddVioNode::initVo(ros::NodeHandle &nh)
{
  vo_.reset(new sdd_vio::VoStereo(stereoCam_.get(), "sdd_vio_node", nh));
  vo_->start();
}

}

/*--------------*/
/*     main     */
/*--------------*/


int main(int argc, char** argv)
{

    ros::init(argc, argv, "sdd_vio_node");
    sdd_vio::SddVioNode sdd_vio_node;

    while(ros::ok()){
 		ros::spinOnce();
 	}

	printf("VINS terminated.\n");
    return  0;
}

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

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>
#include <message_filters/simple_filter.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>

#include "sdd_vio/sdd_vio_nodelet.h"
#include "sdd_vio/vo_stereo.h"

using namespace sdd_vio;

/**
 * Inherits from message_filters::SimpleFilter<M>
 * to use protected signalMessage function
 */
template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M> {
  public:
  void newMessage(const boost::shared_ptr<M const>& msg)
  {
    this->signalMessage(msg);
  }
};

class SddVioBagReader {
  public:
  SddVioBagReader(ros::NodeHandle& nh);
  void run();

  private:
  void initSddVioNodelet(ros::NodeHandle& nh);
  void callback(const sensor_msgs::Image::ConstPtr& l_img,
      const sensor_msgs::Image::ConstPtr& r_img);

  SddVioNodelet sdd_vio_nodelet_;

  // Set up fake subscribers to capture images
  BagSubscriber<sensor_msgs::Image> l_img_sub_, r_img_sub_;
  BagSubscriber<sensor_msgs::Imu> imu_sub_;

  // Use time synchronizer to make sure we get properly synchronized images
  std::unique_ptr<message_filters::Synchronizer<SddVioNodelet::MySyncPolicy>>
      sync_;

  rosbag::Bag bag_;
  std::string left_image_topic_, right_image_topic_, imu_topic_;
};

SddVioBagReader::SddVioBagReader(ros::NodeHandle& nh)
{
  initSddVioNodelet(nh);

  std::string filename;
  nh.param("bag_file", filename, std::string(""));
  bag_.open(filename, rosbag::bagmode::Read);

  nh.param(
      "left_image_topic", left_image_topic_, std::string("left/image_raw"));
  nh.param(
      "right_image_topic", right_image_topic_, std::string("right/image_raw"));
  nh.param("imu_topic", imu_topic_, std::string("imu"));
}

void SddVioBagReader::callback(const sensor_msgs::Image::ConstPtr& l_img,
    const sensor_msgs::Image::ConstPtr& r_img)
{
  std::cout << "SddVioBagReader::callback\n";
  sdd_vio_nodelet_.imgCb(l_img, r_img);
}

void SddVioBagReader::initSddVioNodelet(ros::NodeHandle& nh)
{
  sdd_vio_nodelet_.loadParams(nh);

  image_transport::ImageTransport it(nh);
  sdd_vio_nodelet_.pub_rect_[0] = it.advertise("debug_img1", 1);
  sdd_vio_nodelet_.pub_rect_[1] = it.advertise("debug_img2", 1);

  sdd_vio_nodelet_.pub_points_
      = nh.advertise<sensor_msgs::PointCloud>("points", 10);
  sdd_vio_nodelet_.pub_cam_ = nh.advertise<nav_msgs::Odometry>("cam", 10);
  sdd_vio_nodelet_.pub_cam_poseStamped_
      = nh.advertise<geometry_msgs::PoseStamped>("pose_stamped", 10);

  sdd_vio_nodelet_.imu_cache_.reset(
      new message_filters::Cache<sensor_msgs::Imu>(imu_sub_, 100));
  sdd_vio_nodelet_.imu_cache_->registerCallback(
      boost::bind(&sdd_vio::SddVioNodelet::imuCb, &sdd_vio_nodelet_, _1));

  sync_.reset(new message_filters::Synchronizer<SddVioNodelet::MySyncPolicy>(
      SddVioNodelet::MySyncPolicy(100), l_img_sub_, r_img_sub_));
  sync_->registerCallback(boost::bind(&SddVioBagReader::callback, this, _1, _2));

  std::cout << "onInit: Initializing VO object." << std::endl;
  sdd_vio_nodelet_.initVo(nh);
}

void SddVioBagReader::run()
{
  rosbag::View view(bag_);

  std::cout << "left_image_topic_: " << left_image_topic_ << '\n';
  std::cout << "right_image_topic_: " << right_image_topic_ << '\n';
  std::cout << "imu_topic_: " << imu_topic_ << '\n';

  // Load all messages into our stereo dataset
  BOOST_FOREACH (rosbag::MessageInstance const m, view) {
    ros::spinOnce();

    if (m.getTopic() == left_image_topic_) {
      sensor_msgs::Image::ConstPtr l_img = m.instantiate<sensor_msgs::Image>();
      if (l_img != NULL)
        l_img_sub_.newMessage(l_img);
    }

    if (m.getTopic() == right_image_topic_) {
      sensor_msgs::Image::ConstPtr r_img = m.instantiate<sensor_msgs::Image>();
      if (r_img != NULL)
        r_img_sub_.newMessage(r_img);
    }

    if (m.getTopic() == imu_topic_) {
      sensor_msgs::Imu::ConstPtr imu = m.instantiate<sensor_msgs::Imu>();
      if (imu != NULL)
        imu_sub_.newMessage(imu);
    }
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "sdd_vio_bag_reader");

  ros::NodeHandle nh("~");

  SddVioBagReader bag_reader(nh);

  bag_reader.run();

  return 0;
}

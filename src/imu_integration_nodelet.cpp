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
#include <nodelet/nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/cache.h>
#include <nav_msgs/Odometry.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>
#include <tuple>

namespace sdd_vio {

class ImuIntegrationNodelet : public nodelet::Nodelet
{
   private:
    void onInit(void);

    bool has_received_first_vio_;
    bool new_odom_available_;  // if true, then a new odom has been received and IMU should re-integrate from that point.
    ros::Time vio_odom_timestamp_;  // The timestamp of new odom, same as image timestamp
    double vio_odom_seq_;

    /* the odom from vio, these are with respect to the initial IMU frame */
    Eigen::Vector3f p_vio_;
    Eigen::Isometry3f T_vio_;
    Eigen::Quaternionf q_vio_;
    Eigen::Vector3f v_vio_;

    /* the actual current state (still relative to the initial IMU frame) that needs to be published */
    Eigen::Vector3f p_;
    Eigen::Isometry3f T_;
    Eigen::Quaternionf q_;
    Eigen::Vector3f v_;


    /* fixed params */
    Eigen::Isometry3f T_world_imu_;  // transformation between world frame and imu body frame
    Eigen::Vector3f b_w_;  // gyro bias
    Eigen::Vector3f b_a_;  // accelerometer bias
    Eigen::Vector3f g_;  // gravity in initial IMU frame

    ros::Time last_timestamp_;


    /* publisher to publish high rate odometry */
    ros::Publisher pub_odom_;
    /* subscriber to tightly-coupled vio odometry low frequency output */
    message_filters::Subscriber<nav_msgs::Odometry> sub_odom_;
    /* subscriber to IMU messages */
    message_filters::Subscriber<sensor_msgs::Imu> sub_imu_;
    /* IMU cache linked to subscriber */
    message_filters::Cache<sensor_msgs::Imu>* imu_cache_;

    /* publish odom at IMU rate */
    void imu_callback(const sensor_msgs::Imu::ConstPtr& imu);
    /* set flag for incoming vio odometry to trigger IMU re-integration */
    void vio_callback(const nav_msgs::Odometry::ConstPtr& odom);
    /* integrate IMU from last image pose to current */
    std::tuple<Eigen::Vector3f, Eigen::Vector3f, Eigen::Quaternionf> imuIntegrate(const std::vector<sensor_msgs::Imu::ConstPtr> &imu_vector) const;


};


void ImuIntegrationNodelet::onInit(void)
{
  ros::NodeHandle nh = getPrivateNodeHandle();

  /* initialization */
  p_vio_.setZero(3);
  T_vio_.setIdentity();
  q_vio_.setIdentity();
  v_vio_.setZero(3);

  p_.setZero(3);
  T_.setIdentity();
  q_.setIdentity();
  v_.setZero(3);

  b_w_.setZero(3);  // imu biases
  b_a_.setZero(3);
  g_ << 0,0,9.8;  // gravity
  T_world_imu_.setIdentity();
  Eigen::Matrix3f R_world_imu;
  R_world_imu << 1,0,0,
                0,-1,0,
                0,0,-1;
  T_world_imu_.linear() = R_world_imu;

  //std::cout<<"isometry T_: "<<T_.matrix()<<"\n";
  //std::cout<<"isometry T_world_imu_: "<<T_world_imu_.matrix()<<"\n";

  last_timestamp_ = ros::Time::now();

  new_odom_available_ = false;
  has_received_first_vio_ = false;

  /* initialize publisher */
  pub_odom_ = nh.advertise<nav_msgs::Odometry>("odom", 100);
  /* initialize vio odom subscriber */
  sub_odom_.subscribe(nh, "odom_topic", 100, ros::TransportHints().tcpNoDelay());
  sub_odom_.registerCallback(boost::bind(&ImuIntegrationNodelet::vio_callback, this, _1));
  /* initialize imu subscriber and cache callback */
  sub_imu_.subscribe(nh, "imu_topic", 500, ros::TransportHints().tcpNoDelay());
  imu_cache_ = new message_filters::Cache<sensor_msgs::Imu>(sub_imu_,500);
  imu_cache_->registerCallback(boost::bind(&ImuIntegrationNodelet::imu_callback, this, _1));

}


/* upon receiving an odom message, save the timestamp and odom info, set flag */
void ImuIntegrationNodelet::vio_callback(const nav_msgs::Odometry::ConstPtr &odom)
{

    new_odom_available_ = true;
    has_received_first_vio_ = true;
    vio_odom_timestamp_ = odom->header.stamp;
    vio_odom_seq_ = odom->header.seq;

    p_vio_(0) = odom->pose.pose.position.x;
    p_vio_(1) = odom->pose.pose.position.y;
    p_vio_(2) = odom->pose.pose.position.z;

    q_vio_.x() = odom->pose.pose.orientation.x;
    q_vio_.y() = odom->pose.pose.orientation.y;
    q_vio_.z() = odom->pose.pose.orientation.z;
    q_vio_.w() = odom->pose.pose.orientation.w;

    T_vio_.linear() = q_vio_.toRotationMatrix();
    T_vio_.translation() = p_vio_;

    v_vio_(0) = odom->twist.twist.linear.x;
    v_vio_(1) = odom->twist.twist.linear.y;
    v_vio_(2) = odom->twist.twist.linear.z;

}

void ImuIntegrationNodelet::imu_callback(const sensor_msgs::Imu::ConstPtr &imu)
{

    ros::Time current_timestamp = imu->header.stamp;


    /* raw observation */
    Eigen::Vector3f w, a;
    w(0) = imu->angular_velocity.x;
    w(1) = imu->angular_velocity.y;
    w(2) = imu->angular_velocity.z;
    a(0) = imu->linear_acceleration.x;
    a(1) = imu->linear_acceleration.y;
    a(2) = imu->linear_acceleration.z;

    /* Calibration of IMU */
    static const int total_num = 300;
    static int data_i = 0;  // the number of data processed
    static float w1 = 0, w2 = 0, w3 = 0, a1 = 0, a2 = 0, a3 = 0;
    if (data_i < total_num)  // take first 300 data - for calibration on bias
    {
        ROS_INFO("ImuIntegration nodelet - Imu Seq: [%d]", imu->header.seq);
        w1 += w(0);
        w2 += w(1);
        w3 += w(2);
        a1 += a(0) + g_(0);
        a2 += a(1) + g_(1);
        a3 += a(2) + g_(2);

        if (data_i == total_num-1)  // the last date collected for calibration
        {
            Eigen::Vector3f w_b(w1/total_num, w2/total_num, w3/total_num), a_b(a1/total_num, a2/total_num, a3/total_num);
            b_w_ = w_b;
            //b_a_ = a_b;
            std::cout<<"calibrated gyro bias: \n"<<w_b<<"\n";
            std::cout<<"calibrated accel bias: \n"<<a_b<<"\n";
//            throw std::runtime_error("Calibration done!");
        }
        data_i++;
    }


    if (new_odom_available_)
    {
        //std::cout<<"Received new odom from VIO! Getting imu data vector from buffer: \n";
        //std::cout<<"vio_odom_timestamp: "<<vio_odom_timestamp_<<"\n";
        //std::cout<<"current_timestamp: "<<current_timestamp<<"\n";
        std::vector<sensor_msgs::Imu::ConstPtr> imu_vector = imu_cache_->getInterval(vio_odom_timestamp_, current_timestamp);
        if (imu_vector.size() == 0)
          ROS_WARN_STREAM("Fail to retrieve IMU data between current frame and last frame.");
        std::tie(p_, v_, q_) = imuIntegrate(imu_vector);
        new_odom_available_ = false;

    }

    if (data_i != 1 && has_received_first_vio_)  // if not the first IMU data
    {
      /* getting current odometry in initial IMU frame */
      Eigen::Vector3f w_mBias = w - b_w_;
      Eigen::Vector3f a_mBias = a - b_a_;
      float delta_t = (current_timestamp - last_timestamp_).toSec();

      /* increment T_pub by one IMU measurement */
      Eigen::AngleAxisf aa_inc(w_mBias.norm() * delta_t, w_mBias.normalized());
      p_ = p_ + v_*delta_t + 0.5*g_*delta_t*delta_t + 0.5*q_.toRotationMatrix()*(a_mBias*delta_t*delta_t);
      v_ = v_ + g_*delta_t + q_.toRotationMatrix()*(a_mBias*delta_t);
      q_ = q_ * aa_inc;

      T_.linear() = q_.toRotationMatrix();
      T_.translation() = p_;
    }

    last_timestamp_ = current_timestamp;


    /* transform to world frame */
    Eigen::Isometry3f T_pub = T_world_imu_ * T_ * T_world_imu_.inverse();
    Eigen::Vector3f v_pub = T_world_imu_ * v_;
    Eigen::Quaternionf q_pub(T_pub.rotation());
    Eigen::Vector3f p_pub = T_pub.translation();

    /* publish odometry */
    nav_msgs::Odometry odom;
    std_msgs::Header header_msg;
    header_msg.frame_id = "world";
    header_msg.stamp = imu->header.stamp;  // use original imu message's header
    header_msg.seq = imu->header.seq;
    /* prepare cam msg to publish */
    odom.header = header_msg;
    odom.pose.pose.position.x = p_pub(0);
    odom.pose.pose.position.y = p_pub(1);
    odom.pose.pose.position.z = p_pub(2);
    odom.pose.pose.orientation.x = q_pub.x();
    odom.pose.pose.orientation.y = q_pub.y();
    odom.pose.pose.orientation.z = q_pub.z();
    odom.pose.pose.orientation.w = q_pub.w();
    odom.twist.twist.linear.x = v_pub(0);
    odom.twist.twist.linear.y = v_pub(1);
    odom.twist.twist.linear.z = v_pub(2);
    pub_odom_.publish(odom);


}


std::tuple<Eigen::Vector3f, Eigen::Vector3f, Eigen::Quaternionf> ImuIntegrationNodelet::imuIntegrate(const std::vector<sensor_msgs::Imu::ConstPtr>& imu_vector) const
{

    int num_measurement = imu_vector.size();
    Eigen::Vector3f v_proc = v_vio_;  // states during integration process
    Eigen::Vector3f p_proc = p_vio_;
    Eigen::Quaternionf q_proc = q_vio_;

    // not to use the last element
    for (int i=0;i<num_measurement-1;i++)
    {
        //std::cout<<"measurement "<<i<<": \n";
        Eigen::Vector3f w(imu_vector[i]->angular_velocity.x,
                          imu_vector[i]->angular_velocity.y,
                          imu_vector[i]->angular_velocity.z);  // angular velocity
        Eigen::Vector3f a(imu_vector[i]->linear_acceleration.x,
                          imu_vector[i]->linear_acceleration.y,
                          imu_vector[i]->linear_acceleration.z);  // linear acceleration
        /* remove bias */
        w = w - b_w_;
        a = a - b_a_;
        /* obtain delta t */
        const float delta_t = (imu_vector[i+1]->header.stamp - imu_vector[i]->header.stamp).toSec();

        /* update state */
        const Eigen::AngleAxisf aa_inc(w.norm() * delta_t, w.normalized());
        const Eigen::Quaternionf q_inc(aa_inc);
        p_proc = p_proc + v_proc*delta_t + 0.5*g_*delta_t*delta_t + 0.5*q_proc.toRotationMatrix()*(a*delta_t*delta_t);
        v_proc = v_proc + g_*delta_t + q_proc.toRotationMatrix()*(a*delta_t);
        q_proc = q_proc * q_inc;
    }

    return std::make_tuple(p_proc, v_proc, q_proc);
}



}

PLUGINLIB_EXPORT_CLASS(sdd_vio::ImuIntegrationNodelet, nodelet::Nodelet);

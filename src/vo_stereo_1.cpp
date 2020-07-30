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


#include "sdd_vio/vo_stereo.h"

#include <iostream>
#include "sdd_vio/utils/math_utils.h"
#include <algorithm>
#include <math.h>
#include <ros/ros.h>
#include <cmath>
#include <assert.h>
#include <boost/thread.hpp>
#include "sdd_vio/utils/ros_params_helper.h"
#include "sdd_vio/utils/timer.hpp"
#include <ctime>
#include <opencv2/core/eigen.hpp>
#include "sdd_vio/utils/calib_utils.h"

namespace sdd_vio {

    VoStereo::VoStereo(sdd_vio::PinholeCameraStereo *cam, const std::string& ns, const ros::NodeHandle &nh) :
        cam_(cam)
    {
        ROS_DEBUG_STREAM("init VoStereo object...");

        /* general settings */
        use_opt_vis_gb_ = getParam<bool>(ns+"/vo/use_opt_vis_gb");
        pub_image_debug_ = getParam<bool>(ns+"/vo/pub_image_debug");
        use_thread_ = getParam<bool>(ns+"/vo/use_thread");
        use_ica_ = getParam<bool>(ns+"/vo/use_ica");
        use_lma_ = getParam<bool>(ns+"/vo/use_lma");
        use_huber_ = getParam<bool>(ns+"/vo/use_huber");
        use_weights_ = getParam<bool>(ns+"/vo/use_weights");
        c_= getParam<float>(ns+"/vo/c");
        /* layer settings */
        full_layer_ = getParam<int>(ns+"/vo/full_layer");
        base_layer_ = getParam<int>(ns+"/vo/base_layer");
        /* feature point selection settings */
        disp_range_ = getParam<float>(ns+"/vo/disp_range");
        gd_size_ = getParam<int>(ns+"/vo/gd_size");
        use_gd_ = getParam<bool>(ns+"/vo/use_gd");
        /* keyframe switching settings */
        switch_count_ = getParam<int>(ns+"/vo/switch_count");
        switch_perc_ = getParam<float>(ns+"/vo/switch_perc");
        switch_depth_ = getParam<float>(ns+"/vo/switch_depth");
        switch_angle_ = getParam<float>(ns+"/vo/switch_angle");
        /* optimization settings */
        max_num_iter_ = getParam<int>(ns+"/vo/max_num_iter");
        verbose_ = getParam<bool>(ns+"/vo/verbose");
        lambda_ = getParam<float>(ns+"/vo/lambda");
        up_factor_ = getParam<float>(ns+"/vo/up_factor");
        down_factor_ = getParam<float>(ns+"/vo/down_factor");
        error_scale_factor_ = getParam<float>(ns+"/vo/error_scale_factor");
        target_derr_ = getParam<float>(ns+"/vo/target_derr");
        /* feature extraction settings */
        adapt_size_ = getParam<int>(ns+"/vo/adapt_size");
        adapt_thresh_ = getParam<int>(ns+"/vo/adapt_thresh");
        feat_margin_ = getParam<int>(ns+"/vo/feat_margin");
        /* visualization of points layers */
        visual_pts_layer_ = getParam<int>(ns+"/vo/visual_pts_layer");
        visual_image_layer_ = getParam<int>(ns+"/vo/visual_image_layer");
        /* imu coupling */
        alpha_ = getParam<float>(ns+"/vo/alpha");
        beta_ = getParam<float>(ns+"/vo/beta");
        float bias_a1 = getParam<float>(ns+"/vo/ba1");
        float bias_a2 = getParam<float>(ns+"/vo/ba2");
        float bias_a3 = getParam<float>(ns+"/vo/ba3");


        /* IMU Biases */
        b_w_ << 0,0,0;
        b_a_ << bias_a1, bias_a2, bias_a3;

        /* gravity - in IMU frame */
        g_ << 0,0,-9.8;

        /* set initial poses */
        T_ = Eigen::Isometry3f::Identity();
        T_kf_ = Eigen::Isometry3f::Identity();
        T_curr_ = Eigen::Isometry3f::Identity();


        // find transformation from camera to imu
        cv::Mat T_bc_inv_cv = utils::getTransformCV(nh, "cam0/T_cam_imu");
        cv::cv2eigen(T_bc_inv_cv, T_bc_inv_.matrix());
        T_bc_ = T_bc_inv_.inverse();

        /* initialize first state */
        T_wb_.setIdentity();
        T_wb_last_.setIdentity();
        v_last_.setZero();
        v_.setZero();

        /* set camera related stuff */
        frameSize_x_ = cam_->getWidth();
        frameSize_y_ = cam_->getHeight();
        focal_ = float(cam_->getf());


        /* set initial condition for member variables */
        stage_ = STAGE_PAUSED;
        set_reset_ = false;
        set_start_ = false;
        paused_ = true;
        vis_cam_avail_ = false;
        vis_pts_avail_ = false;
        frame_count_ = 0;
        perc_ = 100;
        avg_depth_ = 0;
        tracking_lost_ = false;
        layer_chop_ = false;
        num_feat_pts_.clear();

        nlays_ = full_layer_-base_layer_+1;
        assert(base_layer_ <= full_layer_);


        /* set member variables size */
        feat_pixels_.resize(nlays_);
        feat_3D_.resize(nlays_);
        intensities_.resize(nlays_);
        Gx_.resize(nlays_);
        Gy_.resize(nlays_);
        num_feat_pts_.resize(nlays_);
        error_.resize(nlays_);
        variance_.resize(nlays_);
        mask_.resize(nlays_);
        J_.resize(nlays_);

        disp_range_pyr_.resize(nlays_);
        frameSize_x_pyr_.resize(nlays_);
        frameSize_y_pyr_.resize(nlays_);
        focal_pyr_.resize(nlays_);


        /* set pyramid and grid */
        gd_.resize(nlays_);
        cam_->initPyramid(nlays_, base_layer_);
        cam_->configPyramid();
        setPyrScale();
        setGD();

        // !!NOTE:
        // number ilay starts from zero, the same size as nlays_, make sure ilay=0 corresponds to 400 pixel level pyramid.
        // Otherwise modify thread functions with the absolute ilay number passing in.

    }

    VoStereo::~VoStereo()
    {
        std::cout << "Destroying VoStereo object..." << std::endl;
        threadPrepImg0_.join();
        threadPrepImg1_.join();
        thread_pyr2_.join();
        thread_pyr3_.join();
        thread_pyr4_.join();
        thread_pyr1_disp_.join();
        thread_pyr1_feat_.join();
        thread_tracking_.join();
        std::cout << "All threads joined in vo." << std::endl;

        for (int i=0; i<nlays_; ++i) {
            if (gd_[i]!= NULL)
                delete gd_[i];
        }
    }


 /*
  * When sdd_vio_nodelet is stopped, vo_->reset() is called.
  * This will call adImagePair one more time to obtain the statistics.
  */
    void VoStereo::reset()
    {
        set_reset_ = true;
        set_start_ = false;

        threadPrepImg0_.join();
        threadPrepImg1_.join();
        thread_pyr2_.join();
        thread_pyr3_.join();
        thread_pyr4_.join();
        thread_pyr1_disp_.join();
        thread_pyr1_feat_.join();
        thread_tracking_.join();
        std::cout << "All threads joined in vo from reset." << std::endl;

        /* calling function with empty params to show statistics */
        cv::Mat img0, img1;
        double timestamp = 0;
        double seq = 0;
        addImagePair(img0, img1, timestamp, seq);

        std::cout << "vo reset finished!" << std::endl;

    }


    void VoStereo::addImagePair(const cv::Mat& img0, const cv::Mat& img1, double timestamp, double seq)
    {
        ROS_DEBUG_STREAM("addImagePair entered.");

        /* for statistics collection */
        static int keyframe_count = 0;      // total number of keyframes
        static int nextframe_count = 0;     // total number of normal frames
        static int totalframe_count = 0;    // total number of all frame
        static int num_points_largest_layer = 0;  // total number of points tracked


        if (set_reset_==true) {
            ROS_INFO_STREAM("*** RESET! -> STAGE_PAUSED");
            stage_ = STAGE_PAUSED;
            paused_ = true;
            set_reset_ = false;
        }

        vis_cam_avail_ = false;
        vis_pts_avail_ = false;


        if (stage_==STAGE_PAUSED) {
            if (set_start_==true)
            {
                ROS_INFO_STREAM("*** START! first frame received. STAGE_PAUSED");
                ROS_DEBUG_STREAM("add-ImagePair: getting image layers...");
                std::vector<cv::Mat> img0_ready(full_layer_), img1_ready(full_layer_);
                if (use_thread_ == false)
                {
                    cv::GaussianBlur(img0, img0_ready[0],cv::Size(3,3),0,0);
                    cv::GaussianBlur(img1, img1_ready[0],cv::Size(3,3),0,0);

                    for (int i=1; i<full_layer_; ++i) {
                        cv::pyrDown(img0_ready[i-1], img0_ready[i], cv::Size(int(img0_ready[i-1].cols/2), int(img0_ready[i-1].rows/2)));
                        cv::pyrDown(img1_ready[i-1], img1_ready[i], cv::Size(int(img1_ready[i-1].cols/2), int(img1_ready[i-1].rows/2)));
                    }
                }
                else
                {
                    threadPrepImg0_ = boost::thread(&VoStereo::threadCreatePyr, this, boost::ref(img0), boost::ref(img0_ready));
                    threadPrepImg1_ = boost::thread(&VoStereo::threadCreatePyr, this, boost::ref(img1), boost::ref(img1_ready));
                    threadPrepImg0_.join();
                    threadPrepImg1_.join();
                }

                /* ====== two image pipeline ====== */
                ROS_DEBUG_STREAM("addImagePair: calling twoImagePipeline...");
                twoImagePipeline(img0_ready, img1_ready);

                /* for statistics */
                keyframe_count++;
                totalframe_count++;
                num_points_largest_layer+=num_feat_pts_[0];

                /* for state machine */
                stage_ = STAGE_NEXT_FRAME;  // in the first-frame stage
                vis_cam_avail_ = true;
                vis_pts_avail_ = true;
                set_start_ = false;
                frame_count_ = 1;
                lost_frames_ = 0;
            }
            else
            {
                ROS_INFO_STREAM("*** STAGE_PAUSED ");
                std::cout << "$$$$$$$$$ -------- STATISTICS -------- $$$$$$$$$" << std::endl;
                std::cout << "Total number of keyframes: " << keyframe_count << std::endl;
                std::cout << "Total number of frames: " << totalframe_count << std::endl;
                std::cout << "Average number of points tracked in largest pyramid level: " <<
                             num_points_largest_layer/keyframe_count << std::endl;
                return;
            }

        }
        else if (stage_==STAGE_NEXT_FRAME) {
            ROS_INFO_STREAM("*** new frame received. STAGE_NEXT_FRAME");
            static kr::Timer<kr::us> timer_nextframe("nextframe_timer");
            timer_nextframe.Start();

            /* ====== get layers of images ====== */
            ROS_DEBUG_STREAM("add-ImagePair: getting image layers...");
            std::vector<cv::Mat> img0_ready(full_layer_);
            cv::GaussianBlur(img0, img0_ready[0],cv::Size(3,3),0,0);
            for (int i=1; i<full_layer_; ++i) {
                cv::pyrDown(img0_ready[i-1], img0_ready[i], cv::Size(int(img0_ready[i-1].cols/2), int(img0_ready[i-1].rows/2)));
            }

            ROS_DEBUG_STREAM("add-ImagePair: setting visualization image...");
            if (use_opt_vis_gb_)
                cv::cvtColor(img0_ready[base_layer_-1+visual_image_layer_],vis_curr,CV_GRAY2RGB);

            /* ====== one image pipeline ====== */
            ROS_DEBUG_STREAM("add-ImagePair: calling one-ImagePipeline...");
            oneImagePipeline(img0_ready);

            /* for statistics */
            nextframe_count++;
            totalframe_count++;

            /* for state machine */
            frame_count_++;
            // std::cout << "norm of translation since last keyframe: " << t_curr_.norm() << std::endl;
            const float rot_curr = log_SO3(T_curr_.rotation()).norm();  // norm of angle
            const Eigen::Vector3f t_curr = T_curr_.translation();
            if (perc_ < switch_perc_ || frame_count_ > switch_count_ || rot_curr > switch_angle_ ||
                    t_curr.norm() > switch_depth_*avg_depth_ || tracking_lost_ == true) {
                if (perc_ < switch_perc_)
                    ROS_INFO_STREAM("New KF for percentage of points visible falls below "<<switch_perc_<<"%");
                if (frame_count_ > switch_count_)
                    ROS_INFO_STREAM("New KF for number of frames exceeds "<<switch_count_);
                if (rot_curr > switch_angle_)
                    ROS_INFO_STREAM("New KF for rotation greater than threshold "<<switch_angle_);
                if (t_curr.norm() > 0.1*avg_depth_)
                    ROS_INFO_STREAM("New KF for translation over 0.1* average scene depth: "<<0.1*avg_depth_);
                if (tracking_lost_ == true)
                    ROS_INFO_STREAM("New KF for tracking lost");

                stage_ = STAGE_NEW_KEYFRAME;
                tracking_lost_ = false;
            }
            vis_cam_avail_ = true;

            timer_nextframe.Stop();
            timer_nextframe.Report();

        }
        else if (stage_ == STAGE_NEW_KEYFRAME) {
            ROS_INFO_STREAM("*** new keyframe received. STAGE_NEXT_FRAME -> STAGE_NEW_KEYFRAME");
            static kr::Timer<kr::us> timer_keyframe("keyframe_timer");
            timer_keyframe.Start();

            ROS_DEBUG_STREAM("add-ImagePair: getting image layers...");
            std::vector<cv::Mat> img0_ready(full_layer_), img1_ready(full_layer_);
            if (use_thread_ == false)
            {
                cv::GaussianBlur(img0, img0_ready[0],cv::Size(3,3),0,0);
                cv::GaussianBlur(img1, img1_ready[0],cv::Size(3,3),0,0);
                for (int i=1; i<full_layer_; ++i) {
                    cv::pyrDown(img0_ready[i-1], img0_ready[i], cv::Size(int(img0_ready[i-1].cols/2), int(img0_ready[i-1].rows/2)));
                    cv::pyrDown(img1_ready[i-1], img1_ready[i], cv::Size(int(img1_ready[i-1].cols/2), int(img1_ready[i-1].rows/2)));
                }
            }
            else
            {
                threadPrepImg0_ = boost::thread(&VoStereo::threadCreatePyr, this, boost::ref(img0), boost::ref(img0_ready));
                threadPrepImg1_ = boost::thread(&VoStereo::threadCreatePyr, this, boost::ref(img1), boost::ref(img1_ready));
                threadPrepImg0_.join();
                threadPrepImg1_.join();
            }

            ROS_DEBUG_STREAM("add-ImagePair: setting visualization image...");
            if (use_opt_vis_gb_)
                cv::cvtColor(img0_ready[base_layer_-1+visual_image_layer_],vis_curr,CV_GRAY2RGB);

            /* ====== two image pipeline ====== */
            ROS_DEBUG_STREAM("add-ImagePair: calling two-ImagePipeline...");
            twoImagePipeline(img0_ready, img1_ready);

            /* for statistics */
            keyframe_count++;
            totalframe_count++;
            num_points_largest_layer+=num_feat_pts_[0];

            /* for state machine */
            stage_ = STAGE_NEXT_FRAME;  // in the first-frame stage
            vis_cam_avail_ = true;
            vis_pts_avail_ = true;
            frame_count_ = 1;
            lost_frames_ = 0;

            timer_keyframe.Stop();
            timer_keyframe.Report();
        }

    }  // void VoStereo::addImagePair


    /* transformation of 3D points from one frame to another */
    // Input: points_f1, T_21, npts
    // Output: points_f2
    // points_f2 = T_21 * points_f1;
    void VoStereo::transform3D(const vector_aligned<Eigen::Vector3f>& points_f1, vector_aligned<Eigen::Vector3f>& points_f2,
            const Eigen::Isometry3f& T_21, const int npts)
    {
        points_f2.resize(npts);
        for (int i=0; i<npts; ++i)
        {
            Eigen::Vector4f p1 = sdd_vio::unproject3d(points_f1[i]);
            Eigen::Vector4f p2 = T_21 * p1;
            points_f2[i] = sdd_vio::project3d(p2);
        }
    }


    void VoStereo::updatePose()
    {
        /* update previous state */
        T_wb_last_ = T_bc_ * T_ * T_bc_inv_;
        /* update current pose (camera track) */
        T_ = T_kf_ * (T_curr_.inverse());
        /* update current state */
        T_wb_ = T_bc_ * T_ * T_bc_inv_;
    }


    void VoStereo::setKfPose()
    {
        /* copy current pose to keyframe pose */
        T_kf_ = T_;

        /* reinitialize current to keyframe pose */
        T_curr_ = Eigen::Isometry3f::Identity();

    }


    vector_aligned<Eigen::Vector3f> VoStereo::getVisPoints()
    {
        vector_aligned<Eigen::Vector3f> feat_3D_world;  // the points expressed in the first camera frame
        Eigen::Isometry3f T;
        T = T_bc_ * T_;

        /* for visualization only use the bottom layer of 3D points */
        transform3D(feat_3D_[visual_pts_layer_], feat_3D_world, T, feat_3D_[visual_pts_layer_].size());  // transform to camera frame

        return feat_3D_world;
    }


    void VoStereo::setPyrScale()
    {
        float scale;
        for (int i=0; i<nlays_; ++i) {
            scale = pow(0.5,(base_layer_-1+i));
            frameSize_x_pyr_[i] = scale * frameSize_x_;
            frameSize_y_pyr_[i] = scale * frameSize_y_;
            focal_pyr_[i] = scale * focal_;
            disp_range_pyr_[i] = scale * disp_range_;
        }
    }

    void VoStereo::setGD()
    {
        for (int i=0; i<nlays_; ++i) {
            gd_[i] = new Grid(frameSize_y_pyr_[i], frameSize_x_pyr_[i], gd_size_, gd_size_);
        }
    }

    void VoStereo::importImuMeas(const Eigen::Matrix3f& R_meas, const Eigen::Vector3f& v_meas, const Eigen::Vector3f& p_meas, const float t_meas, const Eigen::Matrix<float,9,9>& Cov_meas)
    {
        R_meas_ = R_meas;
        v_meas_ = v_meas;
        p_meas_ = p_meas;
        t_meas_ = t_meas;
        Cov_meas_ = Cov_meas;
    }

    void VoStereo::setImuBiasGyro(const Eigen::Vector3f& w_b)
    {
        b_w_ = w_b;
    }

    void VoStereo::setImuBiasAccel(const Eigen::Vector3f& a_b)
    {
        b_a_ = a_b;
    }


}  // namespace sdd_vio

/*-------------------------------------------------------------
Copyright 2019 Wenxin Liu, Kartik Mohta, Giuseppe Loianno

licensed under the apache license, version 2.0 (the "license");
you may not use this file except in compliance with the license.
you may obtain a copy of the license at

    http://www.apache.org/licenses/license-2.0

unless required by applicable law or agreed to in writing, software
distributed under the license is distributed on an "as is" basis,
without warranties or conditions of any kind, either express or implied.
see the license for the specific language governing permissions and
limitations under the license.
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


namespace sdd_vio {

/*
 * Pipeline for tracking -
 * Take in a pyramid of images and perform coarse-to-fine optimization between
 * last keyframe and current frame. Update global pose.
 *
 * Keeping SE3 T for compatibility to the code written before. Ceres uses only R and t.
 */
    void VoStereo::oneImagePipeline(const std::vector<cv::Mat>& img0)
    {
        ROS_DEBUG_STREAM("one-ImagePipeline: ----------- ");

        /* an iterator on all layers participating in tracking, coarse to fine order */
        int itop = nlays_-1;  // top layer index
        if (layer_chop_ == true) {
            itop = nlays_chopped_-1;
        }

        v_last_ = v_;
        T_wb_last_ = T_wb_;

        // change initial guess from IMU integration
        T_wb_.linear() = T_wb_last_.rotation() * R_meas_;
        v_ = T_wb_last_.rotation() * v_meas_ + v_last_ + g_ * t_meas_;
        T_wb_.translation() = T_wb_last_.rotation() * p_meas_ + T_wb_last_.translation() +
                              v_last_ * t_meas_ + 0.5 * g_ * t_meas_ * t_meas_;
        T_ = T_bc_inv_ * T_wb_ * T_bc_;
        T_curr_ = T_.inverse()*T_kf_;

        /* Do optimization for all pyramid layers, coarse to fine */
        for (int i=itop; i>=0; --i) {

          /* enable convergence visualization only for the specific visual layer specified in config file */
          use_opt_vis_ = (use_opt_vis_gb_ == true) ? ((i==visual_image_layer_) ? true : false) : false;

          if (use_ica_) //Inverse compositional approach
          {
            optimize(img0[base_layer_+i-1], J_[i], T_curr_, error_[i], variance_[i], feat_pixels_[i], feat_3D_[i],
                intensities_[i], mask_[i], num_feat_pts_[i], max_num_iter_, i);
            updatePose();  // vision only, so need to propagate T_curr changes to T and T_wb
          }
          else  // forward compositional approach
            optimize_fca(img0[base_layer_+i-1], T_curr_, T_wb_, T_, v_, i);  // T_curr, T, T_wb all changes in optimize_fca

          // ROS_INFO_STREAM("v_optimized: " << v_.transpose());
          // v_.setZero(); // currently set to zero to prevent drifting
          // v_ = (T_wb_.block<3,1>(0,3) - T_wb_last_.block<3,1>(0,3))/t_meas_;
        }

    }


    /* Pipeline for new keyframe */
    void VoStereo::twoImagePipeline(const std::vector<cv::Mat>& img0, const std::vector<cv::Mat>& img1)
    {

        ROS_DEBUG_STREAM("two-ImagePipeline entered.");

        layer_chop_ = false;  // first assume no layer chop needed for this keyframe

        /* declaring disp related vectors */
        std::vector<cv::Mat> disp(nlays_), G_binary(nlays_), Gx_mat(nlays_), Gy_mat(nlays_);
        std::vector< std::vector<float> > disp_vec(nlays_);

        /* obtain disp, G_binary, Gx_mat, Gy_mat and perform tracking */
        if (use_thread_ == true)
        {
            disp.clear();
            G_binary.clear();
            Gx_mat.clear();
            Gy_mat.clear();

            /* temporary vectors */
            cv::Mat disp_pyr1, disp_pyr2, disp_pyr3, disp_pyr4;
            cv::Mat G_binary_pyr1, G_binary_pyr2, G_binary_pyr3, G_binary_pyr4;
            cv::Mat Gx_mat_pyr1, Gx_mat_pyr2, Gx_mat_pyr3, Gx_mat_pyr4;
            cv::Mat Gy_mat_pyr1, Gy_mat_pyr2, Gy_mat_pyr3, Gy_mat_pyr4;

            /*
             * spawning threads -
             * only the largest layer gets disparity and feature extraction separated into two threads
             * all other smaller pyramid images combine them into one thread.
             */
            assert(nlays_==1 || nlays_==2 || nlays_==3 || nlays_==4);

            switch (nlays_) {
            /* switch on the number of layers, default is 1. any upper layers will spawn all the lower layer threads */
            case 4: {
                thread_pyr4_ = boost::thread(&VoStereo::threadFuncComb, this, boost::ref(img0[base_layer_+2]), boost::ref(img1[base_layer_+2]),
                        boost::ref(G_binary_pyr4), boost::ref(disp_pyr4), boost::ref(Gx_mat_pyr4), boost::ref(Gy_mat_pyr4), 3);
            }
            case 3: {
                thread_pyr3_ = boost::thread(&VoStereo::threadFuncComb, this, boost::ref(img0[base_layer_+1]), boost::ref(img1[base_layer_+1]),
                        boost::ref(G_binary_pyr3), boost::ref(disp_pyr3), boost::ref(Gx_mat_pyr3), boost::ref(Gy_mat_pyr3), 2);
            }
            case 2: {
                thread_pyr2_ = boost::thread(&VoStereo::threadFuncComb, this, boost::cref(img0[base_layer_]), boost::cref(img1[base_layer_]),
                        boost::ref(G_binary_pyr2), boost::ref(disp_pyr2), boost::ref(Gx_mat_pyr2), boost::ref(Gy_mat_pyr2), 1);
            }
            case 1: {
                thread_pyr1_disp_ = boost::thread(&VoStereo::threadFunc_disp, this,
                        boost::ref(img0[base_layer_-1]), boost::ref(img1[base_layer_-1]), boost::ref(disp_pyr1));
                thread_pyr1_feat_ = boost::thread(&VoStereo::threadFunc_feat, this,
                        boost::ref(img0[base_layer_-1]), boost::ref(G_binary_pyr1), boost::ref(Gx_mat_pyr1), boost::ref(Gy_mat_pyr1));
                thread_tracking_ = boost::thread(&VoStereo::threadOneImagePipeline, this, boost::cref(img0));
            }
            }

            /*
             * join threads -
             */
            switch (nlays_) {
            case 4:
                thread_pyr4_.join();
            case 3:
                thread_pyr3_.join();
            case 2:
                thread_pyr2_.join();
            case 1:
                thread_pyr1_disp_.join();
                thread_pyr1_feat_.join();
                thread_tracking_.join();
            }

            /*
             * push temporary vectors to general vectors, need to push bottom up the pyramid
             */
            // TODO - in FCA we don't need to get Gx and Gy
            disp.push_back(disp_pyr1);
            G_binary.push_back(G_binary_pyr1);
            Gx_mat.push_back(Gx_mat_pyr1);
            Gy_mat.push_back(Gy_mat_pyr1);
            if (nlays_ > 1) {
                disp.push_back(disp_pyr2);
                G_binary.push_back(G_binary_pyr2);
                Gx_mat.push_back(Gx_mat_pyr2);
                Gy_mat.push_back(Gy_mat_pyr2);
            }
            if (nlays_ > 2) {
                disp.push_back(disp_pyr3);
                G_binary.push_back(G_binary_pyr3);
                Gx_mat.push_back(Gx_mat_pyr3);
                Gy_mat.push_back(Gy_mat_pyr3);
            }
            if (nlays_ > 3) {
                disp.push_back(disp_pyr4);
                G_binary.push_back(G_binary_pyr4);
                Gx_mat.push_back(Gx_mat_pyr4);
                Gy_mat.push_back(Gy_mat_pyr4);
            }

        }
        else //if (use_thread_ == false)
        {
            /* tracking */
            if (stage_!=STAGE_PAUSED) {
                ROS_DEBUG_STREAM("two-ImagePipeline: tracking...");
                oneImagePipeline(img0);
            }

            disp.clear();
            G_binary.clear();
            Gx_mat.clear();
            Gy_mat.clear();

            for (int i=0; i<nlays_; ++i)
            {
                ROS_DEBUG_STREAM("two-ImagePipeline: not using thread. getting disparity...");
                cam_->getDisparityMap(img0[base_layer_+i-1], img1[base_layer_+i-1], disp[i]);

                // showMatProp(disp_[i]);   // disp type: 8UC1
                ROS_DEBUG_STREAM("two-ImagePipeline: extracting features...");
                extractFeats(img0[base_layer_+i-1], G_binary[i], Gx_mat[i], Gy_mat[i], i);
            }

        }

        /* obtain vectors for each layer. If use inverse comositional approach, calculate Jacobian */
        for (int i=0; i<nlays_; ++i)
        {
            /* here Gx_ is the vector, Gx_mat_ is the matrix, both member variables */
            // TODO - in FCA we don't need to get Gx and Gy
            fillVectors(img0[base_layer_+i-1], G_binary[i], disp[i], Gx_mat[i], Gy_mat[i], disp_vec[i], feat_pixels_[i],
                    intensities_[i], Gx_[i], Gy_[i], num_feat_pts_[i], i);

            cam_->get3DPoints(feat_pixels_[i], disp_vec[i], feat_3D_[i], num_feat_pts_[i], i);
            ROS_INFO_STREAM("number of feature points: " << num_feat_pts_[i]);

            if (use_ica_ == true) {
                /* get jacobian J_ for feat_3D */
                getJacobian(Gx_[i], Gy_[i], focal_pyr_[i], focal_pyr_[i], feat_3D_[i], J_[i], num_feat_pts_[i]);

                /* initialize error and variance vector */
                error_[i] = Eigen::VectorXf::Constant(num_feat_pts_[i],0);
                variance_[i] = Eigen::VectorXf::Constant(num_feat_pts_[i],1);
                mask_[i] = Eigen::VectorXf::Constant(num_feat_pts_[i],1);
            }
        }


        /* getting average scene depth - use the top layer */
        float sum_depth = 0;
        for (int i=0; i<num_feat_pts_[nlays_-1]; ++i) {
            sum_depth = sum_depth + feat_3D_[nlays_-1][i](2);
        }
        avg_depth_ = sum_depth/num_feat_pts_[nlays_-1];

        /* check if number of points decrease as layer up */
        for (int i=0; i<nlays_; ++i) {
            if ((i-1)>=0) { // if there is a layer before this layer
                if (num_feat_pts_[i-1] < num_feat_pts_[i])
                {
                    ROS_WARN_STREAM("unreasonable feature point layout in pyramid!");
                    ROS_WARN_STREAM("number of feature points in layer " << i << ": " << num_feat_pts_[i]);
                    ROS_WARN_STREAM("number of feature points in layer " << i-1 << ": " << num_feat_pts_[i-1]);
                }
            }
        }

        /* detect layer chop */
        for (int i=0; i<nlays_; ++i) {
            if (num_feat_pts_[i] < 20) { // number of points too small
                layer_chop_ = true;
                nlays_chopped_ = i;
                ROS_INFO_STREAM("Layer chop set to true. New number of layers for current frame: " << nlays_chopped_);
                if (i==0)
                    ROS_WARN_STREAM("Chopped number of layers is 0! No tracking being done.");
            }
        }

    }


    void VoStereo::threadCreatePyr(const cv::Mat& img, std::vector<cv::Mat>& img_ready)
    {
        ROS_DEBUG_STREAM("threadCreatePyr entered.");
        cv::GaussianBlur(img, img_ready[0],cv::Size(3,3),0,0);
        for (int i=1; i<full_layer_; ++i) {
            cv::pyrDown(img_ready[i-1], img_ready[i], cv::Size(int(img_ready[i-1].cols/2), int(img_ready[i-1].rows/2)));
        }
    }


    /* disparity thread - used on the largest image in the pyramid */
    void VoStereo::threadFunc_disp(const cv::Mat& img0, const cv::Mat& img1, cv::Mat& disp)
    {
        static kr::Timer<kr::us> timer("disp");
        timer.Start();
        ROS_DEBUG_STREAM("thread-Func-Disp: getting disparity maps...");
        cam_->getDisparityMap(img0, img1, disp);

        if (pub_image_debug_) {
            ROS_DEBUG_STREAM("thread-Func-Disp: setting disparity visualization...");
            cv::ximgproc::getDisparityVis(disp, visual1);
        }
        timer.Stop();
        timer.Report();
    }

    /* feature extraction thread - used on the largest image in the pyramid */
    void VoStereo::threadFunc_feat(const cv::Mat& img0, cv::Mat& G_binary, cv::Mat& Gx_mat, cv::Mat& Gy_mat)
    {
        static kr::Timer<kr::us> timer("feat");
        timer.Start();
        ROS_DEBUG_STREAM("thread-Func-Feat: extracting features...");
        extractFeats(img0, G_binary, Gx_mat, Gy_mat, 0);

        if (pub_image_debug_) {
            visual2.setTo(0);
            visual2 = G_binary;
        }
        timer.Stop();
        timer.Report();
    }


    /* having disparity and feature extraction in one thread */
    void VoStereo::threadFuncComb(const cv::Mat& img0, const cv::Mat& img1, cv::Mat& G_binary,
            cv::Mat& disp, cv::Mat& Gx_mat, cv::Mat& Gy_mat, int ilay)
    {
        cam_->getDisparityMap(img0, img1, disp);
        extractFeats(img0, G_binary, Gx_mat, Gy_mat, ilay);
    }


    void VoStereo::threadOneImagePipeline(const std::vector<cv::Mat>& img0)
    {
        static kr::Timer<kr::us> timer("tracking");
        if (stage_!=STAGE_PAUSED)
        {
            timer.Start();
            ROS_DEBUG_STREAM("thread-One-ImagePipeline: calling one-ImagePipeline...");
            oneImagePipeline(img0);
            ROS_DEBUG_STREAM("thread-One-ImagePipeline: updating keyframe poses...");
            setKfPose();
            timer.Stop();
        }
        timer.Report();
    }

}

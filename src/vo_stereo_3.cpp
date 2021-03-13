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




namespace sdd_vio {

/*
 * Optimization using Forward Compositional Approach
 *
 * Input:
 *      img_curr: current frame image
 *      ilay: the index of tracking layer in the image pyramid. 0 is the largest layer for tracking.
 * Return:
 *      T_curr: current transformation matrix from keyframe to current frame
 *      T_wb: transformation between current robot frame to world frame
 *      T_: transformation between current camera frame to initial camera frame
 *      v_: velocity of current body frame
 * Use of private class variables:
 *      feat_3D_[ilay]: the vector of 3D points in keyframe
 *      intensities_[ilay]: the vector of pixel intensities in keyframe
 *      mask_[ilay]: indicator of whether points are in view
 * LMA params as private variables:
 *      lambda_
 *      up_factor_
 *      down_factor_
 *
 */
    void VoStereo::optimize_fca(const cv::Mat& img_curr, Eigen::Isometry3f& T_curr, Eigen::Isometry3f& T_wb, Eigen::Isometry3f& T, Eigen::Vector3f& v, int ilay) {


        /* obtain gradient image of the current frame */
        cv::Mat Gx_mat, Gy_mat;
        cv::Sobel(img_curr, Gx_mat, CV_32F, 1, 0, 3);
        cv::Sobel(img_curr, Gy_mat, CV_32F, 0, 1, 3);


        /* masked Jacobian matrix and error vector - remove points outside projection */
        Eigen::MatrixXf J_masked;  // every time J_masked is re-calculated for every pixel inside the view
        Eigen::VectorXf error_masked;
        Eigen::VectorXf W;  // diagonal weight matrix for Huber estimation


        /* Jacobian and error vector of IMU */
        Eigen::MatrixXf J_imu;
        Eigen::VectorXf r_imu;
        Eigen::MatrixXf Z_imu;  // inverse covariance of imu errors
        J_imu.resize(9,9);
        initJacobian_imu_fca(J_imu);
        r_imu.resize(9);
//        Eigen::VectorXf inv_cov_imu(9);
//        inv_cov_imu << weight_R_,weight_R_,weight_R_, weight_v_,weight_v_,weight_v_,weight_p_,weight_p_,weight_p_;
//        Z_imu = inv_cov_imu.asDiagonal();
        Z_imu = Cov_meas_.inverse();
//        std::cout<<"Z_imu:\n"<<Z_imu<<"\n";


        /* Levenberg Marquardt */
        float lambda = lambda_;  // initial lambda is equal to the initial one set
        float up = up_factor_;
        float down = down_factor_;
        bool if_break = false;


        int npts_in;  // number of points remained in view
        perc_ = 1.;  // percentage of points remained in view
        float err_init, err_last = 0;


        /* begins optimization iteration */
        for (int iter=0; iter<max_num_iter_; ++iter)
        {
            /* first iteration */
            if (iter == 0) {

                /* calculate error and Jacobian given initial transformation */
                getError_fca(img_curr, Gx_mat, Gy_mat, T_curr, T_wb, J_masked, error_masked, W, npts_in, ilay);
                getError_imu_fca(T_wb, v, r_imu, J_imu);


                perc_ = 100*npts_in/num_feat_pts_[ilay];
                float err_mean = get_error_mean(error_masked, r_imu);
                err_init = err_mean;
                err_last = err_mean;

            }

            using Mat9 = Eigen::Matrix<float, 9, 9>;
            using Vec9 = Eigen::Matrix<float, 9, 1>;

            if (use_lma_ == false)
            {
                /* obtain H and b */
                const Mat9 H = beta_*J_masked.transpose() * W.asDiagonal() * J_masked + alpha_ * (J_imu.transpose() * Z_imu * J_imu);  // H = J^t * W * J
                const Vec9 b = beta_*J_masked.transpose() * (W.asDiagonal() * error_masked) + alpha_ * (J_imu.transpose() * (Z_imu * r_imu));  // b = J^t * w * e

                /* get delta increment */
                const Vec9 delta = -H.ldlt().solve(b);
//                std::cout<<"delta: \n"<<delta<<"\n";

                update_optimization_step(delta, T_wb, v, T, T_curr);


                /* calculate error vector and Jacobian given transformation */
                getError_fca(img_curr, Gx_mat, Gy_mat, T_curr, T_wb, J_masked, error_masked, W, npts_in, ilay);
                getError_imu_fca(T_wb, v, r_imu, J_imu);
//                std::cout << "r_imu: \n"<<r_imu<<"\n";


                perc_ = 100*npts_in/num_feat_pts_[ilay];
                float err_mean = get_error_mean(error_masked, r_imu);
                float err_diff = (err_mean - err_last)/err_last;
                err_last = err_mean;


                /* terminal output */
                if (verbose_) {
                    std::cout << "iteration = "<<iter<<", err = "<< err_mean<<", derr = "<<err_diff<<std::endl;
                }

                /* Stop condition - if error is going up or decreased below target or reach max iterations */
                if (err_diff>0 || -err_diff < target_derr_ || iter == max_num_iter_-1)
                    if_break = true;

                /* end of optimization */
                if (if_break) {
                    ROS_INFO_STREAM("iteration "<<iter<<"--------");
                    ROS_INFO("err/initial err: %g/%g", err_mean, err_init);
                    if (err_mean > err_init)
                      ROS_WARN_STREAM("bad convergence - err/initial err: "<<err_mean<<"/"<<err_init<<", iteration"<<iter);
                    break;
                }

            }
            else  // use lma
            {

                /* temporary T_curr, is error goes up in subloop can switch back */
                Eigen::Isometry3f T_curr_temp;
                Eigen::Isometry3f T_wb_temp;
                Eigen::Isometry3f T_temp;
                Eigen::Vector3f v_temp;


                /* LMA */
                float mult = lambda;
                bool ill = true;  // if ill conditioned (error increase)
                int lmit = 0;  // number of iterations while ill=true

                /* obtain H and b */
                const Mat9 H = beta_* J_masked.transpose() * W.asDiagonal() * J_masked + alpha_ * (J_imu.transpose() * Z_imu * J_imu);  // H = J^t * W * J
                const Vec9 b = beta_* J_masked.transpose() * (W.asDiagonal() * error_masked) + alpha_ * (J_imu.transpose() * (Z_imu * r_imu));  // b = J^t * w * e

                float err_mean = 0, err_diff = 0;
                /* LMA wrapper of subloop */
                while ((ill==true) && (iter<max_num_iter_) && (lmit<5))
                {

                    /* lma update on H */
                    const Mat9 H_curr = H + Mat9(mult * H.diagonal().asDiagonal());


                    /* get delta increment */
                    const Vec9 delta = -H_curr.ldlt().solve(b);
//                    std::cout<<"delta: \n"<<delta<<"\n";


                    T_curr_temp = T_curr;
                    T_wb_temp = T_wb;
                    T_temp = T;
                    v_temp = v;
                    update_optimization_step(delta, T_wb_temp, v_temp, T_temp, T_curr_temp);


                    /* calculate error vector and Jacobian given transformation */
                    getError_fca(img_curr, Gx_mat, Gy_mat, T_curr_temp, T_wb_temp, J_masked, error_masked, W, npts_in, ilay);
                    getError_imu_fca(T_wb, v_temp, r_imu, J_imu);
//                    std::cout<<"J_imu: \n"<<J_imu<<"\n";
//                    std::cout << "r_imu: \n"<<r_imu<<"\n";


                    perc_ = 100*npts_in/num_feat_pts_[ilay];
                    err_mean = get_error_mean(error_masked, r_imu);
                    err_diff = (err_mean - err_last)/err_last;

                    ill = (err_diff > 0);  // if error larger than before, set ill = true, try increase lambda.

                    /* terminal output */
                    if (verbose_) {
                        std::cout << "iteration = "<<iter<<", lambda = "<<lambda<<", err = "<<
                            err_mean<<", derr = "<<err_diff<<std::endl;
                    }

                    /* if ill=true, higher lambda by up factor, count as one iteration */
                    if (ill) {
                          lambda *= up;
                        mult = lambda;
                        iter++;
                    }
                    lmit++;
                }

                err_last  = err_mean;

                /* update T with the one that makes error smaller or the last try */
                T_curr = T_curr_temp;
                T = T_temp;
                T_wb = T_wb_temp;
                v = v_temp;

                /* if error doesn't get higher for quite some time, this term will bring lambda down eventually */
                if (lambda > 1e-8)  // make sure doesn't overflow
                    lambda *= (1/down);

                /* if LM iterations didn't decrease the error, stop */
                if (ill) {
                    //ROS_WARN_STREAM("bad convergence!");
                    if_break = true;
                }

                /* if error is decreased below target or reach max iterations */
                if (-err_diff < target_derr_ || iter == max_num_iter_-1)
                    if_break = true;

                /* end of optimization */
                if (if_break) {
                    ROS_INFO_STREAM("iteration "<<iter<<"--------");
                    ROS_INFO("err/initial err: %g/%g", err_mean, err_init);
                    if (err_mean > err_init)
                      ROS_WARN_STREAM("bad convergence - err/initial err: "<<err_mean<<"/"<<err_init<<", iteration"<<iter);
                    break;
                }

            }

        } // for (int iter=0;

    }


/*
 * Update optimization targets given delta
 *
 */
    void VoStereo::update_optimization_step(const Eigen::Matrix<float, 9, 1>& delta, Eigen::Isometry3f &T_wb, Eigen::Vector3f &v, Eigen::Isometry3f &T, Eigen::Isometry3f &T_curr)
    {
        /* update current transformation */
        const Eigen::Vector3f delta_phi = delta.segment<3>(0);
        const Eigen::Matrix3f R_delta = exp_SO3(delta_phi);
        const Eigen::Vector3f t_delta = delta.segment<3>(3);
        const Eigen::Vector3f v_delta = delta.segment<3>(6);

        // update T_wb and v
        T_wb.linear() = T_wb.rotation() * R_delta;
        T_wb.translation() += t_delta;
        v += v_delta;

        // update T and T_curr
        T = T_bc_inv_ * T_wb * T_bc_;
        T_curr = T.inverse() * T_kf_;
    }


/*
 * Obtain err_mean for convergence evaluation
 *
 */
    float VoStereo::get_error_mean(const Eigen::VectorXf& error_masked, const Eigen::VectorXf& r_imu)
    {
        int npts_in = error_masked.size();
        float err_photo = error_masked.transpose()*error_masked; // sum of square errors
        // these are raw square errors - not weighed by huber weights
        // photometric part is the mean pixel intensity error, plus the IMU observation square error weighted by optimization weight alpha_
        float err_mean = beta_ * sqrt(err_photo/npts_in) / error_scale_factor_ + alpha_*(r_imu.transpose()*r_imu).value();
        return err_mean;
    }



 /*
  * initialize imu jacobian with the parts that do not change in iterations
  *
  */
    void VoStereo::initJacobian_imu_fca(Eigen::MatrixXf &J_imu)
    {
        /* these are the parts that do not change through iterations */
        J_imu.setZero();
        Eigen::Matrix3f R_i = T_wb_last_.rotation();
        J_imu.block<3,3>(6,3) = R_i.transpose();
        J_imu.block<3,3>(3,6) = R_i.transpose();
    }

/*
 * obtain imu error vector and update imu jacobian
 * Input:
 *      T_wb: current robot pose in world frame
 *      v: current robot velocity in world frame
 * Output:
 *      r_imu: 9x1 vector of imu errors
 *      J_imu: 9x9 jacobian of state (phi,p,v of current robot frame in world) with respect to imu errors
 *
 */
    void VoStereo::getError_imu_fca(const Eigen::Isometry3f &T_wb, const Eigen::Vector3f &v, Eigen::VectorXf &r_imu, Eigen::MatrixXf &J_imu)
    {

        Eigen::Matrix3f R_i = T_wb_last_.rotation();  // rotation w.r.t. world frame of last body frame
        Eigen::Matrix3f R_j = T_wb.rotation();  // rotation w.r.t. world frame of current body frame
        Eigen::Vector3f p_i = T_wb_last_.translation();  // position of last robot frame in world
        Eigen::Vector3f p_j = T_wb.translation();  // position of current robot frame in world

        Eigen::Matrix3f R_err = R_meas_.transpose() * R_i.transpose() * R_j;
        Eigen::Vector3f r_R_imu = log_SO3(R_err);
        Eigen::Vector3f r_v_imu = R_i.transpose() * (v - v_last_ - g_*t_meas_) - v_meas_;
        Eigen::Vector3f r_p_imu = R_i.transpose() * (p_j - p_i - v_last_*t_meas_ - 0.5*g_*t_meas_*t_meas_) - p_meas_;

        r_imu.block<3,1>(0,0) = r_R_imu;
        r_imu.block<3,1>(3,0) = r_v_imu;
        r_imu.block<3,1>(6,0) = r_p_imu;

        J_imu.block<3,3>(0,0) = log_Jacobian(r_R_imu);

    }



/*
 * Get error, Jacobian, and huber weights for Forward Compositional Apporach.
 *
 * Input:
 *      img_curr: current frame image, for getting pixel intensities
 *      Gx: current image gradient image on x direction
 *      Gy: current image gradient image on y direction
 *      ilay: the pyramid layer of current image
 *      T_curr: currently optimized transformation
 * Output:
 *      npts_in: the number of points falling in current camera view
 *      J_masked: Jacobian of the points in-view
 *      error_masked: error of the points in-view
 *      r_imu: 9x1 vector of IMU integrated observation error
 *      W: Huber weight for the points in-view
 * Use of class member:
 * Updates:
 *      mask_[ilay]: indicator of points in-view
 * Data-in:
 *      num_feat_pts_[ilay]
 *      feat_3D_[ilay]
 *      intensities_[ilay]
 *
 */
    void VoStereo::getError_fca(const cv::Mat& img_curr,
                                const cv::Mat& Gx_mat,
                                const cv::Mat& Gy_mat,
                                const Eigen::Isometry3f& T_curr,
                                const Eigen::Isometry3f& T_wb,
                                Eigen::MatrixXf& J_masked,
                                Eigen::VectorXf& error_masked,
                                Eigen::VectorXf& W,
                                int& npts_in,
                                const int ilay)
    {

        /* transformation of 3D point in keyframe to current frame and reproject */
        vector_aligned<Eigen::Vector3f> feat_3D_curr;  // 3D points in current camera's frame
        vector_aligned<Eigen::Vector2f> feat_pixels_curr;  // 2D pixel coordinates in current image
        transform3D(feat_3D_[ilay], feat_3D_curr, T_curr, num_feat_pts_[ilay]);
        cam_->get2DPixels(feat_3D_curr, feat_pixels_curr, num_feat_pts_[ilay], ilay);


        /* update mask and obtain gradients of reprojected points in current frame */
        npts_in = 0;
        std::vector<float> Gx, Gy;  // gradients of points in view
        std::vector<int> index;  // vector of size npts_in storing the corresponding index value
        std::vector<float> error_vec;  // error vector

        const float scale = error_scale_factor_; // Arbitrary scaling for better numerical conditioning
        for (int i=0; i<num_feat_pts_[ilay]; ++i) {
            if (inBound(feat_pixels_curr[i](0), feat_pixels_curr[i](1), 0, ilay)) {
                Gx.push_back(scale * interpolate_gradient(feat_pixels_curr[i](0), feat_pixels_curr[i](1), Gx_mat));
                Gy.push_back(scale * interpolate_gradient(feat_pixels_curr[i](0), feat_pixels_curr[i](1), Gy_mat));
                error_vec.push_back(scale * (intensities_[ilay][i]-interpolate(feat_pixels_curr[i](0),feat_pixels_curr[i](1),img_curr)));
                index.push_back(i);

//                mask_[ilay][i] = 1;
                npts_in += 1;
            }
//            else
//                mask_[ilay][i] = 0;
        }


        /* obtain photometric error for points in-view */
        error_masked = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(error_vec.data(), error_vec.size());



        /* obtain jacobian matrix for photometric error with respect to state */
        getJacobian_fca(Gx, Gy, feat_3D_curr, index, T_curr, T_wb, J_masked, ilay, npts_in);


        if (use_huber_)
        {
            /* find standard deviation of the error vector and compute W */
            float r_mean = error_masked.mean();
            Eigen::VectorXf r_hat = error_masked.array() - r_mean;  // normalize by removing the mean (original order)
            float sigma = sqrt(((r_hat.array()).square()).sum()/npts_in);  // standard deviation
            // std::cout << "sigma: " << sigma << std::endl;

            W.resize(npts_in);
            for (int i=0; i<npts_in; ++i) {
                if (fabs(r_hat[i]) < 1.345*sigma)
                    W[i] = 1;
                else
                    W[i] = (1.345*sigma)/fabs(r_hat[i]);
                if(use_weights_)
                    W[i] = W[i] * c_/(c_+error_masked[i]*error_masked[i]);

            }
        }
        else {

            W.resize(npts_in);
            W = Eigen::VectorXf::Constant(npts_in,1);

            if(use_weights_)
                for (int i=0; i<npts_in; ++i) {
                    W[i]=c_/(c_+error_masked[i]*error_masked[i]);
                    //std::cout<<"W" << W[i];
                }
            //std::cout<<"W ="<<W;
        }



    }




/*
 * Compute the Jacobian matrix for Forward Compositional Approach (FCA)
 * Called every iteration
 *
 * Input:
 *      Gx: gradient vector in x direction of current image
 *      Gy: gradient vector in y direction of current image
 *      feat_3D_curr: 3D feature points represented in current frame, of size num_feat_pts[ilay]
 *      index: vector of index for each point in-view, of size npts_in, ranges within num_feat_pts[ilay]
 *      T_curr: current transformation from keyframe to current frame. p_curr = T_curr * p_kf
 *      T_wb: transformation from current body frame to world frame. p_w = T_wb * p_b
 *      ilay: number of layer we're optimizing on
 *      npts_in: number of points in-view
 * Output:
 *      J: jacobian of size npts_in x 6, photometric error with respect to delta rotation phi in so(3) and delta translation p.
 *
 *
 */
    void VoStereo::getJacobian_fca(const std::vector<float>& Gx, const std::vector<float>& Gy,
                                   const vector_aligned<Eigen::Vector3f>& feat_3D_curr, const std::vector<int> &index,
                                   const Eigen::Isometry3f& T_curr, const Eigen::Isometry3f& T_wb, Eigen::MatrixXf& J, const int ilay, const int npts_in)
    {
//        std::cout<<"+++++calculating jacobian \n";
//        int id_ind = 475;

        Eigen::Matrix<float,3,3> R_cb = T_bc_inv_.rotation();
        Eigen::Matrix<float,3,3> R_wb = T_wb.rotation();

        J.resize(npts_in,9);
        J.setZero();
        Eigen::Matrix<float,1,3> J_i_phi; // Jacobian of one point on rotation
        Eigen::Matrix<float,1,3> J_i_p;  // Jacobian of one point on position
        Eigen::Matrix<float,1,2> J_grad; // gradient jacobian
        Eigen::Matrix<float,2,3> J_proj; // projection jacobian
        Eigen::Matrix<float,3,3> J_SO3;  // exponential jacobian

        float fx = focal_pyr_[ilay];
        float fy = focal_pyr_[ilay];

//        std::cout<<"fx: "<<fx<<"\n";
//        bool jacobianhasnan = false;
//        bool jacobianhasinf = false;
//        bool jacobian_too_large = false;

        for (int i=0; i < npts_in; ++i)
        {
            /* calculate for each point the 1x6 Jacobian */
            Eigen::Vector3f p_curr;  // point in current frame
            p_curr(0) = feat_3D_curr[index[i]](0);
            p_curr(1) = feat_3D_curr[index[i]](1);
            p_curr(2) = feat_3D_curr[index[i]](2);

            Eigen::Vector3f p_kf;  // point in keyframe
            p_kf(0) = feat_3D_[ilay][index[i]](0);
            p_kf(1) = feat_3D_[ilay][index[i]](1);
            p_kf(2) = feat_3D_[ilay][index[i]](2);


            Eigen::Vector4f p_kf_homo = unproject3d(p_kf); // homogeneous coordinate in keyframe
            Eigen::Vector4f p_hat_homo = T_bc_ * T_curr * p_kf_homo;
            Eigen::Vector3f p_hat = project3d(p_hat_homo);  // point p_i prime in jacobian formulation

            J_grad(0,0) = Gx[i];
            J_grad(0,1) = Gy[i];

            J_proj(0,0) = fx/p_curr(2);
            J_proj(1,0) = 0;
            J_proj(0,1) = 0;
            J_proj(1,1) = fy/p_curr(2);
            J_proj(0,2) = -fx*p_curr(0)/(p_curr(2)*p_curr(2));
            J_proj(1,2) = -fy*p_curr(1)/(p_curr(2)*p_curr(2));

            J_SO3 << 0,-p_hat(2),p_hat(1),
                    p_hat(2),0,-p_hat(0),
                    -p_hat(1),p_hat(0),0;  // [p_hat]x

            J_i_phi = - J_grad * J_proj * R_cb * J_SO3;
            J_i_p = J_grad * J_proj * R_cb * R_wb.transpose();

//            if (i==id_ind)
//            {
//                std::cout<<"J_grad: \n"<<J_grad<<"\n";
//                std::cout<<"J_proj: \n"<<J_proj<<"\n";
//                std::cout<<"J_SO3: \n"<<J_SO3<<"\n";
//                std::cout<<"J_i_p: \n"<<J_i_p<<"\n";
//                std::cout<<"J_i_phi: \n"<<J_i_phi<<"\n";
//            }

            /* assign J_i to J */
            J.block<1,3>(i,0) = J_i_phi;
            J.block<1,3>(i,3) = J_i_p;

            /* for debug - will print out info if nan values found */

//            for (int k=0; k<6; ++k){
//                if (std::isnan(J(i,k)))
//                    jacobianhasnan = true;
//                if (std::isinf(J(i,k)))
//                    jacobianhasinf = true;
//                if(J(i,k) > error_scale_factor_ * 1e8)
//                {
//                    jacobian_too_large = true;
//                    ROS_WARN_STREAM("jacobian too large! i: " << i << ", Index[i]: " << index[i] << " / " << num_feat_pts_[ilay] << "\nJ.row: "<< J.row(i));
//                    std::cout<<"J_grad: \n"<<J_grad<<"\n";
//                    std::cout<<"J_proj: \n"<<J_proj<<"\n";
//                    std::cout<<"J_SO3: \n"<<J_SO3<<"\n";
//                    std::cout<<"J_i_p: \n"<<J_i_p<<"\n";
//                    std::cout<<"J_i_phi: \n"<<J_i_phi<<"\n";
//                }
//            }

            /* for debug */
        }
//        if (jacobianhasnan)
//            std::cout << "Jacobian has nan"<< std::endl;
//        if (jacobianhasinf)
//            std::cout <<"Jacobian has inf"<<std::endl;
    }


}

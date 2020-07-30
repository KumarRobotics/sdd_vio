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


#ifndef PINHOLE_CAMERA_STEREO_H_
#define PINHOLE_CAMERA_STEREO_H_

#include <Eigen/Dense>
#include "utils/math_utils.h"
#include <opencv2/opencv.hpp>
#include <ros/ros.h>


namespace sdd_vio {

/* this class keeps relative transformation information about the stereo camera set */
class PinholeCameraStereo {

public:
	cv::Mat rectify_map_[2][2];
	cv::Mat Q_;  // 4x4 matrix for converting disparity map to depth map
    cv::Ptr<cv::StereoSGBM> matcher_sgbm_400_, matcher_sgbm_200_;  // block matcher for disparity map
	cv::Ptr<cv::StereoBM> matcher_bm_400_;
	cv::Ptr<cv::StereoBM> matcher_bm_200_;
	cv::Ptr<cv::StereoBM> matcher_bm_100_;
	cv::Ptr<cv::StereoBM> matcher_bm_50_;
  	bool rectify_maps_initialized_;
  	double cx_rec_, cy_rec_, f_rec_, base_;
  	Eigen::Matrix3f K_rec_;
  	std::vector<double> cx_rec_pyr_, cy_rec_pyr_, f_rec_pyr_;
  	vector_aligned<Eigen::Matrix3f> K_rec_pyr_;
  	int frame_width_, frame_height_;
  	int nlays_, base_layer_;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    PinholeCameraStereo(const ros::NodeHandle &nh);

    inline bool isRectified() { return rectify_maps_initialized_; }
    inline double getf() {return f_rec_;}
    inline int getWidth() {return frame_width_;}
    inline int getHeight() {return frame_height_;}

	void undistortImage(const cv::Mat& raw0, cv::Mat& rectified0, const cv::Mat& raw1, cv::Mat& rectified1);
	void get3DMap(const cv::Mat& img0, const cv::Mat& img1, cv::Mat& depth);
    void getDisparityMap(const cv::Mat& img0, const cv::Mat& img1, cv::Mat& disp);


	/* given feature pixel coordinates and disparity values, triangulate 3D points */
	void get3DPoints(const vector_aligned<Eigen::Vector2i>& feat_pixels,
        const std::vector<float>& disp_vec, vector_aligned<Eigen::Vector3f>& feat_3D, int npts, int ilays);

	/* given 3D points in camera frame (left), reproject to get image pixel coordinates */
	void get2DPixels(const vector_aligned<Eigen::Vector3f>& feat_3D,
		vector_aligned<Eigen::Vector2f>& feat_pixels, int npts, const int ilays);

	/* configure camera intrinsics for specified pyramid level */
	void configPyramid();

	/* initialize the size of camera intrinsic vectors */
	void initPyramid(int nlays, int base_layer);


};  // end class PinholeCameraStereo

}  // end namespace sdd_vio

#endif /* PINHOLE_CAMERA_STEREO_H_ */

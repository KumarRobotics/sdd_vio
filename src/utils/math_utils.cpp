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



#include "sdd_vio/utils/math_utils.h"
#include <iostream>
#include <string>
#include <cassert>
#include <math.h>

namespace sdd_vio {

void setTransform(const std::vector<double> &T_vec,
       Eigen::Matrix3f &R, Eigen::Vector3f &t_vec)
{
  R(0, 0) = T_vec.at(0);
  R(0, 1) = T_vec.at(1);
  R(0, 2) = T_vec.at(2);
  R(1, 0) = T_vec.at(4);
  R(1, 1) = T_vec.at(5);
  R(1, 2) = T_vec.at(6);
  R(2, 0) = T_vec.at(8);
  R(2, 1) = T_vec.at(9);
  R(2, 2) = T_vec.at(10);

  t_vec(0) = T_vec.at(3);
  t_vec(1) = T_vec.at(7);
  t_vec(2) = T_vec.at(11);
}

void showMatProp( const cv::Mat& inputMat )
{
    int inttype = inputMat.type();
    cv::Size sz = inputMat.size();

    std::string r, a;
    uchar depth = inttype & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (inttype >> CV_CN_SHIFT);
    switch ( depth ) {
        case CV_8U:  r = "8U";   a = "Mat.at<uchar>(y,x)"; break;
        case CV_8S:  r = "8S";   a = "Mat.at<schar>(y,x)"; break;
        case CV_16U: r = "16U";  a = "Mat.at<ushort>(y,x)"; break;
        case CV_16S: r = "16S";  a = "Mat.at<short>(y,x)"; break;
        case CV_32S: r = "32S";  a = "Mat.at<int>(y,x)"; break;
        case CV_32F: r = "32F";  a = "Mat.at<float>(y,x)"; break;
        case CV_64F: r = "64F";  a = "Mat.at<double>(y,x)"; break;
        default:     r = "User"; a = "Mat.at<UKNOWN>(y,x)"; break;
    }
    r += "C";
    r += (chans+'0');
    std::cout << "Mat is of type " << r << " and should be accessed with " << a << std::endl;
    std::cout << "Mat is of size " << sz << std::endl;
}

cv::Mat adjustVis(const cv::Mat& M)
{
    double min, max;
    cv::minMaxIdx(M, &min, &max);
    cv::Mat M_shifted = M - min;
    cv::Mat adjM;
    cv::convertScaleAbs(M_shifted, adjM, 255 / (max-min));
    return adjM;
}

void showMatMinMax(const cv::Mat& inputMat)
{
    double min, max;
    cv::minMaxLoc(inputMat, &min, &max);
    std::cout << "min: " << min << "\n" << "max: " << max << std::endl;
}


Eigen::Matrix4f TfromRt(const Eigen::Matrix3f& R, const Eigen::Vector3f& t)
{
    Eigen::Matrix4f T;
    T.setIdentity();
    T.block<3,3>(0,0) = R;
    T.block<3,1>(0,3) = t;
    return T;
}

void RtfromT(const Eigen::Matrix4f& T, Eigen::Matrix3f& R, Eigen::Vector3f& t, Eigen::Quaternionf& q)
{
    R = T.block<3,3>(0,0);
    q = R;
    q.normalize();
    t = T.block<3,1>(0,3);
}

void RtfromT(const Eigen::Matrix4f& T, Eigen::Matrix3f& R, Eigen::Vector3f& t)
{
    R = T.block<3,3>(0,0);
    t = T.block<3,1>(0,3);
}

bool isIdentityT(const Eigen::Matrix4f& T)
{
    bool isIdy = (T(0,0)==1)&&(T(1,1)==1)&&(T(2,2)==1)
                &&(T(0,1)==0)&&(T(0,2)==0)&&(T(1,2)==0)
                &&(T(0,3)==0)&&(T(1,3)==0)&&(T(2,3)==0)
                &&(T(3,0)==0)&&(T(3,1)==0)&&(T(3,2)==0)&&(T(3,3)==1);
    return isIdy;
}


}  // namespace sdd_vio

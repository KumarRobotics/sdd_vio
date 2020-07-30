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

#ifndef MATH_UTILS_H_
#define MATH_UTILS_H_

#include <Eigen/Dense>
#include <algorithm>
#include <iostream>

#include "opencv2/core/version.hpp"

#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>


namespace sdd_vio
{
  template <typename T>
    using vector_aligned = std::vector<T, Eigen::aligned_allocator<T>>;

  inline Eigen::Vector2f project2d(const Eigen::Vector3f& v)
  { return v.head<2>()/v[2]; }


  inline Eigen::Vector3f unproject2d(const Eigen::Vector2f& v)
  { return Eigen::Vector3f(v[0], v[1], 1.0); }

  inline Eigen::Vector3f project3d(const Eigen::Vector4f& v)
  { return v.head<3>()/v[3]; }

  inline Eigen::Vector4f unproject3d(const Eigen::Vector3f& v)
  { return Eigen::Vector4f(v[0], v[1], v[2], 1.0); }

  /* transform external calibration vectors */
  void setTransform(const std::vector<double> &T_vec,
      Eigen::Matrix3f &R, Eigen::Vector3f &t_vec);

  /* print cv matrix property to command line */
  void showMatProp(const cv::Mat& inputMat);
  /* adjust matrix M for visualization between 0-255 */
  cv::Mat adjustVis(const cv::Mat& M);
  /* show min max values of cv matrix */
  void showMatMinMax(const cv::Mat& inputMat);
  /* show min max elements of std::vector */
  template<class T>
    void showVecMinMax(std::vector<T>& inputVec)
    {
      auto result = std::minmax_element (inputVec.begin(),inputVec.end());
      std::cout << "min is " << *result.first;
      std::cout << ", at position " << (result.first-inputVec.begin()) << '\n';
      std::cout << "max is " << *result.second;
      std::cout << ", at position " << (result.second-inputVec.begin()) << '\n';
    }
  /* show min max values of Eigen matrix */
  template<class T>
    void showEigenInfo(T& inputM)
    {
      // std::ptrdiff_t i;
      std::cout << "The Eigen array is of size " << inputM.rows() << "x" << inputM.cols() << std::endl;
      // std::cout << "min is " << inputM.minCoeff(&i) << " is at position " << i << '\n';
      // std::cout << "max is " << inputM.maxCoeff(&i) << " is at position " << i << '\n';
      std::cout << "min is " << inputM.minCoeff() << '\n';
      std::cout << "max is " << inputM.maxCoeff() << '\n';
      std::cout << "mean is " << inputM.mean() << '\n';
      std::cout << "sum is " << inputM.sum() << '\n';
    }



  template <typename T>
  Eigen::Matrix<T,3,3> hat(const Eigen::Matrix<T,3,1>& vec)
  {
      Eigen::Matrix<T,3,3> mat;
      T a1 = vec(0);
      T a2 = vec(1);
      T a3 = vec(2);
//      mat << 0,-a3,a2,a3,0,-a1,-a2,a1,0;
      mat(0,0) = T(0);
      mat(0,1) = -a3;
      mat(0,2) = a2;
      mat(1,0) = a3;
      mat(1,1) = T(0);
      mat(1,2) = -a1;
      mat(2,0) = -a2;
      mat(2,1) = a1;
      mat(2,2) = T(0);
      return mat;
  }


  template <typename T>
  Eigen::Matrix<T,3,1> get_translation(const Eigen::Matrix<T, 4, 4>& SE3){
      Eigen::Matrix<T,3,1>  result;
      result(0,0) = SE3(0,3);
      result(1,0) = SE3(1,3);
      result(2,0) = SE3(2,3);
      return result;
  }


  template <typename T>
  Eigen::Matrix<T,3,3> get_rotation(const Eigen::Matrix<T, 4, 4>& SE3){
      Eigen::Matrix<T,3,3>  result = SE3.block(0,0,3,3);
      return result;
  }


  template <typename T>
  Eigen::Matrix<T,4,4> exp_SE3(Eigen::Matrix<T,Eigen::Dynamic,1>& se3)
  {
      assert(se3.size() == 6);
      // std::cout << "passed first assert" << std::endl;
      /* implemented according to Ethan's paper on lie group */
      Eigen::Matrix<T,3,1> u,w;
      T theta, A, B, C;
      Eigen::Matrix<T,3,3> R, V, I;
      Eigen::Matrix<T,4,4> T_mat;

      u = se3.block(0,0,3,1);
      w = se3.block(3,0,3,1);

      theta = sqrt(w.transpose()*w);
      /* when theta is small, do taylor expansion on A, B, C */
      if (theta < 0.001)
      {
          A = 1.0 - pow(theta,2)/(3*2);
          B = 1.0/2 - pow(theta,2)/(4*3*2);
          C = 1.0/(3*2) - pow(theta,2)/(5*4*3*2);
      }
      else
      {
          A = sin(theta)/theta;
          B = (1-cos(theta))/pow(theta,2);
          C = (1-A)/pow(theta,2);
      }

      I = Eigen::Matrix<T,3,3>::Identity();
      R = I + A*hat(w) + B*hat(w)*hat(w);
      V = I + B*hat(w) + C*hat(w)*hat(w);

      T_mat = Eigen::Matrix<T,4,4>::Identity();
      T_mat.block(0,0,3,3) = R;
      T_mat.block(0,3,3,1) = V*u;

      return T_mat;
  }


  template <typename T>
  void rodrigues_so3_exp(const Eigen::Matrix<T,3,1>& w, const T A, const T B, Eigen::Matrix<T,3,3>& R){

      const T wx2 = w(0,0)*w(0,0);
      const T wy2 = w(1,0)*w(1,0);
      const T wz2 = w(2,0)*w(2,0);

      R(0,0) = 1.0 - B*(wy2 + wz2);
      R(1,1) = 1.0 - B*(wx2 + wz2);
      R(2,2) = 1.0 - B*(wx2 + wy2);

      T a = A*w(2,0);
      T b = B*(w(0,0)*w(1,0));

      R(0,1) = b - a;
      R(1,0) = b + a;

      a = A*w(1,0);
      b = B*(w(0,0)*w(2,0));

      R(0,2) = b + a;
      R(2,0) = b - a;

      a = A*w(0,0);
      b = B*(w(1,0)*w(2,0));

      R(1,2) = b - a;
      R(2,1) = b + a;

  }



  template <typename T>
  Eigen::Matrix<T,3,3> exp_SO3(const Eigen::Matrix<T,3,1>& so3)
  {
      assert(so3.size() == 3);
      T theta, A, B;

      theta = sqrt(T(so3.transpose()*so3));
      if (theta < 0.001)
      {
          A = 1.0 - pow(theta,2)/T(3*2);
          B = 1.0/2 - pow(theta,2)/T(4*3*2);
      }
      else
      {
          A = sin(theta)/theta;
          B = (T(1)-cos(theta))/pow(theta,2);
      }

      Eigen::Matrix<T,3,3> I = Eigen::Matrix<T,3,3>::Identity();
      Eigen::Matrix<T,3,3> R = I + A*hat(so3) + B*hat(so3)*hat(so3);

      return R;
  }



  template <typename T>
  Eigen::Matrix<T,3,1> log_SO3(const Eigen::Matrix<T,3,3>& SO3){

      Eigen::Matrix<T, 3, 1> result;
      const T cos_angle = (SO3(0,0) + SO3(1,1) + SO3(2,2) - 1.0) * 0.5;
      result(0,0) = (SO3(2,1)-SO3(1,2))/2;
      result(1,0) = (SO3(0,2)-SO3(2,0))/2;
      result(2,0) = (SO3(1,0)-SO3(0,1))/2;

      T sin_angle_abs = sqrt(result.transpose()*result);

      if (cos_angle > M_SQRT1_2) {            // [0 - Pi/4[ use asin
          if(sin_angle_abs > 0){
              result *= asin(sin_angle_abs) / sin_angle_abs;
          }
      }
      else if( cos_angle > -M_SQRT1_2) {    // [Pi/4 - 3Pi/4[ use acos, but antisymmetric part
          const T angle = acos(cos_angle);
          result *= angle / sin_angle_abs;


      }
      else
      {  // rest use symmetric part
          // antisymmetric part vanishes, but still large rotation, need information from symmetric part
          const T angle = M_PI - asin(sin_angle_abs);
          const T d0 = SO3(0,0) - cos_angle,
                  d1 = SO3(1,1) - cos_angle,
                  d2 = SO3(2,2) - cos_angle;
          Eigen::Matrix<T,3,1> r2;

          if(d0*d0 > d1*d1 && d0*d0 > d2*d2){ // first is largest, fill with first column
              r2(0,0) = d0;
              r2(1,0) = (SO3(1,0) + SO3(0,1))/2;
              r2(2,0) = (SO3(0,2) + SO3(2,0))/2;


          }
          else if(d1*d1 > d2*d2) {              // second is largest, fill with second column
              r2(0,0) = (SO3(1,0) + SO3(0,1))/2;
              r2(1,0) = d1;
              r2(2,0) = (SO3(2,1) + SO3(1,2))/2;


          }
          else {                                // third is largest, fill with third column
              r2(0,0) = (SO3(0,2) + SO3(2,0))/2;
              r2(1,0) = (SO3(2,1) + SO3(1,2))/2;
              r2(2,0) = d2;


          }
          // flip, if we point in the wrong direction!
          if(r2.transpose() * result < 0)
              r2 *= -1;
          //normalize the vector
          r2.normalize();
          //result = TooN::operator*(angle,r2);
          result = angle*r2;
      }
      return result;
  }


  template <typename T>
  Eigen::Matrix<T, 6, 1> log_SE3(const Eigen::Matrix<T,4,4>& SE3) {
      Eigen::Matrix<T,3,1> rot = log_SO3(get_rotation(SE3));
      const T theta = sqrt(rot.transpose()*rot);

      T shtot = T(0.5);
      if(theta > 0.00001) {
          shtot = sin(theta/2)/theta;
      }

       // now do the rotation
      const Eigen::Matrix<T,3,3> halfrotator = exp_SO3<T>(rot * -0.5);
      Eigen::Matrix<T,3,1> rottrans = halfrotator * get_translation(SE3);

      if(theta > 0.001){
          rottrans -=(rot*((get_translation(SE3).transpose() * rot) * (1.0-2.0*shtot) / (rot.transpose()*rot)));
       } else {
          rottrans -=(rot*((get_translation(SE3).transpose() * rot)/24.0));
      }

      rottrans /= (2 * shtot);

      Eigen::Matrix<T,6,1> result;
      result.block(0,0,3,1) = rottrans;
      result.block(3,0,3,1) = rot;

      return result;
  }


  /* right Jacobian for Log operation on SO(3) */
  template <typename T>
  Eigen::Matrix<T,3,3> log_Jacobian(const Eigen::Matrix<T,3,1>& phi)
  {
      Eigen::Matrix<T,3,3> I = Eigen::Matrix<T,3,3>::Identity();
      Eigen::Matrix<T,3,3> J;

      T theta = phi.norm();
      if (theta < 0.001)
          J = I + 0.5*hat(phi);
      else
          J = I + 0.5*hat(phi) + (1/pow(theta,2) + ((1+cos(theta))/(2*theta*sin(theta))))*hat(phi)*hat(phi);

      return J;
  }

  /* right Jacobian for Exp operation on SO(3) */
  template <typename T>
  Eigen::Matrix<T,3,3> exp_Jacobian(const Eigen::Matrix<T,3,1>& phi)
  {
      Eigen::Matrix<T,3,3> I = Eigen::Matrix<T,3,3>::Identity();
      Eigen::Matrix<T,3,3> J;

      T theta = phi.norm();
      if (theta < 0.001)
          J = I - 0.5 * hat(phi) + (1.0/6.0)*hat(phi)*hat(phi);
      else
          J = I - (1-cos(theta))/pow(theta,2) * hat(phi) + (theta-sin(theta))/pow(theta,3)*hat(phi)*hat(phi);

      return J;
  }



  // Eigen::Matrix4f TfromRt(const Eigen::Matrix3f &R, const Eigen::Vector3f &t);

  // void RtfromT(const Eigen::Matrix4f& T, Eigen::Matrix3f& R, Eigen::Vector3f& t, Eigen::Quaternionf& q);
  // void RtfromT(const Eigen::Matrix4f& T, Eigen::Matrix3f& R, Eigen::Vector3f& t);

  /* check if T is identity */
  bool isIdentityT(const Eigen::Matrix4f& T);


}

#endif /* MATH_UTILS_H_ */

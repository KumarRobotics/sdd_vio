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

#ifndef ROS_PARAMS_HELPER_H_
#define ROS_PARAMS_HELPER_H_

#include <string>
#include <ros/ros.h>
#include <iostream>
using namespace std;

namespace sdd_vio {

inline
bool hasParam(const std::string& name)
{
  return ros::param::has(name);
}

template<typename T>
T getParam(const std::string& name, const T& defaultValue)
{
  T v;
  if(ros::param::get(name, v))
  {
    // ROS_INFO_STREAM("Found parameter: " << name << ", value: " << v);
    return v;
  }
  else
    ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
  return defaultValue;
}

template<typename T>
T getParam(const std::string& name)
{
  T v;
  if(ros::param::get(name, v))
  {
    // ROS_INFO_STREAM("Found parameter: " << name << ", value: " << v);
    return v;
  }
  else
    ROS_ERROR_STREAM("Cannot find value for parameter: " << name);
  return T();
}


} // namespace sdd_vio

#endif // ROS_PARAMS_HELPER_H_

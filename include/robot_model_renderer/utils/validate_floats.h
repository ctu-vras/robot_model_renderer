// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: 2009 Willow Garage, Inc.

#pragma once

// This file is taken from rviz and minimally edited (just code style and different namespace).

#include <cmath>

#include <OgreQuaternion.h>

#include <boost/array.hpp>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <robot_model_renderer/ogre_helpers/ogre_vector.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/ColorRGBA.h>

namespace robot_model_renderer
{
inline bool validateFloats(const float val)
{
  return !(std::isnan(val) || std::isinf(val));
}

inline bool validateFloats(const double val)
{
  return !(std::isnan(val) || std::isinf(val));
}

inline bool validateFloats(const Ogre::Vector3& vec)
{
  bool valid = true;
  valid = valid && validateFloats(vec.x);
  valid = valid && validateFloats(vec.y);
  valid = valid && validateFloats(vec.z);
  return valid;
}

inline bool validateFloats(const Ogre::Quaternion& quat)
{
  bool valid = true;
  valid = valid && validateFloats(quat.x);
  valid = valid && validateFloats(quat.y);
  valid = valid && validateFloats(quat.z);
  valid = valid && validateFloats(quat.w);
  return valid;
}

inline bool validateFloats(const geometry_msgs::Point& msg)
{
  bool valid = true;
  valid = valid && validateFloats(msg.x);
  valid = valid && validateFloats(msg.y);
  valid = valid && validateFloats(msg.z);
  return valid;
}

inline bool validateFloats(const geometry_msgs::Point32& msg)
{
  bool valid = true;
  valid = valid && validateFloats(msg.x);
  valid = valid && validateFloats(msg.y);
  valid = valid && validateFloats(msg.z);
  return valid;
}

inline bool validateFloats(const geometry_msgs::Vector3& msg)
{
  bool valid = true;
  valid = valid && validateFloats(msg.x);
  valid = valid && validateFloats(msg.y);
  valid = valid && validateFloats(msg.z);
  return valid;
}

inline bool validateFloats(const geometry_msgs::Twist& twist)
{
  bool valid = true;
  valid = valid && validateFloats(twist.linear);
  valid = valid && validateFloats(twist.angular);
  return valid;
}

inline bool validateFloats(const geometry_msgs::Quaternion& msg)
{
  bool valid = true;
  valid = valid && validateFloats(msg.x);
  valid = valid && validateFloats(msg.y);
  valid = valid && validateFloats(msg.z);
  valid = valid && validateFloats(msg.w);
  return valid;
}

inline bool validateFloats(const std_msgs::ColorRGBA& msg)
{
  bool valid = true;
  valid = valid && validateFloats(msg.r);
  valid = valid && validateFloats(msg.g);
  valid = valid && validateFloats(msg.b);
  valid = valid && validateFloats(msg.a);
  return valid;
}

inline bool validateFloats(const geometry_msgs::PointStamped& msg)
{
  return validateFloats(msg.point);
}

inline bool validateFloats(const geometry_msgs::Pose& msg)
{
  bool valid = true;
  valid = valid && validateFloats(msg.position);
  valid = valid && validateFloats(msg.orientation);
  return valid;
}

inline bool validateFloats(const geometry_msgs::PoseStamped& msg)
{
  return validateFloats(msg.pose);
}

template <typename T>
inline bool validateFloats(const std::vector<T>& vec)
{
  typedef std::vector<T> VecType;
  typename VecType::const_iterator it = vec.begin();
  typename VecType::const_iterator end = vec.end();
  for (; it != end; ++it)
  {
    if (!validateFloats(*it))
    {
      return false;
    }
  }

  return true;
}

template <typename T, size_t N>
inline bool validateFloats(const boost::array<T, N>& arr)
{
  typedef boost::array<T, N> ArrType;
  typename ArrType::const_iterator it = arr.begin();
  typename ArrType::const_iterator end = arr.end();
  for (; it != end; ++it)
  {
    if (!validateFloats(*it))
    {
      return false;
    }
  }

  return true;
}

inline bool validateFloats(const sensor_msgs::CameraInfo& msg)
{
  bool valid = true;
  valid = valid && validateFloats(msg.D);
  valid = valid && validateFloats(msg.K);
  valid = valid && validateFloats(msg.R);
  valid = valid && validateFloats(msg.P);
  return valid;
}

} // namespace robot_model_renderer

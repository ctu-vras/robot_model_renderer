// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <robot_model_renderer/utils/sensor_msgs_ogre.hpp>

#include <string>

#include <OgrePixelFormat.h>

#include <sensor_msgs/image_encodings.h>

Ogre::PixelFormat robot_model_renderer::sensorMsgsEncodingToOgrePixelFormat(const std::string& encoding)
{
  namespace enc = sensor_msgs::image_encodings;
  if (encoding == enc::RGB8)
  {
    return Ogre::PF_BYTE_RGB;
  }
  else if (encoding == enc::BGR8)
  {
    return Ogre::PF_BYTE_BGR;
  }
  else if (encoding == enc::RGBA8)
  {
    return Ogre::PF_BYTE_RGBA;
  }
  else if (encoding == enc::BGRA8)
  {
    return Ogre::PF_BYTE_BGRA;
  }
  else if (encoding == enc::MONO8)
  {
    return Ogre::PF_BYTE_L;
  }
  else if (encoding == enc::RGB16)
  {
    return Ogre::PF_SHORT_RGB;
  }
  else if (encoding == enc::RGBA16)
  {
    return Ogre::PF_SHORT_RGBA;
  }
  else if (encoding == enc::MONO16)
  {
    return Ogre::PF_SHORT_L;
  }
  throw std::runtime_error("Unsupported pixel format");
}

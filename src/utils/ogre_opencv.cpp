// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <opencv2/core.hpp>

#include <robot_model_renderer/utils/ogre_opencv.hpp>

int robot_model_renderer::ogrePixelFormatToCvMatType(const Ogre::PixelFormat& pf)
{
  int cvType;
  const auto componentType = Ogre::PixelUtil::getComponentType(pf);

  switch (componentType)
  {
    case Ogre::PCT_BYTE:
      cvType = CV_8U;
      break;
    case Ogre::PCT_SHORT:
      cvType = CV_16U;
      break;
    case Ogre::PCT_FLOAT16:
      cvType = CV_16F;
      break;
    case Ogre::PCT_FLOAT32:
      cvType = CV_32F;
      break;
    case Ogre::PCT_SINT:
      cvType = CV_32S;
      break;
    default:
      throw std::runtime_error(std::string("Unsupported pixel component type: ") + std::to_string(componentType));
  }
  const auto cvDepth = Ogre::PixelUtil::getComponentCount(pf);

  return CV_MAKETYPE(cvType, cvDepth);
}

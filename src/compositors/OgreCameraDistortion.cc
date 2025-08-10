// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2021 Open Source Robotics Foundation
// SPDX-FileCopyrightText: Czech Technical University in Prague

// This file is based on https://github.com/gazebosim/gz-rendering/blob/gz-rendering9/ogre/src/OgreDistortionPass.cc
// with many local changes.

/**
 * \file
 * \brief OGRE compositor for camera distortion effect.
 * \author Martin Pecka
 * \author Gazebo authors
 */

#include <OgreCamera.h>
#include <OgreCompositionPass.h>
#include <OgreCompositionTargetPass.h>
#include <OgreCompositorInstance.h>
#include <OgreCompositorManager.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterial.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <OgreViewport.h>

#include <Eigen/Core>

#include <opencv2/calib3d.hpp>

#include <cras_cpp_common/log_utils.h>
#include <image_geometry/pinhole_camera_model.h>
#include <robot_model_renderer/compositors/OgreCameraDistortion.hh>
#include <robot_model_renderer/pinhole_camera.h>

namespace robot_model_renderer
{

struct OgreCameraDistortion::Implementation
{
  //! \brief Distortion compositor.
  Ogre::CompositorInstance* distortionInstance = nullptr;

  //! \brief Ogre Material that contains the distortion shader
  Ogre::MaterialPtr distortionMaterial;

  //! \brief Ogre Texture that contains the distortion map
  Ogre::TexturePtr distortionTexture;

  //! \brief Model of the camera.
  PinholeCameraModel pinholeCameraModel;

  //! \brief Whether to use distortion map.
  bool useDistortionMap {false};

  //! \brief Name of the compositor.
  std::string compositorName;
};

Ogre::Matrix3 ogreMatFromCv(const cv::Matx33d& cv)
{
  // Ogre uses column-major matrices.
  return {
    static_cast<float>(cv(0, 0)), static_cast<float>(cv(1, 0)), static_cast<float>(cv(2, 0)),
    static_cast<float>(cv(0, 1)), static_cast<float>(cv(1, 1)), static_cast<float>(cv(2, 1)),
    static_cast<float>(cv(0, 2)), static_cast<float>(cv(1, 2)), static_cast<float>(cv(2, 2))
  };
}

Ogre::Matrix3 ogreMatFromCv(const cv::Matx34d& cv)
{
  // Ogre uses column-major matrices.
  return {
    static_cast<float>(cv(0, 0)), static_cast<float>(cv(1, 0)), static_cast<float>(cv(2, 0)),
    static_cast<float>(cv(0, 1)), static_cast<float>(cv(1, 1)), static_cast<float>(cv(2, 1)),
    static_cast<float>(cv(0, 2)), static_cast<float>(cv(1, 2)), static_cast<float>(cv(2, 2))
  };
}

OgreCameraDistortion::OgreCameraDistortion(const cras::LogHelperPtr& log, const bool useDistortionMap)
  : cras::HasLogger(log), dataPtr(std::make_unique<Implementation>())
{
  this->dataPtr->useDistortionMap = useDistortionMap;
  this->dataPtr->compositorName = this->dataPtr->useDistortionMap ?
    "RenderPass/CameraMappedDistortion" : "RenderPass/InverseRectification";
}

OgreCameraDistortion::~OgreCameraDistortion()
{
  this->Destroy();
}

void OgreCameraDistortion::SetCamera(Ogre::Camera* camera)
{
  this->ogreCamera = camera;
}

bool OgreCameraDistortion::SetCameraModel(const PinholeCameraModel& cam)
{
  this->dataPtr->pinholeCameraModel = cam;
  return true;
}

void OgreCameraDistortion::CreateRenderPass()
{
  if (!this->ogreCamera)
  {
    CRAS_ERROR_NAMED("compositors.distortion", "No camera set for applying Distortion Pass");
    return;
  }

  if (this->dataPtr->distortionInstance)
  {
    CRAS_ERROR_NAMED("compositors.distortion", "Distortion pass already created.");
    return;
  }

  const auto& coeffs = this->dataPtr->pinholeCameraModel.distortionCoeffs();
  // If no distortion is required, immediately return.
  if (std::all_of(coeffs.begin(), coeffs.end(), [](const double x) {return std::fabs(x) < 1e-6;}))
    return;

  const auto& renderTextureResolution = this->dataPtr->pinholeCameraModel.reducedResolution();
  const auto& distortionTextureWidth = renderTextureResolution.width;
  const auto& distortionTextureHeight = renderTextureResolution.height;

  // set up the distortion instance
  const auto matName = this->dataPtr->useDistortionMap ? "CameraMappedDistortion" : "InverseRectification";
  const auto& mat = Ogre::MaterialManager::getSingleton().getByName(matName);
  this->dataPtr->distortionMaterial = mat->clone(this->ogreCamera->getName() + "_Distortion");
  this->dataPtr->distortionMaterial->getTechnique(0)->getPass(0)->setTextureFiltering(Ogre::TFO_BILINEAR);

  if (this->dataPtr->useDistortionMap)
  {
    // initialize distortion map
    cv::Mat unrectifyMap = this->dataPtr->pinholeCameraModel.getUnrectifyFloatMap(
      this->dataPtr->pinholeCameraModel.intrinsicMatrix(), renderTextureResolution);

    // create the distortion map texture for the distortion instance
    const auto texName = this->ogreCamera->getName() + "_distortionTex";
    this->dataPtr->distortionTexture = Ogre::TextureManager::getSingleton().createManual(
      texName, "General", Ogre::TEX_TYPE_2D, distortionTextureWidth, distortionTextureHeight, 0, Ogre::PF_FLOAT32_RGB);

    const auto pixelBuffer = this->dataPtr->distortionTexture->getBuffer();
    pixelBuffer->lock(Ogre::HardwareBuffer::HBL_NORMAL);
    const auto& pixelBox = pixelBuffer->getCurrentLock();

    // Copy the data from format xyxyxy to xy0xy0xy0 as the texture has to be 3-channel (why??)
    using DistortionTexture = Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor | Eigen::DontAlign>;
    using PixelBufferMap = Eigen::Map<DistortionTexture, Eigen::Unaligned, Eigen::Stride<3 ,1>>;
    using DistortionTextureMap = Eigen::Map<DistortionTexture>;

    const auto distortionTextureSize = distortionTextureWidth * distortionTextureHeight;
    PixelBufferMap pixelBufferMap(static_cast<float*>(pixelBox.data), distortionTextureSize, 2);
    const DistortionTextureMap distortionTextureMap(unrectifyMap.ptr<float>(0, 0), distortionTextureSize, 2);
    pixelBufferMap = distortionTextureMap;

    // Scale to relative image coordinates
    const Eigen::Array2f stepSize(1.0f / distortionTextureWidth, 1.0f / distortionTextureHeight);
    pixelBufferMap.array().rowwise() *= stepSize.transpose();

    pixelBuffer->unlock();

    // set up the distortion map texture to be used in the pixel shader.
    this->dataPtr->distortionMaterial->getTechnique(0)->getPass(0)->createTextureUnitState(texName, 1);

    auto params = this->dataPtr->distortionMaterial->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
    params->setNamedConstant("backgroundColor", this->ogreCamera->getViewport()->getBackgroundColour());
  }
  else
  {
    auto params = this->dataPtr->distortionMaterial->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
    std::array<float, 14> D{};
    std::copy_n(coeffs.begin(), std::min(14, coeffs.size().area()), D.begin());
    params->setNamedConstant("cameraMatrixVec",
      ogreMatFromCv(this->dataPtr->pinholeCameraModel.intrinsicMatrix())[0], 1, 9);
    params->setNamedConstant("distCoeffs", D.data(), 1, 14);
    params->setNamedConstant("rectificationRotationVec",
      ogreMatFromCv(this->dataPtr->pinholeCameraModel.rotationMatrix())[0], 1, 9);
    params->setNamedConstant("newCameraMatrixVec",
      ogreMatFromCv(this->dataPtr->pinholeCameraModel.projectionMatrix())[0], 1, 9);
    params->setNamedConstant("size", Ogre::Vector2(distortionTextureWidth, distortionTextureHeight));
    params->setNamedConstant("backgroundColor", this->ogreCamera->getViewport()->getBackgroundColour());
  }

  // create compositor instance
  this->dataPtr->distortionInstance = Ogre::CompositorManager::getSingleton().addCompositor(
    this->ogreCamera->getViewport(), this->dataPtr->compositorName);
  this->dataPtr->distortionInstance->getTechnique()->getOutputTargetPass()->getPass(0)->setMaterial(
    this->dataPtr->distortionMaterial);

  this->dataPtr->distortionInstance->setEnabled(true);
}

void OgreCameraDistortion::Destroy()
{
  if (this->dataPtr->distortionInstance)
  {
    this->dataPtr->distortionInstance->setEnabled(false);
    Ogre::CompositorManager::getSingleton().removeCompositor(
      this->ogreCamera->getViewport(), this->dataPtr->compositorName);

    this->dataPtr->distortionInstance = nullptr;
  }
  if (!this->dataPtr->distortionMaterial.isNull())
  {
    this->dataPtr->distortionMaterial->unload();
    this->dataPtr->distortionMaterial.setNull();
  }
  if (!this->dataPtr->distortionTexture.isNull())
  {
    Ogre::TextureManager::getSingleton().unload(this->dataPtr->distortionTexture->getHandle());
    this->dataPtr->distortionTexture.setNull();
  }
}

}

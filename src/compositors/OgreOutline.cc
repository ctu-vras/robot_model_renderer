// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief OGRE compositor drawing an outline around the rendered objects.
 * \author Martin Pecka
 */

#include <robot_model_renderer/compositors/OgreOutline.hh>

#include <OgreCamera.h>
#include <OgreCompositionPass.h>
#include <OgreCompositionTargetPass.h>
#include <OgreCompositorInstance.h>
#include <OgreCompositorManager.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterial.h>
#include <OgreTechnique.h>

namespace robot_model_renderer
{

struct OgreOutline::Implementation
{
  //! \brief Distortion compositor.
  Ogre::CompositorInstance* distortionInstance = nullptr;

  //! \brief Ogre Material that contains the outline shader
  Ogre::MaterialPtr distortionMaterial;

  //! \brief Width of the outline.
  float outlineWidth {5.0f};

  Ogre::ColourValue outlineColor {Ogre::ColourValue::Black};

  bool outlineFromClosestColor {false};
};

OgreOutline::OgreOutline(
  const double outlineWidth, const Ogre::ColourValue& outlineColor, const bool outlineFromClosestColor)
  : dataPtr(std::make_unique<Implementation>())
{
  this->dataPtr->outlineWidth = outlineWidth;
  this->dataPtr->outlineColor = outlineColor;
  this->dataPtr->outlineFromClosestColor = outlineFromClosestColor;
}

OgreOutline::~OgreOutline()
{
  this->Destroy();
}

void OgreOutline::SetCamera(Ogre::Camera* camera)
{
  this->ogreCamera = camera;
}

void OgreOutline::CreateRenderPass()
{
  if (!this->ogreCamera)
  {
    Ogre::LogManager::getSingleton().logMessage("No camera set for applying Outline Pass", Ogre::LML_CRITICAL);
    return;
  }

  if (this->dataPtr->distortionInstance)
  {
    Ogre::LogManager::getSingleton().logMessage("Outline pass already created. ", Ogre::LML_CRITICAL);
    return;
  }

  this->dataPtr->distortionMaterial = Ogre::MaterialManager::getSingleton().getByName("Outline/PostProcess");

  const auto params = this->dataPtr->distortionMaterial->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
  params->setNamedConstant("outline_color", this->dataPtr->outlineColor);
  params->setNamedConstant("use_closest_color", static_cast<int>(this->dataPtr->outlineFromClosestColor));

  const auto blurVMat = Ogre::MaterialManager::getSingleton().getByName("Outline/BlurV");
  const auto blurVParams = blurVMat->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
  blurVParams->setNamedConstant("outline_width", this->dataPtr->outlineWidth);

  const auto blurHMat = Ogre::MaterialManager::getSingleton().getByName("Outline/BlurH");
  const auto blurHParams = blurHMat->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
  blurHParams->setNamedConstant("outline_width", this->dataPtr->outlineWidth);

  // create compositor instance
  this->dataPtr->distortionInstance = Ogre::CompositorManager::getSingleton().addCompositor(
    this->ogreCamera->getViewport(), "Outline");
  this->dataPtr->distortionInstance->getTechnique()->getOutputTargetPass()->getPass(0)->setMaterial(
    this->dataPtr->distortionMaterial);

  this->dataPtr->distortionInstance->setEnabled(true);
}

void OgreOutline::Destroy()
{
  if (this->dataPtr->distortionInstance)
  {
    this->dataPtr->distortionInstance->setEnabled(false);
    Ogre::CompositorManager::getSingleton().removeCompositor(this->ogreCamera->getViewport(), "Outline");

    this->dataPtr->distortionInstance = nullptr;
  }
  if (!this->dataPtr->distortionMaterial.isNull())
  {
    this->dataPtr->distortionMaterial->unload();
    this->dataPtr->distortionMaterial.setNull();
  }
}

}

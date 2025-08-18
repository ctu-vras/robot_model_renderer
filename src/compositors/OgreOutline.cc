// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief OGRE compositor drawing an outline around the rendered objects.
 * \author Martin Pecka
 */

#include <robot_model_renderer/compositors/OgreOutline.hpp>

#include <OgreCamera.h>
#include <OgreCompositionPass.h>
#include <OgreCompositionTargetPass.h>
#include <OgreCompositorInstance.h>
#include <OgreCompositorManager.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterial.h>
#include <OgreTechnique.h>

#include <cras_cpp_common/log_utils.h>

#include <robot_model_renderer/ogre_helpers/compositor.hpp>

namespace robot_model_renderer
{

struct OgreOutline::Implementation
{
  //! \brief Outline compositor.
  Ogre::CompositorInstance* outlineInstance = nullptr;

  //! \brief Ogre Material that contains the outline shader
  Ogre::MaterialPtr outlineMaterial;

  Ogre::MaterialPtr blurVMaterial;

  Ogre::MaterialPtr blurHMaterial;

  //! \brief Width of the outline.
  float outlineWidth {5.0f};

  Ogre::ColourValue outlineColor {Ogre::ColourValue::Black};

  bool outlineFromClosestColor {false};
};

OgreOutline::OgreOutline(const cras::LogHelperPtr& log,
  const double outlineWidth, const Ogre::ColourValue& outlineColor, const bool outlineFromClosestColor)
  : cras::HasLogger(log), dataPtr(std::make_unique<Implementation>())
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

void OgreOutline::SetOutlineWidth(const float width)
{
  this->dataPtr->outlineWidth = width;
}

void OgreOutline::CreateRenderPass()
{
  if (!this->ogreCamera)
  {
    CRAS_ERROR_NAMED("compositors.outline", "No camera set for applying Outline Pass");
    return;
  }

  if (this->dataPtr->outlineInstance)
  {
    CRAS_ERROR_NAMED("compositors.outline", "Outline pass already created. ");
    return;
  }

  this->dataPtr->outlineMaterial = Ogre::MaterialManager::getSingleton().getByName(
    "Outline/PostProcess")->clone("Outline/PostProcess/" + this->ogreCamera->getName());

  const auto params = this->dataPtr->outlineMaterial->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
  params->setNamedConstant("outline_color", this->dataPtr->outlineColor);
  params->setNamedConstant("use_closest_color", static_cast<int>(this->dataPtr->outlineFromClosestColor));

  this->dataPtr->blurVMaterial = Ogre::MaterialManager::getSingleton().getByName("Outline/BlurV")->clone(
    "Outline/BlurV/" + this->ogreCamera->getName());
  const auto blurVParams = this->dataPtr->blurVMaterial->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
  blurVParams->setNamedConstant("outline_width", this->dataPtr->outlineWidth);

  this->dataPtr->blurHMaterial = Ogre::MaterialManager::getSingleton().getByName("Outline/BlurH")->clone(
    "Outline/BlurH/" + this->ogreCamera->getName());
  const auto blurHParams = this->dataPtr->blurHMaterial->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
  blurHParams->setNamedConstant("outline_width", this->dataPtr->outlineWidth);

  // create compositor instance
  this->dataPtr->outlineInstance = Ogre::CompositorManager::getSingleton().addCompositor(
    this->ogreCamera->getViewport(), "Outline");

  getPass(this->dataPtr->outlineInstance, "", 12345)->setMaterial(this->dataPtr->outlineMaterial);
  getPass(this->dataPtr->outlineInstance, "rt0", 12345)->setMaterial(this->dataPtr->blurVMaterial);
  getPass(this->dataPtr->outlineInstance, "blurred", 12345)->setMaterial(this->dataPtr->blurHMaterial);

  this->dataPtr->outlineInstance->setEnabled(true);
}

void OgreOutline::Destroy()
{
  if (this->dataPtr->outlineInstance)
  {
    this->dataPtr->outlineInstance->setEnabled(false);
    Ogre::CompositorManager::getSingleton().removeCompositor(this->ogreCamera->getViewport(), "Outline");

    this->dataPtr->outlineInstance = nullptr;
  }
  if (!this->dataPtr->outlineMaterial.isNull())
  {
    Ogre::MaterialManager::getSingleton().remove(this->dataPtr->outlineMaterial->getName());
    this->dataPtr->outlineMaterial.setNull();
  }
  if (!this->dataPtr->blurVMaterial.isNull())
  {
    Ogre::MaterialManager::getSingleton().remove(this->dataPtr->blurVMaterial->getName());
    this->dataPtr->blurVMaterial.setNull();
  }
  if (!this->dataPtr->blurHMaterial.isNull())
  {
    Ogre::MaterialManager::getSingleton().remove(this->dataPtr->blurHMaterial->getName());
    this->dataPtr->blurHMaterial.setNull();
  }
}

}

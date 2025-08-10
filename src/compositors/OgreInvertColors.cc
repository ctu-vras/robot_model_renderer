// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief OGRE compositor inverting colors.
 * \author Martin Pecka
 */

#include <robot_model_renderer/compositors/OgreInvertColors.hh>

#include <OgreCamera.h>
#include <OgreCompositionPass.h>
#include <OgreCompositionTargetPass.h>
#include <OgreCompositorInstance.h>
#include <OgreCompositorManager.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterial.h>
#include <OgreTechnique.h>

#include <cras_cpp_common/log_utils.h>

namespace robot_model_renderer
{

struct OgreInvertColors::Implementation
{
  //! \brief Compositor.
  Ogre::CompositorInstance* compositorInstance = nullptr;

  //! \brief Ogre Material that contains the shader.
  Ogre::MaterialPtr material;

  //! \brief Whether to also invert the alpha channel.
  bool invertAlpha {false};
};

OgreInvertColors::OgreInvertColors(const cras::LogHelperPtr& log, const bool invertAlpha)
  : cras::HasLogger(log), dataPtr(std::make_unique<Implementation>())
{
  this->dataPtr->invertAlpha = invertAlpha;
}

OgreInvertColors::~OgreInvertColors()
{
  this->Destroy();
}

void OgreInvertColors::SetCamera(Ogre::Camera* camera)
{
  this->ogreCamera = camera;
}

void OgreInvertColors::CreateRenderPass()
{
  if (!this->ogreCamera)
  {
    CRAS_ERROR_NAMED("compositors.invert_colors", "No camera set for applying InvertColors Pass");
    return;
  }

  if (this->dataPtr->compositorInstance)
  {
    CRAS_ERROR_NAMED("compositors.invert_colors", "InvertColors pass already created. ");
    return;
  }

  this->dataPtr->material = Ogre::MaterialManager::getSingleton().getByName("InvertColor");

  const auto params = this->dataPtr->material->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
  params->setNamedConstant("invert_alpha", static_cast<int>(this->dataPtr->invertAlpha));

  // create compositor instance
  this->dataPtr->compositorInstance = Ogre::CompositorManager::getSingleton().addCompositor(
    this->ogreCamera->getViewport(), "InvertColor");
  this->dataPtr->compositorInstance->getTechnique()->getOutputTargetPass()->getPass(0)->setMaterial(
    this->dataPtr->material);

  this->dataPtr->compositorInstance->setEnabled(true);
}

void OgreInvertColors::Destroy()
{
  if (this->dataPtr->compositorInstance)
  {
    this->dataPtr->compositorInstance->setEnabled(false);
    Ogre::CompositorManager::getSingleton().removeCompositor(this->ogreCamera->getViewport(), "InvertColor");

    this->dataPtr->compositorInstance = nullptr;
  }
  if (!this->dataPtr->material.isNull())
  {
    this->dataPtr->material->unload();
    this->dataPtr->material.setNull();
  }
}

}

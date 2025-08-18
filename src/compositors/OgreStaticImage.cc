// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief OGRE compositor drawing a static image under or over the scene.
 * \author Martin Pecka
 */

#include <robot_model_renderer/compositors/OgreStaticImage.hpp>

#include <opencv2/core/mat.hpp>

#include <OgreCamera.h>
#include <OgreCompositionPass.h>
#include <OgreCompositionTargetPass.h>
#include <OgreCompositorInstance.h>
#include <OgreCompositorManager.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreImage.h>
#include <OgreMaterial.h>
#include <OgreTechnique.h>

#include <cras_cpp_common/log_utils.h>
#include <robot_model_renderer/ogre_helpers/compositor.hpp>
#include <robot_model_renderer/types.hpp>

namespace robot_model_renderer
{

struct OgreStaticImage::Implementation
{
  //! \brief Distortion compositor.
  Ogre::CompositorInstance* instance = nullptr;

  //! \brief Ogre Material that contains the shader
  Ogre::MaterialPtr material;

  Ogre::Image textureImage;

  //! \brief The texture with the static image converted to scene pixel format.
  Ogre::TexturePtr texture;

  //! \brief The texture with the static image in its original texture format
  Ogre::TexturePtr origTexture;

  cv::Mat image;

  Ogre::PixelFormat format {Ogre::PF_UNKNOWN};

  Ogre::PixelFormat sceneFormat {Ogre::PF_UNKNOWN};

  bool isBackground {true};

  RenderingMode renderingMode {RenderingMode::NORMAL};

  Ogre::ColourValue colorModeColor {Ogre::ColourValue(0.0, 0.0, 0.0, 0.0)};
};

OgreStaticImage::OgreStaticImage(const cras::LogHelperPtr& log, const cv::Mat& staticImage,
  const Ogre::PixelFormat staticImageFormat, const Ogre::PixelFormat sceneFormat, const bool staticImageIsBackground,
  const RenderingMode renderingMode, const Ogre::ColourValue& colorModeColor) :
  cras::HasLogger(log), dataPtr(std::make_unique<Implementation>())
{
  this->dataPtr->image = staticImage;
  this->dataPtr->format = staticImageFormat;
  this->dataPtr->sceneFormat = sceneFormat;
  this->dataPtr->isBackground = staticImageIsBackground;
  this->dataPtr->renderingMode = renderingMode;
  this->dataPtr->colorModeColor = colorModeColor;
}

OgreStaticImage::~OgreStaticImage()
{
  this->Destroy();
}

void OgreStaticImage::SetCamera(Ogre::Camera* camera)
{
  this->ogreCamera = camera;
}

/**
 * \brief Convert a RGBA color to the value that will be used in shaders that use the given pixel format.
 *
 * E.g. The color is `(0, 0, 0, 0)`, but shader will be PF_L8, so this color will be drawn there as `(0, 0, 0, 1)`
 * because the pixel format has no representatoin of alpha (so it sets alpha = 1 to all colors).
 *
 * \param[in] color The RGBA color to convert.
 * \param[in] format The format in which the color will be used.
 * \return The corresponding color that will be used in the given format.
 */
Ogre::ColourValue convertColor(const Ogre::ColourValue& color, const Ogre::PixelFormat& format)
{
  double tmp[4];  // Largest possible color storage for any pixel formats
  Ogre::PixelUtil::packColour(color, format, tmp);
  Ogre::ColourValue pfColor;
  Ogre::PixelUtil::unpackColour(&pfColor, format, tmp);
  return pfColor;
}

void OgreStaticImage::CreateRenderPass()
{
  if (!this->ogreCamera)
  {
    CRAS_ERROR_NAMED("compositors.static_image", "No camera set for applying Static Image Pass");
    return;
  }

  if (this->dataPtr->instance)
  {
    CRAS_ERROR_NAMED("compositors.static_image", "Static Image pass already created. ");
    return;
  }

  if (this->dataPtr->image.total() == 0)
  {
    CRAS_ERROR_NAMED("compositors.static_image", "Empty image given to Static Image pass.");
    return;
  }

  this->dataPtr->material = Ogre::MaterialManager::getSingleton().getByName("StaticImageMat")->clone(
    "StaticImageMat/" + this->ogreCamera->getName());

  const auto params = this->dataPtr->material->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
  params->setNamedConstant("is_background", this->dataPtr->isBackground);
  params->setNamedConstant("background_color",
    convertColor(this->ogreCamera->getViewport()->getBackgroundColour(), this->dataPtr->format));
  params->setNamedConstant("rendering_mode", static_cast<int>(this->dataPtr->renderingMode));
  params->setNamedConstant("color_mode_color", convertColor(this->dataPtr->colorModeColor, this->dataPtr->sceneFormat));

  // initialize the texture
  const auto texName = this->ogreCamera->getName() + "_staticImageTex";
  this->dataPtr->texture = Ogre::TextureManager::getSingleton().createManual(
    texName, "General", Ogre::TEX_TYPE_2D, this->dataPtr->image.cols, this->dataPtr->image.rows, 1, 0,
    this->dataPtr->sceneFormat, Ogre::TU_STATIC_WRITE_ONLY);

  const auto origTexName = texName + "Orig";
  this->dataPtr->origTexture = Ogre::TextureManager::getSingleton().createManual(
    origTexName, "General", Ogre::TEX_TYPE_2D, this->dataPtr->image.cols, this->dataPtr->image.rows, 1, 0,
    this->dataPtr->format, Ogre::TU_STATIC_WRITE_ONLY);

  this->dataPtr->textureImage.loadDynamicImage(
    this->dataPtr->image.data, this->dataPtr->image.cols, this->dataPtr->image.rows, 1, this->dataPtr->format);
  this->dataPtr->texture->loadImage(this->dataPtr->textureImage);
  this->dataPtr->origTexture->loadImage(this->dataPtr->textureImage);

  // set up the texture to be used in the pixel shader.
  this->dataPtr->material->getTechnique(0)->getPass(0)->createTextureUnitState(texName, 1);
  this->dataPtr->material->getTechnique(0)->getPass(0)->createTextureUnitState(origTexName, 2);

  // create compositor instance
  this->dataPtr->instance = Ogre::CompositorManager::getSingleton().addCompositor(
    this->ogreCamera->getViewport(), "StaticImage");
  getPass(this->dataPtr->instance, "", 12345)->setMaterial(this->dataPtr->material);

  this->dataPtr->instance->setEnabled(true);
}

void OgreStaticImage::Destroy()
{
  if (this->dataPtr->instance)
  {
    this->dataPtr->instance->setEnabled(false);
    Ogre::CompositorManager::getSingleton().removeCompositor(this->ogreCamera->getViewport(), "StaticImage");

    this->dataPtr->instance = nullptr;
  }
  if (!this->dataPtr->material.isNull())
  {
    Ogre::MaterialManager::getSingleton().remove(this->dataPtr->material->getName());
    this->dataPtr->material.setNull();
  }
  if (!this->dataPtr->texture.isNull())
  {
    Ogre::TextureManager::getSingleton().remove(this->dataPtr->texture->getHandle());
    this->dataPtr->texture.setNull();
  }
  if (!this->dataPtr->origTexture.isNull())
  {
    Ogre::TextureManager::getSingleton().remove(this->dataPtr->origTexture->getHandle());
    this->dataPtr->origTexture.setNull();
  }
}

}

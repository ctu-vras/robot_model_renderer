// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

/**
 * \file
 * \brief OGRE compositor drawing a static image under or over the scene.
 * \author Martin Pecka
 */

#include <memory>

#include <opencv2/core/mat.hpp>

#include <OgrePixelFormat.h>

#include <cras_cpp_common/log_utils.h>

#include <robot_model_renderer/types.hpp>

namespace Ogre
{

class Camera;

}

namespace robot_model_renderer
{

/**
 * \brief Ogre implementation static image compositor.
 */
class OgreStaticImage : public cras::HasLogger
{
public:
  OgreStaticImage(const cras::LogHelperPtr& log, const cv::Mat& staticImage, Ogre::PixelFormat staticImageFormat,
    Ogre::PixelFormat sceneFormat, bool staticImageIsBackground, RenderingMode renderingMode,
    const Ogre::ColourValue& colorModeColor);

  virtual ~OgreStaticImage();

  /**
   * \brief Destroy the pass and unregister it where needed.
   */
  void Destroy();

  /**
   * \brief Create the render pass.
   */
  void CreateRenderPass();

  /**
   * \brief Set the ogre camera that the render pass applies to
   *
   * \param[in] camera Pointer to the ogre camera.
   */
  virtual void SetCamera(Ogre::Camera* camera);

protected:
  Ogre::Camera* ogreCamera = nullptr;  //!< Pointer to the ogre camera

  struct Implementation;
  std::unique_ptr<Implementation> dataPtr;  //!< PIMPL data
};

}

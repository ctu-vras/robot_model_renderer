// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

/**
 * \file
 * \brief OGRE compositor inverting colors.
 * \author Martin Pecka
 */

#include <memory>

#include <cras_cpp_common/log_utils.h>

namespace Ogre
{

class Camera;

}

namespace robot_model_renderer
{

/**
 * \brief Ogre implementation of color inversion shader.
 */
class OgreInvertColors : public cras::HasLogger
{
public:
  explicit OgreInvertColors(const cras::LogHelperPtr& log, bool invertAlpha = false);

  virtual ~OgreInvertColors();

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

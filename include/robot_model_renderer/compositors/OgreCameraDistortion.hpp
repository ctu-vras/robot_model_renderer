// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

/**
 * \file
 * \brief OGRE compositor for camera distortion effect.
 * \author Martin Pecka
 */

#include <memory>

#include <cras_cpp_common/log_utils.h>
#include <robot_model_renderer/pinhole_camera.hpp>

namespace Ogre
{

class Camera;

}

namespace robot_model_renderer
{

/**
 * \brief Ogre implementation of camera distortion pass.
 */
class OgreCameraDistortion : public cras::HasLogger
{
public:
  /**
   * \brief Create a distortion pass that applies calibrated pinhole camera distortion.
   *
   * \param[in] log Logger.
   * \param[in] useDistortionMap Whether to use a distortion map in the shader or compute distortion on the fly.
   *                             The results should be the same if the inverse rectification shader is written
   *                             correctly. The distortion map should be the map returned from PinholeCameraModel.
   */
  explicit OgreCameraDistortion(const cras::LogHelperPtr& log, bool useDistortionMap = false);

  virtual ~OgreCameraDistortion();

  /**
   * \brief Destroy the pass and unregister it where needed.
   */
  void Destroy();

  /**
   * \brief Set the calibrated camera this render pass represents.
   *
   * \param[in] cam The represented calibrated camera.
   *
   * \return Whether the given camera has been considered valid and calibrated.
   */
  virtual bool SetCameraModel(const PinholeCameraModel& cam);

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

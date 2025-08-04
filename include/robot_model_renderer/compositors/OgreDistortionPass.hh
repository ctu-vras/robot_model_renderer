/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#pragma once

#include <memory>

#include <robot_model_renderer/pinhole_camera.h>

namespace Ogre
{
class Camera;
}

namespace robot_model_renderer
{

/**
 * \brief Ogre implementation of camera distortion pass.
 */
class OgreDistortionPass
{
public:
  /**
   * \brief Create a distortion pass that applies calibrated pinhole camera distortion.
   *
   * \param[in] useDistortionMap Whether to use a distortion map in the shader or compute distortion on the fly.
   *                             The results should be the same if the inverse rectification shader is written
   *                             correctly. The distortion map should be the map returned from PinholeCameraModel.
   */
  explicit OgreDistortionPass(bool useDistortionMap = false);

  virtual ~OgreDistortionPass();

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

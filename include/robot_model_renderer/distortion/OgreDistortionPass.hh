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
#include <vector>

#include <sensor_msgs/CameraInfo.h>

#include <opencv2/opencv.hpp>

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
  OgreDistortionPass();
  virtual ~OgreDistortionPass();

  void Destroy();

  virtual bool SetCameraModel(const robot_model_renderer::PinholeCameraModel& cam);

  void CreateRenderPass();

  /// \brief Set the ogre camera that the render pass applies to
  /// \param[in] _camera Pointer to the ogre camera.
  public: virtual void SetCamera(Ogre::Camera* _camera);

  /// \brief Pointer to the ogre camera
  protected: Ogre::Camera* ogreCamera = nullptr;

  struct Implementation;
  std::unique_ptr<OgreDistortionPass::Implementation> dataPtr;
};

}

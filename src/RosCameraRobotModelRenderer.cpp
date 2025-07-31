/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OgreCamera.h>
#include <OgreRenderSystem.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <cras_cpp_common/tf2_utils/interruptible_buffer.h>
#include <cv_bridge/cv_bridge.h>
#include <robot_model_renderer/pinhole_camera.h>
#include <robot_model_renderer/RobotModelRenderer.h>
#include <robot_model_renderer/RosCameraRobotModelRenderer.h>
#include <robot_model_renderer/utils/sensor_msgs_ogre.h>
#include <robot_model_renderer/utils/validate_floats.h>
#include <tf2_ros/buffer.h>

namespace robot_model_renderer
{

RosCameraRobotModelRenderer::RosCameraRobotModelRenderer(const urdf::Model& model,
  const std::shared_ptr<cras::InterruptibleTFBuffer>& tf, const RosCameraRobotModelRendererConfig& config,
  Ogre::SceneManager* sceneManager, Ogre::SceneNode* sceneNode, Ogre::Camera* camera)
  : config(config)
{
  RobotModelRendererConfig robotConfig;
  robotConfig.pixelFormat = sensorMsgsEncodingToOgrePixelFormat(config.imageEncoding);
  robotConfig.backgroundColor = Ogre::ColourValue(
    config.backgroundColor.r, config.backgroundColor.g, config.backgroundColor.b, config.backgroundColor.a);
  robotConfig.nearClipDistance = config.nearClipDistance;
  robotConfig.farClipDistance = config.farClipDistance;
  robotConfig.visualVisible = config.visualVisible;
  robotConfig.collisionVisible = config.collisionVisible;
  robotConfig.doDistort = config.doDistort;
  robotConfig.gpuDistortion = config.gpuDistortion;

  this->linkUpdater = std::make_unique<TFLinkUpdater>(tf);
  this->renderer = std::make_unique<RobotModelRenderer>(model, this->linkUpdater.get(),
    robotConfig, sceneManager, sceneNode, camera);
}

RosCameraRobotModelRenderer::~RosCameraRobotModelRenderer() = default;

bool RosCameraRobotModelRenderer::updateCameraInfo(const sensor_msgs::CameraInfo& msg)
{
  if (msg.height == 0 || msg.width == 0)
  {
    ROS_ERROR_THROTTLE_NAMED(1.0, "Camera Info",
      "Could not determine width/height of image due to malformed CameraInfo (either width or height is 0)");
    return false;
  }

  if (!validateFloats(msg))
  {
    ROS_ERROR_THROTTLE_NAMED(1.0, "Camera Info", "Contains invalid floating point values (nans or infs)");
    return false;
  }

  if (msg.K[0] == 0.0 || msg.K[4] == 0 || msg.P[0] == 0 || msg.P[5] == 0)
  {
    ROS_ERROR_THROTTLE_NAMED(1.0, "Camera Info", "Camera info contains invalid intrinsic matrix.");
    return false;
  }

  return this->renderer->updateCameraInfo(PinholeCameraModel(msg));
}

sensor_msgs::ImageConstPtr RosCameraRobotModelRenderer::render(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  const auto cameraInfoValid = this->updateCameraInfo(*msg);
  if (!cameraInfoValid)
    return nullptr;

  this->linkUpdater->setFixedFrame(msg->header.frame_id);

  cv_bridge::CvImage cvImg(msg->header, this->config.imageEncoding);
  cvImg.image = this->renderer->render(msg->header.stamp);

  return cvImg.toImageMsg();  // TODO here's an unneeded memcpy, we should rather preallocate and share the buffer
}

void RosCameraRobotModelRenderer::setModel(const urdf::Model& model)
{
  this->renderer->setModel(model);
}

void RosCameraRobotModelRenderer::setNearClipDistance(const double nearClip)
{
  this->renderer->setNearClipDistance(nearClip);
}

void RosCameraRobotModelRenderer::setFarClipDistance(const double farClip)
{
  this->renderer->setFarClipDistance(farClip);
}

void RosCameraRobotModelRenderer::setVisualVisible(const bool visible)
{
  this->renderer->setVisualVisible(visible);
}

void RosCameraRobotModelRenderer::setCollisionVisible(const bool visible)
{
  this->renderer->setCollisionVisible(visible);
}

void RosCameraRobotModelRenderer::reset()
{
  this->renderer->reset();
}

}

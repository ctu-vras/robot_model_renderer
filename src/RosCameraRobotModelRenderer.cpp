// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Renderer of robot model from URDF (ROS interface).
 * \author Martin Pecka
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

std_msgs::ColorRGBA createColor(const float red, const float green, const float blue, const float alpha)
{
  std_msgs::ColorRGBA color;
  color.r = red;
  color.g = green;
  color.b = blue;
  color.a = alpha;
  return color;
}

RosCameraRobotModelRenderer::RosCameraRobotModelRenderer(
  const urdf::Model& model, const std::shared_ptr<cras::InterruptibleTFBuffer>& tf,
  const RosCameraRobotModelRendererConfig& config,
  Ogre::SceneManager* sceneManager, Ogre::SceneNode* sceneNode, Ogre::Camera* camera)
  : config(config)
{
  RobotModelRendererConfig robotConfig;
  robotConfig.pixelFormat = sensorMsgsEncodingToOgrePixelFormat(config.imageEncoding);
  robotConfig.backgroundColor = Ogre::ColourValue(
    config.backgroundColor.r, config.backgroundColor.g, config.backgroundColor.b, config.backgroundColor.a);
  robotConfig.nearClipDistance = config.nearClipDistance;
  robotConfig.farClipDistance = config.farClipDistance;
  robotConfig.doDistort = config.doDistort;
  robotConfig.gpuDistortion = config.gpuDistortion;
  robotConfig.renderingMode = config.renderingMode;
  robotConfig.colorModeColor = Ogre::ColourValue(
    config.colorModeColor.r, config.colorModeColor.g, config.colorModeColor.b, config.colorModeColor.a);
  robotConfig.drawOutline = config.drawOutline;
  robotConfig.outlineWidth = config.outlineWidth;
  robotConfig.outlineColor = Ogre::ColourValue(
    config.outlineColor.r, config.outlineColor.g, config.outlineColor.b, config.outlineColor.a);
  robotConfig.outlineFromClosestColor = config.outlineFromClosestColor;
  robotConfig.invertColors = config.invertColors;
  robotConfig.invertAlpha = config.invertAlpha;
  robotConfig.shapeFilter = config.shapeFilter;
  robotConfig.shapeInflationRegistry = config.shapeInflationRegistry;

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

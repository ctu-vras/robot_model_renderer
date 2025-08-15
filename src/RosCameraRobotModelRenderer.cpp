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

#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/tf2_utils/interruptible_buffer.h>
#include <cv_bridge/cv_bridge.h>
#include <robot_model_renderer/pinhole_camera.hpp>
#include <robot_model_renderer/RobotModelRenderer.hpp>
#include <robot_model_renderer/RosCameraRobotModelRenderer.hpp>
#include <robot_model_renderer/utils/sensor_msgs_ogre.hpp>
#include <robot_model_renderer/utils/validate_floats.hpp>
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
  const cras::LogHelperPtr& log, const urdf::Model& model, const std::shared_ptr<cras::InterruptibleTFBuffer>& tf,
  RobotErrors& errors, const RosCameraRobotModelRendererConfig& config,
  Ogre::SceneManager* sceneManager, Ogre::SceneNode* sceneNode, Ogre::Camera* camera)
  : cras::HasLogger(log), config(config)
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
  robotConfig.allLinksRequired = config.allLinksRequired;
  robotConfig.requiredLinks = config.requiredLinks;
  robotConfig.upscalingInterpolation = config.upscalingInterpolation;

  this->linkUpdater = std::make_unique<TFROSLinkUpdater>(this->log, tf, "", "", config.tfTimeout);
  this->renderer = std::make_unique<RobotModelRenderer>(this->log, model, this->linkUpdater.get(),
    errors, robotConfig, sceneManager, sceneNode, camera);
}

RosCameraRobotModelRenderer::~RosCameraRobotModelRenderer() = default;

bool RosCameraRobotModelRenderer::updateCameraInfo(const sensor_msgs::CameraInfo& msg)
{
  if (msg.height == 0 || msg.width == 0)
  {
    CRAS_ERROR_THROTTLE_NAMED(1.0, "camera_info",
      "Could not determine width/height of image due to malformed CameraInfo (either width or height is 0)");
    return false;
  }

  if (!validateFloats(msg))
  {
    CRAS_ERROR_THROTTLE_NAMED(1.0, "camera_info", "Contains invalid floating point values (nans or infs)");
    return false;
  }

  if (msg.K[0] == 0.0 || msg.K[4] == 0 || msg.P[0] == 0 || msg.P[5] == 0)
  {
    CRAS_ERROR_THROTTLE_NAMED(1.0, "camera_info", "Camera info contains invalid intrinsic matrix.");
    return false;
  }

  return this->renderer->updateCameraInfo(PinholeCameraModel(msg));
}

cras::expected<sensor_msgs::ImageConstPtr, std::string> RosCameraRobotModelRenderer::render(
  const sensor_msgs::CameraInfo::ConstPtr& msg, RenderErrors& errors)
{
  const auto cameraInfoValid = this->updateCameraInfo(*msg);
  if (!cameraInfoValid)
    return cras::make_unexpected("Update of camera parameters failed.");

  this->linkUpdater->setFixedFrame(msg->header.frame_id);
  this->linkUpdater->setTimeoutStart(ros::Time::now());

  cv_bridge::CvImage cvImg(msg->header, this->config.imageEncoding);
  const auto maybeImage = this->renderer->render(msg->header.stamp, errors);
  if (!maybeImage.has_value())
    return cras::make_unexpected(maybeImage.error());

  cvImg.image = maybeImage.value();

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

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

/**
 * \file
 * \brief Renderer of robot model from URDF (ROS interface).
 * \author Martin Pecka
 */

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/tf2_utils/interruptible_buffer.h>
#include <robot_model_renderer/RobotModelRenderer.h>
#include <robot_model_renderer/robot/tf_link_updater.h>
#include <ros/duration.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/ColorRGBA.h>
#include <tf2_ros/buffer.h>

namespace robot_model_renderer
{

std_msgs::ColorRGBA createColor(float red, float green, float blue, float alpha);

/**
 * \brief Config of RosCameraRobotModelRenderer.
 */
struct RosCameraRobotModelRendererConfig
{
  bool setupDefaultLighting {true};

  std::string imageEncoding {sensor_msgs::image_encodings::RGBA8};
  std_msgs::ColorRGBA backgroundColor {createColor(0, 0, 0, 0)};

  bool doDistort {true};
  bool gpuDistortion {true};

  float nearClipDistance {0.03f};
  float farClipDistance {0.0f};

  RenderingMode renderingMode {RenderingMode::NORMAL};
  std_msgs::ColorRGBA colorModeColor {createColor(1, 0, 0, 1)};

  bool drawOutline {false};
  double outlineWidth {5.0};
  std_msgs::ColorRGBA outlineColor {createColor(0, 0, 0, 1)};
  bool outlineFromClosestColor {false};

  bool invertColors {false};
  bool invertAlpha {false};

  std::shared_ptr<ShapeFilter> shapeFilter {nullptr};
  std::shared_ptr<ShapeInflationRegistry> shapeInflationRegistry {nullptr};

  ros::Duration tfTimeout {0.01};
};

/**
 * \brief Renderer of robot model from URDF (ROS interface).
 */
class RosCameraRobotModelRenderer : public cras::HasLogger
{
public:
  RosCameraRobotModelRenderer(const cras::LogHelperPtr& log, const urdf::Model& model,
    const std::shared_ptr<cras::InterruptibleTFBuffer>& tf, const RosCameraRobotModelRendererConfig& config = {},
    Ogre::SceneManager* sceneManager = nullptr, Ogre::SceneNode* sceneNode = nullptr, Ogre::Camera* camera = nullptr);
  virtual ~RosCameraRobotModelRenderer();

  sensor_msgs::ImageConstPtr render(const sensor_msgs::CameraInfo::ConstPtr& msg);

  virtual void setModel(const urdf::Model& model);
  virtual void setNearClipDistance(double nearClip);
  virtual void setFarClipDistance(double farClip);
  virtual void setVisualVisible(bool visible);
  virtual void setCollisionVisible(bool visible);

  virtual void reset();

protected:
  bool updateCameraInfo(const sensor_msgs::CameraInfo& msg);

  std::unique_ptr<TFLinkUpdater> linkUpdater;
  std::unique_ptr<RobotModelRenderer> renderer;

  RosCameraRobotModelRendererConfig config;
};

}

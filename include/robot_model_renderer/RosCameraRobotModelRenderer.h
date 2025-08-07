/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <cras_cpp_common/tf2_utils/interruptible_buffer.h>
#include <robot_model_renderer/RobotModelRenderer.h>
#include <robot_model_renderer/robot/tf_link_updater.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/ColorRGBA.h>
#include <tf2_ros/buffer.h>

namespace robot_model_renderer
{

std_msgs::ColorRGBA createColor(float red, float green, float blue, float alpha);

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
};

class RosCameraRobotModelRenderer
{
public:
  RosCameraRobotModelRenderer(const urdf::Model& model, const std::shared_ptr<cras::InterruptibleTFBuffer>& tf,
    const RosCameraRobotModelRendererConfig& config = {},
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

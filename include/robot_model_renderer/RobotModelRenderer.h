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

#include <OgreCamera.h>
#include <OgrePixelFormat.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreSharedPtr.h>

#include <opencv2/core/core.hpp>

#include <robot_model_renderer/compositors/OgreDistortionPass.hh>
#include <robot_model_renderer/compositors/OgreInvertColors.hh>
#include <robot_model_renderer/compositors/OgreOutline.hh>
#include <robot_model_renderer/pinhole_camera.h>
#include <robot_model_renderer/robot/link_updater.h>
#include <robot_model_renderer/robot/robot.h>
#include <robot_model_renderer/robot/shape_filter.h>
#include <robot_model_renderer/robot/shape_inflation_registry.h>
#include <urdf/model.h>

namespace Ogre
{

class Root;
class SceneManager;
class SceneNode;
class ManualObject;
class Rectangle2D;
class Camera;

}

namespace robot_model_renderer
{

enum class RenderingMode
{
  NORMAL,
  COLOR,
  MASK,
};

struct RobotModelRendererConfig
{
  bool setupDefaultLighting {true};

  Ogre::PixelFormat pixelFormat {Ogre::PF_BYTE_RGBA};
  Ogre::ColourValue backgroundColor {0, 0, 0, 0};

  bool doDistort {true};
  bool gpuDistortion {true};

  float nearClipDistance {0.03f};
  float farClipDistance {0.0f};

  RenderingMode renderingMode {RenderingMode::NORMAL};
  Ogre::ColourValue colorModeColor {Ogre::ColourValue::Red};

  bool drawOutline {false};
  double outlineWidth {5.0};
  Ogre::ColourValue outlineColor {Ogre::ColourValue::Black};
  bool outlineFromClosestColor {false};

  bool invertColors {false};
  bool invertAlpha {false};

  std::shared_ptr<ShapeFilter> shapeFilter {nullptr};
  std::shared_ptr<ShapeInflationRegistry> shapeInflationRegistry {nullptr};
};

class RobotModelRenderer
{
public:
  RobotModelRenderer(const urdf::Model& model, const LinkUpdater* linkUpdater,
    const RobotModelRendererConfig& config = {},
    Ogre::SceneManager* sceneManager = nullptr, Ogre::SceneNode* sceneNode = nullptr, Ogre::Camera* camera = nullptr);
  virtual ~RobotModelRenderer();

  virtual void setModel(const urdf::Model& model);
  virtual void setNearClipDistance(double nearClip);
  virtual void setFarClipDistance(double farClip);
  virtual void setVisualVisible(bool visible);
  virtual void setCollisionVisible(bool visible);
  virtual void setPixelFormat(const Ogre::PixelFormat& pf);

  virtual void reset();

  virtual cv::Mat render(const ros::Time& time);
  virtual bool updateCameraInfo(const robot_model_renderer::PinholeCameraModel& model);

protected:
  virtual void updateOgreCamera();

  const LinkUpdater* linkUpdater;

  robot_model_renderer::PinholeCameraModel origCameraModel;
  robot_model_renderer::PinholeCameraModel rectifiedCameraModel;

  RobotModelRendererConfig config;

  bool isDistorted;

  std::unique_ptr<Robot> robot_;

  Ogre::SceneManager* scene_manager_ {nullptr};
  Ogre::Light* default_light_ {nullptr};
  Ogre::SceneNode* scene_node_ {nullptr};
  Ogre::TexturePtr tex_;
  Ogre::RenderTarget* rt_ {nullptr};
  Ogre::Camera* camera_ {nullptr};
  Ogre::Viewport* viewPort_ {nullptr};
  OgreDistortionPass distortionPass_;
  OgreInvertColors invertColorsPass_;
  OgreOutline outlinePass_;
  int cvImageType;
};

}

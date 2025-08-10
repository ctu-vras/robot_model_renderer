// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

/**
 * \file
 * \brief Renderer of robot model from URDF.
 * \author Martin Pecka
 */

#include <memory>
#include <string>

#include <OgreCamera.h>
#include <OgrePixelFormat.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreSharedPtr.h>

#include <cras_cpp_common/log_utils.h>
#include <robot_model_renderer/compositors/OgreCameraDistortion.hh>
#include <robot_model_renderer/compositors/OgreInvertColors.hh>
#include <robot_model_renderer/compositors/OgreOutline.hh>
#include <robot_model_renderer/pinhole_camera.h>
#include <robot_model_renderer/ogre_helpers/render_system.h>
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

/**
 * \brief Mode of robot model rendering.
 */
enum class RenderingMode
{
  NORMAL,  //!< Normal mode, visual meshes use their textures, collision meshes use red or link color.
  COLOR,  //!< All meshes are rendered with the specified color (with lighting effects).
  MASK,  //!< All meshes are rendered as a binary mask (robot = white, background = black).
};

/**
 * \brief Configuration of RobotModelRenderer.
 */
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

/**
 * \brief Renderer of robot model from URDF.
 */
class RobotModelRenderer : public cras::HasLogger
{
public:
  RobotModelRenderer(const cras::LogHelperPtr& log, const urdf::Model& model, const LinkUpdater* linkUpdater,
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

  robot_model_renderer::RenderSystem render_system_;
  Ogre::SceneManager* scene_manager_ {nullptr};
  Ogre::Light* default_light_ {nullptr};
  Ogre::SceneNode* scene_node_ {nullptr};
  Ogre::TexturePtr tex_;
  Ogre::RenderTarget* rt_ {nullptr};
  Ogre::Camera* camera_ {nullptr};
  Ogre::Viewport* viewPort_ {nullptr};
  OgreCameraDistortion distortionPass_;
  OgreInvertColors invertColorsPass_;
  OgreOutline outlinePass_;
  int cvImageType;
};

}

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

/**
 * \file
 * \brief Renderer of robot model from URDF.
 * \author Martin Pecka
 */

#include <memory>
#include <set>
#include <string>

#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

#include <OgreCamera.h>
#include <OgrePixelFormat.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreSharedPtr.h>

#include <cras_cpp_common/log_utils.h>
#include <robot_model_renderer/compositors/OgreCameraDistortion.hpp>
#include <robot_model_renderer/compositors/OgreInvertColors.hpp>
#include <robot_model_renderer/compositors/OgreOutline.hpp>
#include <robot_model_renderer/compositors/OgreStaticImage.hpp>
#include <robot_model_renderer/pinhole_camera.hpp>
#include <robot_model_renderer/ogre_helpers/render_system.hpp>
#include <robot_model_renderer/robot/link_updater.hpp>
#include <robot_model_renderer/robot/robot.hpp>
#include <robot_model_renderer/robot/shape_filter.hpp>
#include <robot_model_renderer/robot/shape_inflation_registry.hpp>
#include <robot_model_renderer/types.hpp>
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

  bool allLinksRequired {false};
  std::set<std::string> requiredLinks;

  std::shared_ptr<ShapeFilter> shapeFilter {nullptr};
  std::shared_ptr<ShapeInflationRegistry> shapeInflationRegistry {nullptr};

  cv::InterpolationFlags upscalingInterpolation {cv::INTER_LINEAR};
  double renderImageScale {1.0};
  size_t maxRenderImageSize {0u};

  cv::Mat staticMaskImage;
  std::string staticMaskImageEncoding;  //!< Encoding from sensor_msgs/image_encodings.h . If empty, BGR(A) is assumed.
  bool staticMaskIsBackground {true};  //!< If false, the static mask image will be drawn over the rendered image.
};

struct RenderErrors
{
  UpdateErrors updateErrors;

  bool hasError() const;
  std::string toString() const;
};

/**
 * \brief Renderer of robot model from URDF.
 */
class RobotModelRenderer : public cras::HasLogger
{
public:
  RobotModelRenderer(const cras::LogHelperPtr& log, const urdf::Model& model, LinkUpdater* linkUpdater,
    RobotErrors& errors, const RobotModelRendererConfig& config = {},
    Ogre::SceneManager* sceneManager = nullptr, Ogre::SceneNode* sceneNode = nullptr, Ogre::Camera* camera = nullptr);
  virtual ~RobotModelRenderer();

  virtual cras::expected<void, RobotErrors> setModel(const urdf::Model& model);
  virtual void setNearClipDistance(double nearClip);
  virtual void setFarClipDistance(double farClip);
  virtual void setVisualVisible(bool visible);
  virtual void setCollisionVisible(bool visible);
  virtual void setPixelFormat(const Ogre::PixelFormat& pf);

  virtual void reset();

  virtual cras::expected<cv::Mat, std::string> render(const ros::Time& time, RenderErrors& errors);
  virtual bool updateCameraInfo(const robot_model_renderer::PinholeCameraModel& model);

protected:
  virtual void updateOgreCamera();
  virtual cras::expected<cv::Mat, std::string> renderInner(const ros::Time& time, RenderErrors& errors);

  virtual bool hasOverlays() const;

  LinkUpdater* linkUpdater;

  robot_model_renderer::PinholeCameraModel origCameraModel;
  robot_model_renderer::PinholeCameraModel renderingCameraModel;
  robot_model_renderer::PinholeCameraModel rectifiedCameraModel;
  cv::Size idealRectifiedCameraResolution;

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

  Ogre::SceneManager* overlay_scene_manager_ {nullptr};
  Ogre::SceneNode* overlay_scene_node_ {nullptr};
  Ogre::SharedPtr<Ogre::Rectangle2D> overlay_;
  Ogre::TexturePtr overlay_tex_;
  Ogre::TexturePtr overlay_scene_tex_;
  Ogre::RenderTarget* overlay_rt_ {nullptr};
  Ogre::Camera* overlay_camera_ {nullptr};
  Ogre::Viewport* overlay_viewPort_ {nullptr};

  OgreStaticImage staticImagePass_;

  int cvImageType;
};

}

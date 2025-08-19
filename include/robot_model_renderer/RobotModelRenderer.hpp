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

#include <OgreMaterial.h>
#include <OgrePixelFormat.h>
#include <OgreRectangle2D.h>
#include <OgreSharedPtr.h>
#include <OgreTexture.h>

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

class Camera;
class Light;
class ManualObject;
class Rectangle2D;
class RenderTarget;
class Root;
class SceneManager;
class SceneNode;
class Viewport;

}

namespace robot_model_renderer
{

/**
 * \brief Configuration of RobotModelRenderer.
 */
struct RobotModelRendererConfig
{
  bool setupDefaultLighting {true};  //!< If true, a default point and ambient light will be added.

  Ogre::PixelFormat pixelFormat {Ogre::PF_BYTE_RGBA};  //!< Pixel format of the rendered image.
  Ogre::ColourValue backgroundColor {0, 0, 0, 0};  //!< Color of pixels that are not a part of the robot model.

  bool doDistort {true};  //!< Apply lens distortion to the rendered images. If false, the output images are rectified.
  bool gpuDistortion {true};  //!< Do the lens distortion on GPU.

  float nearClipDistance {0.03f};  //!< Near clip plane of the camera (meters).
  float farClipDistance {0.0f};  //!< Far clip plane of the camera (meters). 0.0 means infinity.

  RenderingMode renderingMode {RenderingMode::NORMAL};  //!< The mode of rendering.
  Ogre::ColourValue colorModeColor {Ogre::ColourValue::Red};  //!< Color of the robot that will be used in COLOR mode.

  bool drawOutline {false};  //!< Whether to draw an outline.
  double outlineWidth {5.0};  //!< Width of the outline.
  Ogre::ColourValue outlineColor {Ogre::ColourValue::Black};  //!< Color of the outline.
  bool outlineFromClosestColor {false};  //!< Whether outline color should be taken from the nearest on-robot pixel.

  bool invertColors {false};  //!< Invert RGB channel values.
  bool invertAlpha {false};  //!< Invert alpha channel values.

  //! If true, all links from URDF are required to have a valid TF for rendering to happen.
  bool allLinksRequired {false};
  std::set<std::string> requiredLinks;  //!< The list of links whose TFs are required for the rendering to happen.

  //! Filter of visuals/collisions. Non-matching shapes are not rendered.
  std::shared_ptr<ShapeFilter> shapeFilter {nullptr};
  //! Registry of scaling and padding parameters for individual links/visuals/collisions.
  std::shared_ptr<ShapeInflationRegistry> shapeInflationRegistry {nullptr};

  cv::InterpolationFlags upscalingInterpolation {cv::INTER_LINEAR};  //!< Interpolation method used of upscaling.
  double renderImageScale {1.0};  //!< If lower than 1.0, scale down the rendered image to save resources.
  size_t maxRenderImageSize {0u};  //!< If non-zero, sets the maximum size of render textures. Decreases quality.

  cv::Mat staticMaskImage;  //!< If non-empty, this image will be drawn over/below the robot model.
  std::string staticMaskImageEncoding;  //!< Encoding from sensor_msgs/image_encodings.h . If empty, BGR(A) is assumed.
  bool staticMaskIsBackground {true};  //!< If false, the static mask image will be drawn over the rendered image.

  bool renderedImageIsStatic {false};  //!< If true, cache rendered images for identical camera geometry.
};

/**
 * \brief Errors that happened during rendering.
 */
struct RenderErrors
{
  UpdateErrors updateErrors;  //!< Errors with TF updates.

  /**
   * \return Whether there is some error.
   */
  bool hasError() const;

  /**
   * \brief Convert the recorded errors into a single string.
   * \return The single string.
   */
  std::string toString() const;
};

/**
 * \brief Renderer of robot model from URDF.
 */
class RobotModelRenderer : public cras::HasLogger
{
public:
  /**
   * \brief Construct the renderer. Check `errors` after construction to see whether it was successful.
   *
   * \param[in] log Logger.
   * \param[in] model The robot model to render.
   * \param[in] linkUpdater Updater of robot link positions.
   * \param[out] errors The errors encountered during construction of this class.
   * \param[in] config Configuration of this class.
   * \param[in] sceneManager Optional scene manager to use.
   * \param[in] sceneNode Optional root scene node to use.
   * \param[in] camera Optional OGRE camera to use.
   */
  RobotModelRenderer(const cras::LogHelperPtr& log, const urdf::Model& model, LinkUpdater* linkUpdater,
    RobotErrors& errors, const RobotModelRendererConfig& config = {},
    Ogre::SceneManager* sceneManager = nullptr, Ogre::SceneNode* sceneNode = nullptr, Ogre::Camera* camera = nullptr);

  virtual ~RobotModelRenderer();

  /**
   * \brief Set a new URDF model to render.
   *
   * \param[in] model The new URDF model to render.
   * \return Possible errors that happened when setting the model.
   */
  virtual cras::expected<void, RobotErrors> setModel(const urdf::Model& model);

  /**
   * \brief Set new camera geometry affecting the rendered view.
   *
   * \note This function is efficient, so you can call it with all incoming camera infos, even on high frequencies.
   *
   * \param[in] model The new camera geometry.
   * \return Whether the camera geometry is valid and has been accepted.
   */
  virtual bool updateCameraInfo(const robot_model_renderer::PinholeCameraModel& model);

  /**
   * \brief Render the model using the last set camera info.
   *
   * \note At least one call to updateCameraInfo() is required before calling render().
   *
   * \param[in] time The time instant for which the model should be rendered.
   * \param[out] errors Non-fatal errors that happened during rendering. Check field `may_succeed_later` of the link
   *                    errors to find out if it makes sense to request rendering for this time again later, when the
   *                    link updater collects more TF information.
   * \return The rendered image, or a fatal error description.
   */
  virtual cras::expected<cv::Mat, std::string> render(const ros::Time& time, RenderErrors& errors);

  /**
   * \brief Reset the renderer state.
   */
  virtual void reset();

  /**
   * \param[in] nearClip Near clip plane of the camera (meters).
   */
  virtual void setNearClipDistance(double nearClip);

  /**
   * \param[in] farClip Far clip plane of the camera (meters). 0.0 means infinity.
   */
  virtual void setFarClipDistance(double farClip);

  /**
   * \param[in] visible Whether visual elements should be rendered.
   */
  virtual void setVisualVisible(bool visible);

  /**
   * \param[in] visible Whether collision elements should be rendered.
   */
  virtual void setCollisionVisible(bool visible);

  /**
   * \brief Set the pixel format of the rendered images.
   *
   * This affects e.g. whether the images have an alpha channel, whether they are mono or RGB, or whether the colors are
   * ordered as RGB or BGR.
   *
   * \param[in] pf The pixel format of the rendered images.
   */
  virtual void setPixelFormat(const Ogre::PixelFormat& pf);

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
  Ogre::MaterialPtr overlay_material_;
  Ogre::TexturePtr overlay_tex_;
  Ogre::TexturePtr overlay_scene_tex_;
  Ogre::RenderTarget* overlay_rt_ {nullptr};
  Ogre::Camera* overlay_camera_ {nullptr};
  Ogre::Viewport* overlay_viewPort_ {nullptr};

  OgreStaticImage staticImagePass_;

  int cvImageType;

  cv::Mat cached_image_;
};

}

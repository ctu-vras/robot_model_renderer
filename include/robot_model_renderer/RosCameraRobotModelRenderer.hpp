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

#include <opencv2/imgproc.hpp>

#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/tf2_utils/interruptible_buffer.h>
#include <robot_model_renderer/RobotModelRenderer.hpp>
#include <robot_model_renderer/robot/tf_link_updater.hpp>
#include <ros/duration.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/ColorRGBA.h>

namespace robot_model_renderer
{

std_msgs::ColorRGBA createColor(float red, float green, float blue, float alpha);

/**
 * \brief Config of RosCameraRobotModelRenderer.
 */
struct RosCameraRobotModelRendererConfig
{
  bool setupDefaultLighting {true};  //!< If true, a default point and ambient light will be added.

  std::string imageEncoding {sensor_msgs::image_encodings::RGBA8};  //!< Encoding of the rendered image.
  //! Color of pixels that are not a part of the robot model.
  std_msgs::ColorRGBA backgroundColor {createColor(0, 0, 0, 0)};

  bool doDistort {true};  //!< Apply lens distortion to the rendered images. If false, the output images are rectified.
  bool gpuDistortion {true};  //!< Do the lens distortion on GPU.

  float nearClipDistance {0.03f};  //!< Near clip plane of the camera (meters).
  float farClipDistance {0.0f};  //!< Far clip plane of the camera (meters). 0.0 means infinity.

  RenderingMode renderingMode {RenderingMode::NORMAL};  //!< The mode of rendering.
  std_msgs::ColorRGBA colorModeColor {createColor(1, 0, 0, 1)};  //!< Color of the robot that will be used in COLOR mode

  bool drawOutline {false};  //!< Whether to draw an outline.
  double outlineWidth {5.0};  //!< Width of the outline.
  std_msgs::ColorRGBA outlineColor {createColor(0, 0, 0, 1)};  //!< Color of the outline.
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

  ros::Duration tfTimeout {0.01};  //!< Maximum time to wait for gathering all required TF transforms.

  cv::InterpolationFlags upscalingInterpolation {cv::INTER_LINEAR};  //!< Interpolation method used of upscaling.
  double renderImageScale {1.0};  //!< If lower than 1.0, scale down the rendered image to save resources.
  size_t maxRenderImageSize {0u};  //!< If non-zero, sets the maximum size of render textures. Decreases quality.

  sensor_msgs::Image staticMaskImage;  //!< If non-empty, this image will be drawn over/below the robot model.
  bool staticMaskIsBackground {true};  //!< If false, the static mask image will be drawn over the rendered image.

  bool renderedImageIsStatic {false};  //!< If true, cache rendered images for identical camera geometry.
};

/**
 * \brief Renderer of robot model from URDF (ROS interface).
 */
class RosCameraRobotModelRenderer : public cras::HasLogger
{
public:
  /**
   * \brief Construct the renderer. Check `errors` after construction to see whether it was successful.
   *
   * \param[in] log Logger.
   * \param[in] model The robot model to render.
   * \param[in] tf TF transform buffer.
   * \param[out] errors The errors encountered during construction of this class.
   * \param[in] config Configuration of this class.
   * \param[in] sceneManager Optional scene manager to use.
   * \param[in] sceneNode Optional root scene node to use.
   * \param[in] camera Optional OGRE camera to use.
   */
  RosCameraRobotModelRenderer(const cras::LogHelperPtr& log, const urdf::Model& model,
    const std::shared_ptr<cras::InterruptibleTFBuffer>& tf, RobotErrors& errors,
    const RosCameraRobotModelRendererConfig& config = {},
    Ogre::SceneManager* sceneManager = nullptr, Ogre::SceneNode* sceneNode = nullptr, Ogre::Camera* camera = nullptr);

  virtual ~RosCameraRobotModelRenderer();

  /**
   * \brief Set a new URDF model to render.
   *
   * \param[in] model The new URDF model to render.
   * \return Possible errors that happened when setting the model.
   */
  virtual cras::expected<void, RobotErrors> setModel(const urdf::Model& model);

  /**
   * \brief Render the model using the given camera info.
   *
   * \param[in] msg The camera info using which the model should be rendered.
   * \param[out] errors Non-fatal errors that happened during rendering. Check field `may_succeed_later` of the link
   *                    errors to find out if it makes sense to request rendering for this time again later, when the
   *                    link updater collects more TF information.
   * \return The rendered image, or a fatal error description.
   */
  cras::expected<sensor_msgs::ImageConstPtr, std::string> render(
    const sensor_msgs::CameraInfo::ConstPtr& msg, RenderErrors& errors);

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

protected:
  bool updateCameraInfo(const sensor_msgs::CameraInfo& msg);

  std::unique_ptr<TFROSLinkUpdater> linkUpdater;
  std::unique_ptr<RobotModelRenderer> renderer;

  RosCameraRobotModelRendererConfig config;
};

}

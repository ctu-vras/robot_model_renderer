// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

/**
 * \file
 * \brief C API for RobotModelRenderer
 * \author Martin Pecka
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * \brief Filter of shapes (visual/collision elements).
 *
 * The methods ignoreShapes and onlyShapes accept "shape name templates". These can be:
 *
 * - `link` (match all shapes in the link with name `link`)
 * - `*::shape` (match all shapes with name `shape` in any link)
 * - `link::shape#` (match the `shape#`-th visual or collision in the given link)
 * - `link::shape` (match shape with name `shape` in link with name `link`)
 * - `*:visual:shape` (match visual with name `shape` in any link)
 * - `link:visual:shape#` (match `shape#`-th visual in link with name `link`)
 * - `link:visual:shape` (match visual with name `shape` in link with name `link`)
 * - `*:collision:shape` (match collision with name `shape` in any link)
 * - `link:collision:shape#` (match `shape#`-th collision in link with name `link`)
 * - `link:collision:shape` (match collision with name `shape` in link with name `link`)
 */
struct robot_model_renderer_ShapeFilterConfig
{
  bool visualAllowed;  //!< Whether visual elements are generally allowed.
  bool collisionAllowed;  //!< Whether collision elements are generally allowed.
  const char* ignoreShapes;  //!< "Shape name templates" of the shapes which will be ignored. Comma-separated string.
  //! "Shape name templates" of white-listed shapes that will be allowed. Comma-separated string.
  const char* onlyShapes;
};

/**
 * \brief A structure holding the configuration of scaling and padding for a single shape.
 */
struct robot_model_renderer_ScaleAndPadding
{
  double scale;  //!< Scaling (1.0 = no scaling).
  double padding;  //!< Padding (meters).
};

/**
 * \brief Scaling and padding configuration for a single shape name template.
 */
struct robot_model_renderer_PerShapeInflation
{
  const char* shapeName;  //!< Shape name template.
  robot_model_renderer_ScaleAndPadding inflation;  //!< The applied scaling and padding.
};

/**
 * \brief A registry of scaling and padding settings for shapes (visual/collision elements).
 *
 * The methods in this class accept "shape name templates". These can be:
 *
 * - `link` (match all shapes in the link with name `link`)
 * - `*::shape` (match all shapes with name `shape` in any link)
 * - `link::shape#` (match the `shape#`-th visual or collision in the given link)
 * - `link::shape` (match shape with name `shape` in link with name `link`)
 * - `*:visual:shape` (match visual with name `shape` in any link)
 * - `link:visual:shape#` (match `shape#`-th visual in link with name `link`)
 * - `link:visual:shape` (match visual with name `shape` in link with name `link`)
 * - `*:collision:shape` (match collision with name `shape` in any link)
 * - `link:collision:shape#` (match `shape#`-th collision in link with name `link`)
 * - `link:collision:shape` (match collision with name `shape` in link with name `link`)
 */
struct robot_model_renderer_ShapeInflationRegistry
{
  //! Default scale and padding to be used for shapes without an explicit override.
  robot_model_renderer_ScaleAndPadding defaultInflation;

  //! Default scale and padding to be used for visuals without an explicit override.
  robot_model_renderer_ScaleAndPadding defaultVisualInflation;

  //! Default scale and padding to be used for collisions without an explicit override.
  robot_model_renderer_ScaleAndPadding defaultCollisionInflation;

  size_t perShapeInflationCount;  //!< The number of perShapeInflation array elements.
  robot_model_renderer_PerShapeInflation* perShapeInflation;  //!< The configured per-shape scale and padding overrides.
};

/**
 * \brief Configuration of RobotModelRenderer.
 */
struct robot_model_renderer_RobotModelRendererConfig
{
  bool setupDefaultLighting;  //!< If true, a default point and ambient light will be added.

  int pixelFormat;  //!< Pixel format of the rendered image (one of Ogre::PF_* constants).
  float backgroundColor[4];  //!< Color of pixels that are not a part of the robot model (RGBA, 0-1).

  bool doDistort;  //!< Apply lens distortion to the rendered images. If false, the output images are rectified.
  bool gpuDistortion;  //!< Do the lens distortion on GPU.

  float nearClipDistance;  //!< Near clip plane of the camera (meters).
  float farClipDistance;  //!< Far clip plane of the camera (meters). 0.0 means infinity.

  int renderingMode;  //!< The mode of rendering (one of robot_model_renderer::RenderingMode constants).
  float colorModeColor[4];  //!< Color of the robot that will be used in COLOR mode (RGBA, 0-1).

  bool drawOutline;  //!< Whether to draw an outline.
  double outlineWidth;  //!< Width of the outline.
  float outlineColor[4];  //!< Color of the outline (RGBA, 0-1).
  bool outlineFromClosestColor;  //!< Whether outline color should be taken from the nearest on-robot pixel.

  bool invertColors;  //!< Invert RGB channel values.
  bool invertAlpha;  //!< Invert alpha channel values.

  //! If true, all links from URDF are required to have a valid TF for rendering to happen.
  bool allLinksRequired;
  //! The list of links whose TFs are required for the rendering to happen. Comma-separated string.
  const char* requiredLinks;

  //! Filter of visuals/collisions. Non-matching shapes are not rendered.
  robot_model_renderer_ShapeFilterConfig shapeFilter;
  //! Registry of scaling and padding parameters for individual links/visuals/collisions.
  robot_model_renderer_ShapeInflationRegistry shapeInflationRegistry;

  int upscalingInterpolation;  //!< Interpolation method used of upscaling.
  double renderImageScale;  //!< If lower than 1.0, scale down the rendered image to save resources.
  size_t maxRenderImageSize;  //!< If non-zero, sets the maximum size of render textures. Decreases quality.

  size_t staticMaskImageWidth;  //!< Width of the static mask image (0 = no image). In pixels.
  size_t staticMaskImageHeight;  //!< Height of the static mask image (0 = no image). In pixels.
  size_t staticMaskImageStep;  //!< Step of the static mask image (0 = no image). In bytes.
  int staticMaskImageCVType;  //!< OpenCV Mat type of the static mask image.
  void* staticMaskImage;  //!< If non-null, this image will be drawn over/below the robot model.
  const char* staticMaskImageEncoding;  //!< Encoding from sensor_msgs/image_encodings.h . If empty, BGR(A) is assumed.
  bool staticMaskIsBackground;  //!< If false, the static mask image will be drawn over the rendered image.

  bool renderedImageIsStatic;  //!< If true, cache rendered images for identical camera geometry.
};

struct ros_Time
{
  uint32_t sec;
  uint32_t nsec;
};

struct std_msgs_Header
{
  uint32_t seq;
  ros_Time stamp;
  const char* frame_id;
};

struct sensor_msgs_RegionOfInterest
{
  uint32_t x_offset;
  uint32_t y_offset;
  uint32_t height;
  uint32_t width;
  uint8_t do_rectify;
};

struct sensor_msgs_CameraInfo
{
  std_msgs_Header header;
  uint32_t height;
  uint32_t width;
  const char* distortion_model;
  size_t D_count;
  double* D;
  double K[9];
  double R[9];
  double P[12];
  uint32_t binning_x;
  uint32_t binning_y;
  sensor_msgs_RegionOfInterest roi;
};

struct geometry_msgs_Transform
{
  double translation[3];
  double rotation[4];
};

struct geometry_msgs_TransformStamped
{
  std_msgs_Header header;
  const char* child_frame_id;
  geometry_msgs_Transform transform;
};

typedef void* (*cras_allocator_t)(size_t);
typedef void* robot_model_renderer_RobotModelRendererHandle;

/**
 * \brief Create a default-initialized robot_model_renderer_RobotModelRendererConfig instance.
 */
robot_model_renderer_RobotModelRendererConfig robot_model_renderer_createDefaultRobotModelRendererConfig();

/**
 * \brief Convert the given image encoding to OGRE pixel format.
 *
 * \param[in] encoding The encoding (one of sensor_msgs/image_encodings.h).
 * \param[in] error_alloc Allocator for error messages.
 * \return The OGRE pixel format (one of Ogre::PF_* constants). 0 means failure.
 */
int robot_model_renderer_sensorMsgsEncodingToOgrePixelFormat(const char* encoding, cras_allocator_t error_alloc);

/**
 * \brief Construct the renderer. Check `errorMessagesAllocator` after construction to see whether it was successful.
 *
 * \param[out] logMessagesAllocator Log message allocator.
 * \param[in] model The robot model to render (string version of the URDF file).
 * \param[out] errorMessagesAllocator Allocator for the errors encountered during construction of this class.
 * \param[in] config Configuration of this class.
 * \return A handle to the constructed class instance. Nullptr if the construction failed.
 * \note Delete this instance by calling robot_model_renderer_deleteRobotModelRenderer() when no longer needed.
 */
robot_model_renderer_RobotModelRendererHandle robot_model_renderer_createRobotModelRenderer(
  cras_allocator_t logMessagesAllocator,
  const char* model,
  cras_allocator_t errorMessagesAllocator,
  robot_model_renderer_RobotModelRendererConfig config);

/**
 * \brief Delete the instance created by robot_model_renderer_createRobotModelRenderer().
 *
 * \param[in] renderer The renderer to delete.
 */
void robot_model_renderer_deleteRobotModelRenderer(robot_model_renderer_RobotModelRendererHandle renderer);

/**
 * \brief Set a new URDF model to render.
 *
 * \param[in] renderer Handle to the renderer instance.
 * \param[in] model The new URDF model to render.
 * \param[out] errorMessagesAllocator Allocator for possible errors that happened when setting the model.
 * \return Whether setting the model succeeded.
 */
bool robot_model_renderer_RobotModelRenderer_setModel(
  robot_model_renderer_RobotModelRendererHandle renderer, const char* model, cras_allocator_t errorMessagesAllocator);

/**
 * \brief Set new camera geometry affecting the rendered view.
 *
 * \note This function is efficient, so you can call it with all incoming camera infos, even on high frequencies.
 *
 * \param[in] renderer Handle to the renderer instance.
 * \param[in] cameraInfo The new camera geometry.
 * \return Whether the camera geometry is valid and has been accepted.
 */
bool robot_model_renderer_RobotModelRenderer_updateCameraInfo(
  robot_model_renderer_RobotModelRendererHandle renderer, sensor_msgs_CameraInfo cameraInfo);

/**
 * \brief Render the model using the last set camera info.
 *
 * \note At least one call to updateCameraInfo() is required before calling render().
 *
 * \param[in] renderer Handle to the renderer instance.
 * \param[in] time The time instant for which the model should be rendered.
 * \param[out] imageStep Step of the returned image (bytes).
 * \param[out] imageDataAllocator Allocator for the rendered image.
 * \param[out] linkErrorsAllocator Allocator for non-fatal errors that happened during rendering.
 * \param[out] errorMessagesAllocator Allocator for fatal errors that happened during rendering.
 * \return Whether the rendering succeeded.
 */
bool robot_model_renderer_RobotModelRenderer_render(
  robot_model_renderer_RobotModelRendererHandle renderer, ros_Time time, size_t* imageStep,
  cras_allocator_t imageDataAllocator, cras_allocator_t linkErrorsAllocator, cras_allocator_t errorMessagesAllocator);

/**
 * \brief Add a transform.
 *
 * \param[in] renderer Handle to the renderer instance.
 * \param[in] transform The transform to set.
 * \param[in] authority Authority of the transform.
 * \param[in] isStatic Whether the transform is static or dynamic.
 * \return Whether the transform was accepted.
 */
bool robot_model_renderer_LinkUpdater_set_transform(robot_model_renderer_RobotModelRendererHandle renderer,
  geometry_msgs_TransformStamped transform, const char* authority, bool isStatic);

#ifdef __cplusplus
}
#endif

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

#ifdef __cplusplus
extern "C"
{
#endif

struct robot_model_renderer_ShapeFilterConfig
{
  bool visualAllowed;
  bool collisionAllowed;
  const char* ignoreShapes;  //!< Comma-separated string
  const char* onlyShapes;  //!< Comma-separated string
};

struct robot_model_renderer_ScaleAndPadding
{
  double scale;
  double padding;
};

struct robot_model_renderer_PerShapeInflation
{
  const char* shapeName;
  robot_model_renderer_ScaleAndPadding inflation;
};

struct robot_model_renderer_ShapeInflationRegistry
{
  robot_model_renderer_ScaleAndPadding defaultInflation;
  robot_model_renderer_ScaleAndPadding defaultVisualInflation;
  robot_model_renderer_ScaleAndPadding defaultCollisionInflation;
  size_t perShapeInflationCount;
  robot_model_renderer_PerShapeInflation* perShapeInflation;
};

/**
 * \brief Configuration of RobotModelRenderer.
 */
struct robot_model_renderer_RobotModelRendererConfig
{
  bool setupDefaultLighting;

  int pixelFormat;
  float backgroundColor[4];

  bool doDistort;
  bool gpuDistortion;

  float nearClipDistance;
  float farClipDistance;

  int renderingMode;
  float colorModeColor[4];

  bool drawOutline;
  double outlineWidth;
  float outlineColor[4];
  bool outlineFromClosestColor;

  bool invertColors;
  bool invertAlpha;

  bool allLinksRequired;
  const char* requiredLinks;  //!< Comma-separated string

  robot_model_renderer_ShapeFilterConfig shapeFilter;

  robot_model_renderer_ShapeInflationRegistry shapeInflationRegistry;

  int upscalingInterpolation;
  double renderImageScale;
  size_t maxRenderImageSize;
  bool renderedImageIsStatic;

  size_t staticMaskImageWidth;
  size_t staticMaskImageHeight;
  size_t staticMaskImageStep;
  int staticMaskImageCVType;
  void* staticMaskImage;
  const char* staticMaskImageEncoding;  //!< Encoding from sensor_msgs/image_encodings.h . If empty, BGR(A) is assumed.
  bool staticMaskIsBackground;  //!< If false, the static mask image will be drawn over the rendered image.
};

struct ros_Time
{
  unsigned int sec;
  unsigned int nsec;
};

struct std_msgs_Header
{
  unsigned int seq;
  ros_Time stamp;
  const char* frame_id;
};

struct sensor_msgs_RegionOfInterest
{
  unsigned int x_offset;
  unsigned int y_offset;
  unsigned int height;
  unsigned int width;
  bool do_rectify;
};

struct sensor_msgs_CameraInfo
{
  std_msgs_Header header;
  unsigned int height;
  unsigned int width;
  const char* distortion_model;
  size_t D_count;
  double* D;
  double K[9];
  double R[9];
  double P[12];
  unsigned int binning_x;
  unsigned int binning_y;
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

robot_model_renderer_RobotModelRendererConfig robot_model_renderer_createDefaultRobotModelRendererConfig();

int robot_model_renderer_sensorMsgsEncodingToOgrePixelFormat(const char* encoding, cras_allocator_t error_alloc);

robot_model_renderer_RobotModelRendererHandle robot_model_renderer_createRobotModelRenderer(
  cras_allocator_t logMessagesAllocator,
  const char* model,
  cras_allocator_t errorMessagesAllocator,
  robot_model_renderer_RobotModelRendererConfig config);

void robot_model_renderer_deleteRobotModelRenderer(robot_model_renderer_RobotModelRendererHandle renderer);

bool robot_model_renderer_RobotModelRenderer_setModel(
  robot_model_renderer_RobotModelRendererHandle renderer, const char* model, cras_allocator_t errorMessagesAllocator);

bool robot_model_renderer_RobotModelRenderer_updateCameraInfo(
  robot_model_renderer_RobotModelRendererHandle renderer, sensor_msgs_CameraInfo cameraInfo);

bool robot_model_renderer_RobotModelRenderer_render(
  robot_model_renderer_RobotModelRendererHandle renderer, ros_Time time, size_t* imageStep,
  cras_allocator_t imageDataAllocator, cras_allocator_t linkErrorsAllocator, cras_allocator_t errorMessagesAllocator);

bool robot_model_renderer_LinkUpdater_set_transform(robot_model_renderer_RobotModelRendererHandle renderer,
  geometry_msgs_TransformStamped transform, const char* authority, bool isStatic);

#ifdef __cplusplus
}
#endif

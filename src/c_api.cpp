// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: 2012 Willow Garage, Inc.
// SPDX-FileCopyrightText: Czech Technical University in Prague

// Parts of this file are taken from rviz

/**
 * \file
 * \brief Renderer of robot model from URDF.
 * \author Martin Pecka
 */

#include <robot_model_renderer/c_api.h>

#include <algorithm>
#include <set>
#include <string>

#include <opencv2/imgproc.hpp>

#include <cras_cpp_common/c_api.h>
#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/log_utils/memory.h>
#include <robot_model_renderer/robot/tf_link_updater.hpp>
#include <robot_model_renderer/RobotModelRenderer.hpp>
#include <robot_model_renderer/utils/sensor_msgs_ogre.hpp>

inline std::set<std::string> parseCommaSeparated(const char* orig)
{
  if (std::string(orig).empty())
    return {};
  const auto split = cras::split(orig, ",");
  return {split.begin(), split.end()};
}

inline robot_model_renderer::ScaleAndPadding convertInflation(const robot_model_renderer_ScaleAndPadding& inflation)
{
  return robot_model_renderer::ScaleAndPadding(inflation.scale, inflation.padding);
}

inline robot_model_renderer::RobotModelRendererConfig convertConfig(
  const robot_model_renderer_RobotModelRendererConfig& c)
{
  robot_model_renderer::RobotModelRendererConfig cpp;

  cpp.setupDefaultLighting = c.setupDefaultLighting;
  cpp.pixelFormat = static_cast<Ogre::PixelFormat>(c.pixelFormat);
  cpp.backgroundColor = Ogre::ColourValue(
    c.backgroundColor[0], c.backgroundColor[1], c.backgroundColor[2], c.backgroundColor[3]);
  cpp.doDistort = c.doDistort;
  cpp.gpuDistortion = c.gpuDistortion;
  cpp.nearClipDistance = c.nearClipDistance;
  cpp.farClipDistance = c.farClipDistance;
  cpp.renderingMode = static_cast<robot_model_renderer::RenderingMode>(c.renderingMode);
  cpp.colorModeColor = Ogre::ColourValue(
    c.colorModeColor[0], c.colorModeColor[1], c.colorModeColor[2], c.colorModeColor[3]);
  cpp.drawOutline = c.drawOutline;
  cpp.outlineWidth = c.outlineWidth;
  cpp.outlineColor = Ogre::ColourValue(c.outlineColor[0], c.outlineColor[1], c.outlineColor[2], c.outlineColor[3]);
  cpp.outlineFromClosestColor = c.outlineFromClosestColor;
  cpp.invertColors = c.invertColors;
  cpp.invertAlpha = c.invertAlpha;
  cpp.allLinksRequired = c.allLinksRequired;
  cpp.requiredLinks = parseCommaSeparated(c.requiredLinks);

  cpp.shapeFilter = std::make_shared<robot_model_renderer::ShapeFilter>(
    c.shapeFilter.visualAllowed, c.shapeFilter.collisionAllowed);
  cpp.shapeFilter->setIgnoreShapes(parseCommaSeparated(c.shapeFilter.ignoreShapes));
  cpp.shapeFilter->setOnlyShapes(parseCommaSeparated(c.shapeFilter.onlyShapes));

  cpp.shapeInflationRegistry = std::make_shared<robot_model_renderer::ShapeInflationRegistry>(
    convertInflation(c.shapeInflationRegistry.defaultInflation));
  cpp.shapeInflationRegistry->setDefaultVisualInflation(
    convertInflation(c.shapeInflationRegistry.defaultVisualInflation));
  cpp.shapeInflationRegistry->setDefaultCollisionInflation(
    convertInflation(c.shapeInflationRegistry.defaultCollisionInflation));
  for (size_t i = 0; i < c.shapeInflationRegistry.perShapeInflationCount; ++i)
  {
    const auto& [linkName, inflation] = c.shapeInflationRegistry.perShapeInflation[i];
    cpp.shapeInflationRegistry->addPerShapeInflation(linkName, convertInflation(inflation));
  }

  cpp.upscalingInterpolation = static_cast<cv::InterpolationFlags>(c.upscalingInterpolation);
  cpp.renderImageScale = c.renderImageScale;
  cpp.maxRenderImageSize = c.maxRenderImageSize;
  cpp.renderedImageIsStatic = c.renderedImageIsStatic;

  if (c.staticMaskImageWidth > 0 && c.staticMaskImageHeight > 0 && c.staticMaskImage != nullptr)
  {
    cpp.staticMaskImage = cv::Mat(c.staticMaskImageHeight, c.staticMaskImageWidth, c.staticMaskImageCVType,
      c.staticMaskImage, c.staticMaskImageStep);
  }
  cpp.staticMaskImageEncoding = c.staticMaskImageEncoding;
  cpp.staticMaskIsBackground = c.staticMaskIsBackground;

  return cpp;
}

robot_model_renderer_RobotModelRendererConfig robot_model_renderer_createDefaultRobotModelRendererConfig()
{
  robot_model_renderer::RobotModelRendererConfig cpp;
  robot_model_renderer_RobotModelRendererConfig c;

  c.setupDefaultLighting = cpp.setupDefaultLighting;
  c.pixelFormat = static_cast<int>(cpp.pixelFormat);
  memcpy(c.backgroundColor, &cpp.backgroundColor[0], sizeof(float) * 4);
  c.doDistort = cpp.doDistort;
  c.gpuDistortion = cpp.gpuDistortion;
  c.nearClipDistance = cpp.nearClipDistance;
  c.farClipDistance = cpp.farClipDistance;
  c.renderingMode = static_cast<int>(cpp.renderingMode);
  memcpy(c.colorModeColor, &cpp.colorModeColor[0], sizeof(float) * 4);
  c.drawOutline = cpp.drawOutline;
  c.outlineWidth = cpp.outlineWidth;
  memcpy(c.outlineColor, &cpp.outlineColor[0], sizeof(float) * 4);
  c.outlineFromClosestColor = cpp.outlineFromClosestColor;
  c.invertColors = cpp.invertColors;
  c.invertAlpha = cpp.invertAlpha;
  c.allLinksRequired = cpp.allLinksRequired;
  c.requiredLinks = "";

  cpp.shapeFilter = std::make_shared<robot_model_renderer::ShapeFilter>();
  c.shapeFilter.visualAllowed = cpp.shapeFilter->isVisualAllowed();
  c.shapeFilter.collisionAllowed = cpp.shapeFilter->isCollisionAllowed();
  c.shapeFilter.ignoreShapes = "";
  c.shapeFilter.onlyShapes = "";

  cpp.shapeInflationRegistry = std::make_shared<robot_model_renderer::ShapeInflationRegistry>();
  c.shapeInflationRegistry.defaultInflation.scale = cpp.shapeInflationRegistry->defaultInflation().scale;
  c.shapeInflationRegistry.defaultInflation.padding = cpp.shapeInflationRegistry->defaultInflation().padding;
  const auto& defInflation = cpp.shapeInflationRegistry->defaultInflation();
  c.shapeInflationRegistry.defaultVisualInflation.scale =
    cpp.shapeInflationRegistry->defaultVisualInflation().value_or(defInflation).scale;
  c.shapeInflationRegistry.defaultVisualInflation.padding =
    cpp.shapeInflationRegistry->defaultVisualInflation().value_or(defInflation).padding;
  c.shapeInflationRegistry.defaultCollisionInflation.scale =
    cpp.shapeInflationRegistry->defaultCollisionInflation().value_or(defInflation).scale;
  c.shapeInflationRegistry.defaultCollisionInflation.padding =
    cpp.shapeInflationRegistry->defaultCollisionInflation().value_or(defInflation).padding;
  c.shapeInflationRegistry.perShapeInflationCount = 0u;
  c.shapeInflationRegistry.perShapeInflation = nullptr;

  c.upscalingInterpolation = cpp.upscalingInterpolation;
  c.renderImageScale = cpp.renderImageScale;
  c.maxRenderImageSize = cpp.maxRenderImageSize;
  c.renderedImageIsStatic = cpp.renderedImageIsStatic;

  c.staticMaskImageWidth = 0u;
  c.staticMaskImageHeight = 0u;
  c.staticMaskImageStep = 0u;
  c.staticMaskImageCVType = 0;
  c.staticMaskImage = nullptr;
  c.staticMaskImageEncoding = "";
  c.staticMaskIsBackground = cpp.staticMaskIsBackground;

  return c;
}

struct robot_model_renderer_RobotModelRendererHandle_cpp
{
  cras_allocator_t logMessagesAllocator;
  std::shared_ptr<cras::MemoryLogHelper> log;
  tf2::BufferCore tf;
  std::shared_ptr<robot_model_renderer::TFLinkUpdater> linkUpdater;
  std::unique_ptr<robot_model_renderer::RobotModelRenderer> renderer;
};

class LogGuard
{
public:
  explicit LogGuard(robot_model_renderer_RobotModelRendererHandle_cpp* handle) : handle(handle)
  {
    this->handle->log->clear();
  }

  ~LogGuard()
  {
    for (const auto& msg : this->handle->log->getMessages())
      cras::outputRosMessage(this->handle->logMessagesAllocator, msg);
    this->handle->log->clear();
  }

  robot_model_renderer_RobotModelRendererHandle_cpp* handle;
};

robot_model_renderer_RobotModelRendererHandle robot_model_renderer_createRobotModelRenderer(
  cras_allocator_t logMessagesAllocator, const char* model, cras_allocator_t errorMessagesAllocator,
  robot_model_renderer_RobotModelRendererConfig config)
{
  urdf::Model robotModel;
  try
  {
    if (!robotModel.initString(model))
    {
      cras::outputString(errorMessagesAllocator, "Failed to parse URDF");
      return nullptr;
    }
  }
  catch (const urdf::ParseError& e)
  {
    cras::outputString(errorMessagesAllocator, "Failed to parse URDF");
    return nullptr;
  }

  const auto config_cpp = convertConfig(config);

  auto handle = new robot_model_renderer_RobotModelRendererHandle_cpp;

  handle->logMessagesAllocator = logMessagesAllocator;
  handle->log = std::make_shared<cras::MemoryLogHelper>();

  robot_model_renderer::RobotErrors errors;
  {
    LogGuard logGuard(handle);

    handle->linkUpdater = std::make_shared<robot_model_renderer::TFLinkUpdater>(handle->log, &handle->tf);

    handle->renderer = std::make_unique<robot_model_renderer::RobotModelRenderer>(
      handle->log, robotModel, handle->linkUpdater.get(), errors, config_cpp);

    for (const auto& e : errors.linkErrors)
    {
      cras::outputString(errorMessagesAllocator, e.error);
      for (const auto& ee : e.visualErrors)
        cras::outputString(errorMessagesAllocator, ee.error);
      for (const auto& ee : e.collisionErrors)
        cras::outputString(errorMessagesAllocator, ee.error);
    }
    for (const auto& e : errors.jointErrors)
      cras::outputString(errorMessagesAllocator, e.error);
  }

  if (errors.hasError(false))
  {
    delete handle;
    handle = nullptr;
  }

  return handle;
}

void robot_model_renderer_deleteRobotModelRenderer(robot_model_renderer_RobotModelRendererHandle renderer)
{
  const auto handle = static_cast<robot_model_renderer_RobotModelRendererHandle_cpp*>(renderer);
  delete handle;
}

bool robot_model_renderer_RobotModelRenderer_setModel(robot_model_renderer_RobotModelRendererHandle renderer,
  const char* model, const cras_allocator_t errorMessagesAllocator)
{
  const auto handle = static_cast<robot_model_renderer_RobotModelRendererHandle_cpp*>(renderer);
  LogGuard logGuard(handle);

  try
  {
    urdf::Model robotModel;
    if (robotModel.initString(model))
    {
      const auto result = handle->renderer->setModel(robotModel);
      if (result.has_value())
        return true;

      for (const auto& e : result.error().linkErrors)
      {
        cras::outputString(errorMessagesAllocator, e.error);
        for (const auto& ee : e.visualErrors)
          cras::outputString(errorMessagesAllocator, ee.error);
        for (const auto& ee : e.collisionErrors)
          cras::outputString(errorMessagesAllocator, ee.error);
      }
      for (const auto& e : result.error().jointErrors)
        cras::outputString(errorMessagesAllocator, e.error);
    }
    else
    {
      cras::outputString(errorMessagesAllocator, "Failed to parse URDF");
    }
  }
  catch (const urdf::ParseError& e)
  {
    cras::outputString(errorMessagesAllocator, "Failed to parse URDF");
  }

  return false;
}

bool robot_model_renderer_RobotModelRenderer_updateCameraInfo(
  robot_model_renderer_RobotModelRendererHandle renderer, sensor_msgs_CameraInfo cameraInfo)
{
  const auto handle = static_cast<robot_model_renderer_RobotModelRendererHandle_cpp*>(renderer);
  LogGuard logGuard(handle);

  sensor_msgs::CameraInfo camInfo;
  camInfo.header.seq = cameraInfo.header.seq;
  camInfo.header.stamp.sec = cameraInfo.header.stamp.sec;
  camInfo.header.stamp.nsec = cameraInfo.header.stamp.nsec;
  camInfo.header.frame_id = cameraInfo.header.frame_id;
  camInfo.height = cameraInfo.height;
  camInfo.width = cameraInfo.width;
  camInfo.distortion_model = cameraInfo.distortion_model;
  camInfo.D.resize(cameraInfo.D_count);
  std::copy_n(cameraInfo.D, cameraInfo.D_count, camInfo.D.data());
  std::copy_n(cameraInfo.K, 9, camInfo.K.c_array());
  std::copy_n(cameraInfo.R, 9, camInfo.R.c_array());
  std::copy_n(cameraInfo.P, 12, camInfo.P.c_array());
  camInfo.binning_x = cameraInfo.binning_x;
  camInfo.binning_y = cameraInfo.binning_y;
  camInfo.roi.x_offset = cameraInfo.roi.x_offset;
  camInfo.roi.y_offset = cameraInfo.roi.y_offset;
  camInfo.roi.height = cameraInfo.roi.height;
  camInfo.roi.width = cameraInfo.roi.width;
  camInfo.roi.do_rectify = cameraInfo.roi.do_rectify;

  const auto result = handle->renderer->updateCameraInfo(robot_model_renderer::PinholeCameraModel(camInfo));
  handle->linkUpdater->setFixedFrame(camInfo.header.frame_id);

  return result;
}

bool robot_model_renderer_RobotModelRenderer_render(robot_model_renderer_RobotModelRendererHandle renderer,
  ros_Time time, size_t* imageStep, cras_allocator_t imageDataAllocator, cras_allocator_t linkErrorsAllocator,
  cras_allocator_t errorMessagesAllocator)
{
  const auto handle = static_cast<robot_model_renderer_RobotModelRendererHandle_cpp*>(renderer);
  LogGuard logGuard(handle);

  ros::Time time_cpp(time.sec, time.nsec);

  robot_model_renderer::RenderErrors errors;
  const auto renderResult = handle->renderer->render(time_cpp, errors);

  if (errors.hasError())
    cras::outputString(errorMessagesAllocator, errors.toString());

  for (const auto& [linkName, e] : errors.updateErrors.linkErrors)
  {
    const auto length = 1 + (e.name.size() + 1) + (e.error.size() + 1);
    const auto buffer = static_cast<uint8_t*>(linkErrorsAllocator(length));

    size_t i = 0;
    buffer[i] = e.maySucceedLater;

    i += 1;
    memcpy(&buffer[i], e.name.c_str(), e.name.size() + 1);

    i += e.name.size() + 1;
    memcpy(&buffer[i], e.error.c_str(), e.error.size() + 1);
  }

  if (!renderResult.has_value())
  {
    cras::outputString(errorMessagesAllocator, renderResult.error());
    return false;
  }

  const auto& cvImage = renderResult.value();
  const auto imageSize = cvImage.total() * cvImage.elemSize();
  const auto outputRowSize = imageSize / cvImage.rows;

  *imageStep = cvImage.cols * cvImage.elemSize();

  auto data = cvImage.data;
  std::vector<uint8_t> dataContinuous;
  if (!cvImage.isContinuous())
  {
    dataContinuous.resize(imageSize);
    for (size_t i = 0; i < cvImage.rows; ++i)
      std::copy_n(&data[cvImage.step[0] * i], outputRowSize, dataContinuous.data() + outputRowSize * i);
    data = dataContinuous.data();
  }
  cras::outputByteBuffer(imageDataAllocator, data, imageSize);

  return true;
}

bool robot_model_renderer_LinkUpdater_set_transform(robot_model_renderer_RobotModelRendererHandle renderer,
  geometry_msgs_TransformStamped transform, const char* authority, const bool isStatic)
{
  const auto handle = static_cast<robot_model_renderer_RobotModelRendererHandle_cpp*>(renderer);
  LogGuard logGuard(handle);

  geometry_msgs::TransformStamped tf;
  tf.header.seq = transform.header.seq;
  tf.header.stamp.sec = transform.header.stamp.sec;
  tf.header.stamp.nsec = transform.header.stamp.nsec;
  tf.header.frame_id = transform.header.frame_id;
  tf.child_frame_id = transform.child_frame_id;
  tf.transform.translation.x = transform.transform.translation[0];
  tf.transform.translation.y = transform.transform.translation[1];
  tf.transform.translation.z = transform.transform.translation[2];
  tf.transform.rotation.x = transform.transform.rotation[0];
  tf.transform.rotation.y = transform.transform.rotation[1];
  tf.transform.rotation.z = transform.transform.rotation[2];
  tf.transform.rotation.w = transform.transform.rotation[3];

  return handle->tf.setTransform(tf, authority, isStatic);
}

int robot_model_renderer_sensorMsgsEncodingToOgrePixelFormat(const char* encoding, cras_allocator_t error_alloc)
{
  try
  {
    return robot_model_renderer::sensorMsgsEncodingToOgrePixelFormat(encoding);
  }
  catch (const std::runtime_error& e)
  {
    cras::outputString(error_alloc, std::string(e.what()));
    return Ogre::PixelFormat::PF_UNKNOWN;
  }
}

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for RobotModelRenderer
 * \author Martin Pecka
 */

#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include <cmath>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <utility>

#include <cras_cpp_common/log_utils/memory.h>
#include <cras_cpp_common/log_utils/node.h>
#include <geometry_msgs/TransformStamped.h>
#include <robot_model_renderer/RobotModelRenderer.hpp>
#include <robot_model_renderer/robot/tf_link_updater.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2/buffer_core.h>
#include <urdf/model.h>

ros::V_string my_argv;

constexpr uint32_t align(const uint32_t x, const uint32_t a)
{
  return ((x) + (a) - 1) & ~((a) - 1);
}

auto alignedStep(const uint32_t width, const uint32_t channels, const uint32_t planes = 1)
{
  const auto step = width * channels;
  return testing::AnyOf(
    testing::Eq(planes * step),
    testing::Eq(planes * align(step, 8)),
    testing::Eq(planes * align(step, 16)),
    testing::Eq(planes * align(step, 32)),
    testing::Eq(planes * align(step, 64)));
}

TEST(RobotModelRendererNodelet, Default)  // NOLINT
{
  // const auto log = std::make_shared<cras::MemoryLogHelper>();
  const auto log = std::make_shared<cras::NodeLogHelper>();

  urdf::Model model;
  model.initString("<robot name=\"test\"><link name=\"link1\"/></robot>");

  tf2::BufferCore tfBuffer;
  robot_model_renderer::TFLinkUpdater linkUpdater(log, &tfBuffer);
  linkUpdater.setFixedFrame("link1");

  robot_model_renderer::RobotModelRendererConfig config;
  config.pixelFormat = Ogre::PF_BYTE_RGBA;

  robot_model_renderer::RobotErrors errors;
  robot_model_renderer::RobotModelRenderer renderer(log, model, &linkUpdater, errors, config);
  ASSERT_FALSE(errors.hasError());

  model.clear();
  model.initFile(std::string(TEST_DATA_DIR) + "/robot.urdf");
  const auto setModelResult = renderer.setModel(model);
  ASSERT_TRUE(setModelResult.has_value());

  geometry_msgs::TransformStamped tf;
  tf.header.frame_id = "link1";
  tf.header.stamp = {1732502880, 585000000};
  tf.child_frame_id = "link2";
  tf.transform.translation.x = 0.3;
  tf.transform.translation.z = 1.0;
  tf.transform.rotation.w = 1.0;
  tfBuffer.setTransform(tf, "test", true);

  sensor_msgs::CameraInfo camInfo;
  camInfo.header.frame_id = "link1";
  camInfo.header.stamp = {1732502880, 585000000};
  camInfo.height = 1616;
  camInfo.width = 1212;
  camInfo.distortion_model = "plumb_bob";
  camInfo.D = {1.0, 0, 0, 0, 0, 0.5, 0, 0};
  camInfo.K = {800.0, 0.0, 600.0, 0.0, 800.0, 800.0, 0.0, 0.0, 1.0};
  camInfo.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  camInfo.P = {800.0, 0.0, 600.0, 0.0, 0.0, 800.0, 800.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  ASSERT_TRUE(renderer.updateCameraInfo(robot_model_renderer::PinholeCameraModel(camInfo)));

  robot_model_renderer::RenderErrors renderErrors;
  const auto renderResult = renderer.render(camInfo.header.stamp, renderErrors);
  ASSERT_TRUE(renderResult.has_value());

  const auto& cvImage = renderResult.value();
  ASSERT_EQ(2, cvImage.dims);
  ASSERT_EQ(1616, cvImage.rows);
  ASSERT_EQ(1212, cvImage.cols);
  ASSERT_EQ(4, cvImage.channels());
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(0, 0));
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(1615, 0));
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(1615, 1211));
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(0, 1211));
  EXPECT_EQ(cv::Vec4b(250, 0, 0, 255), cvImage.at<cv::Vec4b>(1616 / 2, 1212 / 2));
  EXPECT_EQ(cv::Vec4b(250, 0, 0, 255), cvImage.at<cv::Vec4b>(1616 / 2, 1211));
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(1616 / 2, 390));
}

TEST(RobotModelRendererNodelet, DefaultCpuDistortion)  // NOLINT
{
  // const auto log = std::make_shared<cras::MemoryLogHelper>();
  const auto log = std::make_shared<cras::NodeLogHelper>();

  urdf::Model model;
  model.initString("<robot name=\"test\"><link name=\"link1\"/></robot>");

  tf2::BufferCore tfBuffer;
  robot_model_renderer::TFLinkUpdater linkUpdater(log, &tfBuffer);
  linkUpdater.setFixedFrame("link1");

  robot_model_renderer::RobotModelRendererConfig config;
  config.pixelFormat = Ogre::PF_BYTE_RGBA;
  config.gpuDistortion = false;

  robot_model_renderer::RobotErrors errors;
  robot_model_renderer::RobotModelRenderer renderer(log, model, &linkUpdater, errors, config);
  ASSERT_FALSE(errors.hasError());

  model.clear();
  model.initFile(std::string(TEST_DATA_DIR) + "/robot.urdf");
  const auto setModelResult = renderer.setModel(model);
  ASSERT_TRUE(setModelResult.has_value());

  geometry_msgs::TransformStamped tf;
  tf.header.frame_id = "link1";
  tf.header.stamp = {1732502880, 585000000};
  tf.child_frame_id = "link2";
  tf.transform.translation.x = 0.3;
  tf.transform.translation.z = 1.0;
  tf.transform.rotation.w = 1.0;

  tfBuffer.setTransform(tf, "test", true);

  sensor_msgs::CameraInfo camInfo;
  camInfo.header.frame_id = "link1";
  camInfo.header.stamp = {1732502880, 585000000};
  camInfo.height = 1616;
  camInfo.width = 1212;
  camInfo.distortion_model = "plumb_bob";
  camInfo.D = {1.0, 0, 0, 0, 0, 0.5, 0, 0};
  camInfo.K = {800.0, 0.0, 600.0, 0.0, 800.0, 800.0, 0.0, 0.0, 1.0};
  camInfo.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  camInfo.P = {800.0, 0.0, 600.0, 0.0, 0.0, 800.0, 800.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  ASSERT_TRUE(renderer.updateCameraInfo(robot_model_renderer::PinholeCameraModel(camInfo)));

  robot_model_renderer::RenderErrors renderErrors;
  const auto renderResult = renderer.render(camInfo.header.stamp, renderErrors);
  ASSERT_TRUE(renderResult.has_value());

  const auto& cvImage = renderResult.value();
  ASSERT_EQ(2, cvImage.dims);
  ASSERT_EQ(1616, cvImage.rows);
  ASSERT_EQ(1212, cvImage.cols);
  ASSERT_EQ(4, cvImage.channels());
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(0, 0));
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(1615, 0));
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(1615, 1211));
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(0, 1211));
  EXPECT_EQ(cv::Vec4b(250, 0, 0, 255), cvImage.at<cv::Vec4b>(1616 / 2, 1212 / 2));
  EXPECT_EQ(cv::Vec4b(250, 0, 0, 255), cvImage.at<cv::Vec4b>(1616 / 2, 1211));
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(1616 / 2, 390));
}

TEST(RobotModelRendererNodelet, DefaultBGRA)  // NOLINT
{
  // const auto log = std::make_shared<cras::MemoryLogHelper>();
  const auto log = std::make_shared<cras::NodeLogHelper>();

  urdf::Model model;
  model.initString("<robot name=\"test\"><link name=\"link1\"/></robot>");

  tf2::BufferCore tfBuffer;
  robot_model_renderer::TFLinkUpdater linkUpdater(log, &tfBuffer);
  linkUpdater.setFixedFrame("link1");

  robot_model_renderer::RobotModelRendererConfig config;
  config.pixelFormat = Ogre::PF_BYTE_BGRA;

  robot_model_renderer::RobotErrors errors;
  robot_model_renderer::RobotModelRenderer renderer(log, model, &linkUpdater, errors, config);
  ASSERT_FALSE(errors.hasError());

  model.clear();
  model.initFile(std::string(TEST_DATA_DIR) + "/robot.urdf");
  const auto setModelResult = renderer.setModel(model);
  ASSERT_TRUE(setModelResult.has_value());

  geometry_msgs::TransformStamped tf;
  tf.header.frame_id = "link1";
  tf.header.stamp = {1732502880, 585000000};
  tf.child_frame_id = "link2";
  tf.transform.translation.x = 0.3;
  tf.transform.translation.z = 1.0;
  tf.transform.rotation.w = 1.0;

  tfBuffer.setTransform(tf, "test", true);

  sensor_msgs::CameraInfo camInfo;
  camInfo.header.frame_id = "link1";
  camInfo.header.stamp = {1732502880, 585000000};
  camInfo.height = 1616;
  camInfo.width = 1212;
  camInfo.distortion_model = "plumb_bob";
  camInfo.D = {1.0, 0, 0, 0, 0, 0.5, 0, 0};
  camInfo.K = {800.0, 0.0, 600.0, 0.0, 800.0, 800.0, 0.0, 0.0, 1.0};
  camInfo.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  camInfo.P = {800.0, 0.0, 600.0, 0.0, 0.0, 800.0, 800.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  ASSERT_TRUE(renderer.updateCameraInfo(robot_model_renderer::PinholeCameraModel(camInfo)));

  robot_model_renderer::RenderErrors renderErrors;
  const auto renderResult = renderer.render(camInfo.header.stamp, renderErrors);
  ASSERT_TRUE(renderResult.has_value());

  const auto& cvImage = renderResult.value();
  ASSERT_EQ(2, cvImage.dims);
  ASSERT_EQ(1616, cvImage.rows);
  ASSERT_EQ(1212, cvImage.cols);
  ASSERT_EQ(4, cvImage.channels());
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(0, 0));
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(1615, 0));
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(1615, 1211));
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(0, 1211));
  EXPECT_EQ(cv::Vec4b(0, 0, 250, 255), cvImage.at<cv::Vec4b>(1616 / 2, 1212 / 2));
  EXPECT_EQ(cv::Vec4b(0, 0, 250, 255), cvImage.at<cv::Vec4b>(1616 / 2, 1211));
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(1616 / 2, 390));
}

TEST(RobotModelRendererNodelet, DefaultMono)  // NOLINT
{
  // const auto log = std::make_shared<cras::MemoryLogHelper>();
  const auto log = std::make_shared<cras::NodeLogHelper>();

  urdf::Model model;
  model.initString("<robot name=\"test\"><link name=\"link1\"/></robot>");

  tf2::BufferCore tfBuffer;
  robot_model_renderer::TFLinkUpdater linkUpdater(log, &tfBuffer);
  linkUpdater.setFixedFrame("link1");

  robot_model_renderer::RobotModelRendererConfig config;
  config.pixelFormat = Ogre::PF_BYTE_L;

  robot_model_renderer::RobotErrors errors;
  robot_model_renderer::RobotModelRenderer renderer(log, model, &linkUpdater, errors, config);
  ASSERT_FALSE(errors.hasError());

  model.clear();
  model.initFile(std::string(TEST_DATA_DIR) + "/robot.urdf");
  const auto setModelResult = renderer.setModel(model);
  ASSERT_TRUE(setModelResult.has_value());

  geometry_msgs::TransformStamped tf;
  tf.header.frame_id = "link1";
  tf.header.stamp = {1732502880, 585000000};
  tf.child_frame_id = "link2";
  tf.transform.translation.x = 0.3;
  tf.transform.translation.z = 1.0;
  tf.transform.rotation.w = 1.0;

  tfBuffer.setTransform(tf, "test", true);

  sensor_msgs::CameraInfo camInfo;
  camInfo.header.frame_id = "link1";
  camInfo.header.stamp = {1732502880, 585000000};
  camInfo.height = 1616;
  camInfo.width = 1212;
  camInfo.distortion_model = "plumb_bob";
  camInfo.D = {1.0, 0, 0, 0, 0, 0.5, 0, 0};
  camInfo.K = {800.0, 0.0, 600.0, 0.0, 800.0, 800.0, 0.0, 0.0, 1.0};
  camInfo.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  camInfo.P = {800.0, 0.0, 600.0, 0.0, 0.0, 800.0, 800.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  ASSERT_TRUE(renderer.updateCameraInfo(robot_model_renderer::PinholeCameraModel(camInfo)));

  robot_model_renderer::RenderErrors renderErrors;
  const auto renderResult = renderer.render(camInfo.header.stamp, renderErrors);
  ASSERT_TRUE(renderResult.has_value());

  const auto& cvImage = renderResult.value();
  ASSERT_EQ(2, cvImage.dims);
  ASSERT_EQ(1616, cvImage.rows);
  ASSERT_EQ(1212, cvImage.cols);
  ASSERT_EQ(1, cvImage.channels());
  EXPECT_EQ(0, cvImage.at<uint8_t>(0, 0));
  EXPECT_EQ(0, cvImage.at<uint8_t>(1615, 0));
  EXPECT_EQ(0, cvImage.at<uint8_t>(1615, 1211));
  EXPECT_EQ(0, cvImage.at<uint8_t>(0, 1211));
  EXPECT_EQ(250, cvImage.at<uint8_t>(1616 / 2, 1212 / 2));
  EXPECT_EQ(250, cvImage.at<uint8_t>(1616 / 2, 1211));
  EXPECT_EQ(0, cvImage.at<uint8_t>(1616 / 2, 390));
}

TEST(RobotModelRendererNodelet, ColorModeWithOutline)  // NOLINT
{
  // const auto log = std::make_shared<cras::MemoryLogHelper>();
  const auto log = std::make_shared<cras::NodeLogHelper>();

  urdf::Model model;
  model.initString("<robot name=\"test\"><link name=\"link1\"/></robot>");

  tf2::BufferCore tfBuffer;
  robot_model_renderer::TFLinkUpdater linkUpdater(log, &tfBuffer);
  linkUpdater.setFixedFrame("link1");

  robot_model_renderer::RobotModelRendererConfig config;
  config.pixelFormat = Ogre::PF_BYTE_RGBA;
  config.renderingMode = robot_model_renderer::RenderingMode::COLOR;
  config.colorModeColor = Ogre::ColourValue(0.0f, 1.0f, 0.0f, 1.0f);
  config.drawOutline = true;
  config.outlineWidth = 20.0;
  config.outlineColor = Ogre::ColourValue(0.0f, 0.0f, 1.0f, 1.0f);

  robot_model_renderer::RobotErrors errors;
  robot_model_renderer::RobotModelRenderer renderer(log, model, &linkUpdater, errors, config);
  ASSERT_FALSE(errors.hasError());

  model.clear();
  model.initFile(std::string(TEST_DATA_DIR) + "/robot.urdf");
  const auto setModelResult = renderer.setModel(model);
  ASSERT_TRUE(setModelResult.has_value());

  geometry_msgs::TransformStamped tf;
  tf.header.frame_id = "link1";
  tf.header.stamp = {1732502880, 585000000};
  tf.child_frame_id = "link2";
  tf.transform.translation.x = 0.3;
  tf.transform.translation.z = 1.0;
  tf.transform.rotation.w = 1.0;

  tfBuffer.setTransform(tf, "test", true);

  sensor_msgs::CameraInfo camInfo;
  camInfo.header.frame_id = "link1";
  camInfo.header.stamp = {1732502880, 585000000};
  camInfo.height = 1616;
  camInfo.width = 1212;
  camInfo.distortion_model = "plumb_bob";
  camInfo.D = {1.0, 0, 0, 0, 0, 0.5, 0, 0};
  camInfo.K = {800.0, 0.0, 600.0, 0.0, 800.0, 800.0, 0.0, 0.0, 1.0};
  camInfo.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  camInfo.P = {800.0, 0.0, 600.0, 0.0, 0.0, 800.0, 800.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  ASSERT_TRUE(renderer.updateCameraInfo(robot_model_renderer::PinholeCameraModel(camInfo)));

  robot_model_renderer::RenderErrors renderErrors;
  const auto renderResult = renderer.render(camInfo.header.stamp, renderErrors);
  ASSERT_TRUE(renderResult.has_value());

  const auto& cvImage = renderResult.value();
  ASSERT_EQ(2, cvImage.dims);
  ASSERT_EQ(1616, cvImage.rows);
  ASSERT_EQ(1212, cvImage.cols);
  ASSERT_EQ(4, cvImage.channels());
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(0, 0));
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(1615, 0));
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(1615, 1211));
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(0, 1211));
  EXPECT_EQ(cv::Vec4b(0, 250, 0, 255), cvImage.at<cv::Vec4b>(1616 / 2, 1212 / 2));
  EXPECT_EQ(cv::Vec4b(0, 250, 0, 255), cvImage.at<cv::Vec4b>(1616 / 2, 1211));
  EXPECT_EQ(cv::Vec4b(0, 0, 255, 255), cvImage.at<cv::Vec4b>(1616 / 2, 390));
}

TEST(RobotModelRendererNodelet, MaskModeWithOutlineMono)  // NOLINT
{
  // const auto log = std::make_shared<cras::MemoryLogHelper>();
  const auto log = std::make_shared<cras::NodeLogHelper>();

  urdf::Model model;
  model.initString("<robot name=\"test\"><link name=\"link1\"/></robot>");

  tf2::BufferCore tfBuffer;
  robot_model_renderer::TFLinkUpdater linkUpdater(log, &tfBuffer);
  linkUpdater.setFixedFrame("link1");

  robot_model_renderer::RobotModelRendererConfig config;
  config.pixelFormat = Ogre::PF_BYTE_L;
  config.renderingMode = robot_model_renderer::RenderingMode::MASK;
  config.drawOutline = true;
  config.outlineWidth = 20.0;
  config.outlineColor = Ogre::ColourValue(1.0f, 1.0f, 1.0f, 1.0f);

  robot_model_renderer::RobotErrors errors;
  robot_model_renderer::RobotModelRenderer renderer(log, model, &linkUpdater, errors, config);
  ASSERT_FALSE(errors.hasError());

  model.clear();
  model.initFile(std::string(TEST_DATA_DIR) + "/robot.urdf");
  const auto setModelResult = renderer.setModel(model);
  ASSERT_TRUE(setModelResult.has_value());

  geometry_msgs::TransformStamped tf;
  tf.header.frame_id = "link1";
  tf.header.stamp = {1732502880, 585000000};
  tf.child_frame_id = "link2";
  tf.transform.translation.x = 0.3;
  tf.transform.translation.z = 1.0;
  tf.transform.rotation.w = 1.0;

  tfBuffer.setTransform(tf, "test", true);

  sensor_msgs::CameraInfo camInfo;
  camInfo.header.frame_id = "link1";
  camInfo.header.stamp = {1732502880, 585000000};
  camInfo.height = 1616;
  camInfo.width = 1212;
  camInfo.distortion_model = "plumb_bob";
  camInfo.D = {1.0, 0, 0, 0, 0, 0.5, 0, 0};
  camInfo.K = {800.0, 0.0, 600.0, 0.0, 800.0, 800.0, 0.0, 0.0, 1.0};
  camInfo.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  camInfo.P = {800.0, 0.0, 600.0, 0.0, 0.0, 800.0, 800.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  ASSERT_TRUE(renderer.updateCameraInfo(robot_model_renderer::PinholeCameraModel(camInfo)));

  robot_model_renderer::RenderErrors renderErrors;
  const auto renderResult = renderer.render(camInfo.header.stamp, renderErrors);
  ASSERT_TRUE(renderResult.has_value());

  const auto& cvImage = renderResult.value();
  ASSERT_EQ(2, cvImage.dims);
  ASSERT_EQ(1616, cvImage.rows);
  ASSERT_EQ(1212, cvImage.cols);
  ASSERT_EQ(1, cvImage.channels());
  EXPECT_EQ(0, cvImage.at<uint8_t>(0, 0));
  EXPECT_EQ(0, cvImage.at<uint8_t>(1615, 0));
  EXPECT_EQ(0, cvImage.at<uint8_t>(1615, 1211));
  EXPECT_EQ(0, cvImage.at<uint8_t>(0, 1211));
  EXPECT_EQ(255, cvImage.at<uint8_t>(1616 / 2, 1212 / 2));
  EXPECT_EQ(255, cvImage.at<uint8_t>(1616 / 2, 1211));
  EXPECT_EQ(255, cvImage.at<uint8_t>(1616 / 2, 390));
}

TEST(RobotModelRendererNodelet, LowRes)  // NOLINT
{
  // const auto log = std::make_shared<cras::MemoryLogHelper>();
  const auto log = std::make_shared<cras::NodeLogHelper>();

  urdf::Model model;
  model.initString("<robot name=\"test\"><link name=\"link1\"/></robot>");

  tf2::BufferCore tfBuffer;
  robot_model_renderer::TFLinkUpdater linkUpdater(log, &tfBuffer);
  linkUpdater.setFixedFrame("link1");

  robot_model_renderer::RobotModelRendererConfig config;
  config.pixelFormat = Ogre::PF_BYTE_RGBA;
  config.maxRenderImageSize = 256;

  robot_model_renderer::RobotErrors errors;
  robot_model_renderer::RobotModelRenderer renderer(log, model, &linkUpdater, errors, config);
  ASSERT_FALSE(errors.hasError());

  model.clear();
  model.initFile(std::string(TEST_DATA_DIR) + "/robot.urdf");
  const auto setModelResult = renderer.setModel(model);
  ASSERT_TRUE(setModelResult.has_value());

  geometry_msgs::TransformStamped tf;
  tf.header.frame_id = "link1";
  tf.header.stamp = {1732502880, 585000000};
  tf.child_frame_id = "link2";
  tf.transform.translation.x = 0.3;
  tf.transform.translation.z = 1.0;
  tf.transform.rotation.w = 1.0;

  tfBuffer.setTransform(tf, "test", true);

  sensor_msgs::CameraInfo camInfo;
  camInfo.header.frame_id = "link1";
  camInfo.header.stamp = {1732502880, 585000000};
  camInfo.height = 1616;
  camInfo.width = 1212;
  camInfo.distortion_model = "plumb_bob";
  camInfo.D = {1.0, 0, 0, 0, 0, 0.5, 0, 0};
  camInfo.K = {800.0, 0.0, 600.0, 0.0, 800.0, 800.0, 0.0, 0.0, 1.0};
  camInfo.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  camInfo.P = {800.0, 0.0, 600.0, 0.0, 0.0, 800.0, 800.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  ASSERT_TRUE(renderer.updateCameraInfo(robot_model_renderer::PinholeCameraModel(camInfo)));

  robot_model_renderer::RenderErrors renderErrors;
  const auto renderResult = renderer.render(camInfo.header.stamp, renderErrors);
  ASSERT_TRUE(renderResult.has_value());

  const auto& cvImage = renderResult.value();
  ASSERT_EQ(2, cvImage.dims);
  ASSERT_EQ(1616, cvImage.rows);
  ASSERT_EQ(1212, cvImage.cols);
  ASSERT_EQ(4, cvImage.channels());
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(0, 0));
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(1615, 0));
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(1615, 1211));
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(0, 1211));
  EXPECT_EQ(cv::Vec4b(250, 0, 0, 255), cvImage.at<cv::Vec4b>(1616 / 2, 1212 / 2));
  EXPECT_EQ(cv::Vec4b(250, 0, 0, 255), cvImage.at<cv::Vec4b>(1616 / 2, 1211));
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(1616 / 2, 390));
}

TEST(RobotModelRendererNodelet, MaskBackground)  // NOLINT
{
  // const auto log = std::make_shared<cras::MemoryLogHelper>();
  const auto log = std::make_shared<cras::NodeLogHelper>();

  urdf::Model model;
  model.initString("<robot name=\"test\"><link name=\"link1\"/></robot>");

  tf2::BufferCore tfBuffer;
  robot_model_renderer::TFLinkUpdater linkUpdater(log, &tfBuffer);
  linkUpdater.setFixedFrame("link1");

  robot_model_renderer::RobotModelRendererConfig config;
  config.pixelFormat = Ogre::PF_BYTE_RGBA;
  config.staticMaskImage = cv::Mat(1616, 1212, CV_8UC4);
  config.staticMaskImage.at<cv::Vec4b>(0, 0) = cv::Vec4b(255, 128, 0, 255);
  config.staticMaskImage.at<cv::Vec4b>(1616 / 2, 1212 / 2) = cv::Vec4b(0, 128, 0, 255);
  config.staticMaskIsBackground = true;
  config.staticMaskImageEncoding = sensor_msgs::image_encodings::RGBA8;

  robot_model_renderer::RobotErrors errors;
  robot_model_renderer::RobotModelRenderer renderer(log, model, &linkUpdater, errors, config);
  ASSERT_FALSE(errors.hasError());

  model.clear();
  model.initFile(std::string(TEST_DATA_DIR) + "/robot.urdf");
  const auto setModelResult = renderer.setModel(model);
  ASSERT_TRUE(setModelResult.has_value());

  geometry_msgs::TransformStamped tf;
  tf.header.frame_id = "link1";
  tf.header.stamp = {1732502880, 585000000};
  tf.child_frame_id = "link2";
  tf.transform.translation.x = 0.3;
  tf.transform.translation.z = 1.0;
  tf.transform.rotation.w = 1.0;

  tfBuffer.setTransform(tf, "test", true);

  sensor_msgs::CameraInfo camInfo;
  camInfo.header.frame_id = "link1";
  camInfo.header.stamp = {1732502880, 585000000};
  camInfo.height = 1616;
  camInfo.width = 1212;
  camInfo.distortion_model = "plumb_bob";
  camInfo.D = {1.0, 0, 0, 0, 0, 0.5, 0, 0};
  camInfo.K = {800.0, 0.0, 600.0, 0.0, 800.0, 800.0, 0.0, 0.0, 1.0};
  camInfo.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  camInfo.P = {800.0, 0.0, 600.0, 0.0, 0.0, 800.0, 800.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  ASSERT_TRUE(renderer.updateCameraInfo(robot_model_renderer::PinholeCameraModel(camInfo)));

  robot_model_renderer::RenderErrors renderErrors;
  const auto renderResult = renderer.render(camInfo.header.stamp, renderErrors);
  ASSERT_TRUE(renderResult.has_value());

  const auto& cvImage = renderResult.value();
  ASSERT_EQ(2, cvImage.dims);
  ASSERT_EQ(1616, cvImage.rows);
  ASSERT_EQ(1212, cvImage.cols);
  ASSERT_EQ(4, cvImage.channels());
  EXPECT_EQ(cv::Vec4b(255, 128, 0, 255), cvImage.at<cv::Vec4b>(0, 0));
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(1615, 0));
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(1615, 1211));
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(0, 1211));
  EXPECT_EQ(cv::Vec4b(250, 0, 0, 255), cvImage.at<cv::Vec4b>(1616 / 2, 1212 / 2));
  EXPECT_EQ(cv::Vec4b(250, 0, 0, 255), cvImage.at<cv::Vec4b>(1616 / 2, 1211));
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(1616 / 2, 390));
}

TEST(RobotModelRendererNodelet, MaskForeground)  // NOLINT
{
  // const auto log = std::make_shared<cras::MemoryLogHelper>();
  const auto log = std::make_shared<cras::NodeLogHelper>();

  urdf::Model model;
  model.initString("<robot name=\"test\"><link name=\"link1\"/></robot>");

  tf2::BufferCore tfBuffer;
  robot_model_renderer::TFLinkUpdater linkUpdater(log, &tfBuffer);
  linkUpdater.setFixedFrame("link1");

  robot_model_renderer::RobotModelRendererConfig config;
  config.pixelFormat = Ogre::PF_BYTE_RGBA;
  config.staticMaskImage = cv::Mat(1616, 1212, CV_8UC4);
  config.staticMaskImage.at<cv::Vec4b>(0, 0) = cv::Vec4b(255, 128, 0, 255);
  config.staticMaskImage.at<cv::Vec4b>(1616 / 2, 1212 / 2) = cv::Vec4b(0, 128, 0, 255);
  config.staticMaskIsBackground = false;
  config.staticMaskImageEncoding = sensor_msgs::image_encodings::RGBA8;

  robot_model_renderer::RobotErrors errors;
  robot_model_renderer::RobotModelRenderer renderer(log, model, &linkUpdater, errors, config);
  ASSERT_FALSE(errors.hasError());

  model.clear();
  model.initFile(std::string(TEST_DATA_DIR) + "/robot.urdf");
  const auto setModelResult = renderer.setModel(model);
  ASSERT_TRUE(setModelResult.has_value());

  geometry_msgs::TransformStamped tf;
  tf.header.frame_id = "link1";
  tf.header.stamp = {1732502880, 585000000};
  tf.child_frame_id = "link2";
  tf.transform.translation.x = 0.3;
  tf.transform.translation.z = 1.0;
  tf.transform.rotation.w = 1.0;

  tfBuffer.setTransform(tf, "test", true);

  sensor_msgs::CameraInfo camInfo;
  camInfo.header.frame_id = "link1";
  camInfo.header.stamp = {1732502880, 585000000};
  camInfo.height = 1616;
  camInfo.width = 1212;
  camInfo.distortion_model = "plumb_bob";
  camInfo.D = {1.0, 0, 0, 0, 0, 0.5, 0, 0};
  camInfo.K = {800.0, 0.0, 600.0, 0.0, 800.0, 800.0, 0.0, 0.0, 1.0};
  camInfo.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  camInfo.P = {800.0, 0.0, 600.0, 0.0, 0.0, 800.0, 800.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  ASSERT_TRUE(renderer.updateCameraInfo(robot_model_renderer::PinholeCameraModel(camInfo)));

  robot_model_renderer::RenderErrors renderErrors;
  const auto renderResult = renderer.render(camInfo.header.stamp, renderErrors);
  ASSERT_TRUE(renderResult.has_value());

  const auto& cvImage = renderResult.value();
  ASSERT_EQ(2, cvImage.dims);
  ASSERT_EQ(1616, cvImage.rows);
  ASSERT_EQ(1212, cvImage.cols);
  ASSERT_EQ(4, cvImage.channels());
  EXPECT_EQ(cv::Vec4b(255, 128, 0, 255), cvImage.at<cv::Vec4b>(0, 0));
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(1615, 0));
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(1615, 1211));
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(0, 1211));
  EXPECT_EQ(cv::Vec4b(0, 128, 0, 255), cvImage.at<cv::Vec4b>(1616 / 2, 1212 / 2));
  EXPECT_EQ(cv::Vec4b(250, 0, 0, 255), cvImage.at<cv::Vec4b>(1616 / 2, 1211));
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(1616 / 2, 390));
}

TEST(RobotModelRendererNodelet, MaskForegroundColorMode)  // NOLINT
{
  // const auto log = std::make_shared<cras::MemoryLogHelper>();
  const auto log = std::make_shared<cras::NodeLogHelper>();

  urdf::Model model;
  model.initString("<robot name=\"test\"><link name=\"link1\"/></robot>");

  tf2::BufferCore tfBuffer;
  robot_model_renderer::TFLinkUpdater linkUpdater(log, &tfBuffer);
  linkUpdater.setFixedFrame("link1");

  robot_model_renderer::RobotModelRendererConfig config;
  config.pixelFormat = Ogre::PF_BYTE_RGBA;
  config.staticMaskImage = cv::Mat(1616, 1212, CV_8UC4);
  config.staticMaskImage.at<cv::Vec4b>(0, 0) = cv::Vec4b(255, 128, 0, 255);
  config.staticMaskImage.at<cv::Vec4b>(1616 / 2, 1212 / 2) = cv::Vec4b(0, 128, 0, 255);
  config.staticMaskIsBackground = false;
  config.staticMaskImageEncoding = sensor_msgs::image_encodings::RGBA8;
  config.renderingMode = robot_model_renderer::RenderingMode::COLOR;

  robot_model_renderer::RobotErrors errors;
  robot_model_renderer::RobotModelRenderer renderer(log, model, &linkUpdater, errors, config);
  ASSERT_FALSE(errors.hasError());

  model.clear();
  model.initFile(std::string(TEST_DATA_DIR) + "/robot.urdf");
  const auto setModelResult = renderer.setModel(model);
  ASSERT_TRUE(setModelResult.has_value());

  geometry_msgs::TransformStamped tf;
  tf.header.frame_id = "link1";
  tf.header.stamp = {1732502880, 585000000};
  tf.child_frame_id = "link2";
  tf.transform.translation.x = 0.3;
  tf.transform.translation.z = 1.0;
  tf.transform.rotation.w = 1.0;

  tfBuffer.setTransform(tf, "test", true);

  sensor_msgs::CameraInfo camInfo;
  camInfo.header.frame_id = "link1";
  camInfo.header.stamp = {1732502880, 585000000};
  camInfo.height = 1616;
  camInfo.width = 1212;
  camInfo.distortion_model = "plumb_bob";
  camInfo.D = {1.0, 0, 0, 0, 0, 0.5, 0, 0};
  camInfo.K = {800.0, 0.0, 600.0, 0.0, 800.0, 800.0, 0.0, 0.0, 1.0};
  camInfo.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  camInfo.P = {800.0, 0.0, 600.0, 0.0, 0.0, 800.0, 800.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  ASSERT_TRUE(renderer.updateCameraInfo(robot_model_renderer::PinholeCameraModel(camInfo)));

  robot_model_renderer::RenderErrors renderErrors;
  const auto renderResult = renderer.render(camInfo.header.stamp, renderErrors);
  ASSERT_TRUE(renderResult.has_value());

  const auto& cvImage = renderResult.value();
  ASSERT_EQ(2, cvImage.dims);
  ASSERT_EQ(1616, cvImage.rows);
  ASSERT_EQ(1212, cvImage.cols);
  ASSERT_EQ(4, cvImage.channels());
  EXPECT_EQ(cv::Vec4b(255, 0, 0, 255), cvImage.at<cv::Vec4b>(0, 0));
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(1615, 0));
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(1615, 1211));
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(0, 1211));
  EXPECT_EQ(cv::Vec4b(255, 0, 0, 255), cvImage.at<cv::Vec4b>(1616 / 2, 1212 / 2));
  EXPECT_EQ(cv::Vec4b(255, 0, 0, 255), cvImage.at<cv::Vec4b>(1616 / 2, 1211));
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(1616 / 2, 390));
}

TEST(RobotModelRendererNodelet, MaskForegroundMaskMode)  // NOLINT
{
  // const auto log = std::make_shared<cras::MemoryLogHelper>();
  const auto log = std::make_shared<cras::NodeLogHelper>();

  urdf::Model model;
  model.initString("<robot name=\"test\"><link name=\"link1\"/></robot>");

  tf2::BufferCore tfBuffer;
  robot_model_renderer::TFLinkUpdater linkUpdater(log, &tfBuffer);
  linkUpdater.setFixedFrame("link1");

  robot_model_renderer::RobotModelRendererConfig config;
  config.pixelFormat = Ogre::PF_BYTE_RGBA;
  config.staticMaskImage = cv::Mat(1616, 1212, CV_8UC4);
  config.staticMaskImage.at<cv::Vec4b>(0, 0) = cv::Vec4b(255, 128, 0, 255);
  config.staticMaskImage.at<cv::Vec4b>(1616 / 2, 1212 / 2) = cv::Vec4b(0, 128, 0, 255);
  config.staticMaskIsBackground = false;
  config.staticMaskImageEncoding = sensor_msgs::image_encodings::RGBA8;
  config.renderingMode = robot_model_renderer::RenderingMode::MASK;

  robot_model_renderer::RobotErrors errors;
  robot_model_renderer::RobotModelRenderer renderer(log, model, &linkUpdater, errors, config);
  ASSERT_FALSE(errors.hasError());

  model.clear();
  model.initFile(std::string(TEST_DATA_DIR) + "/robot.urdf");
  const auto setModelResult = renderer.setModel(model);
  ASSERT_TRUE(setModelResult.has_value());

  geometry_msgs::TransformStamped tf;
  tf.header.frame_id = "link1";
  tf.header.stamp = {1732502880, 585000000};
  tf.child_frame_id = "link2";
  tf.transform.translation.x = 0.3;
  tf.transform.translation.z = 1.0;
  tf.transform.rotation.w = 1.0;

  tfBuffer.setTransform(tf, "test", true);

  sensor_msgs::CameraInfo camInfo;
  camInfo.header.frame_id = "link1";
  camInfo.header.stamp = {1732502880, 585000000};
  camInfo.height = 1616;
  camInfo.width = 1212;
  camInfo.distortion_model = "plumb_bob";
  camInfo.D = {1.0, 0, 0, 0, 0, 0.5, 0, 0};
  camInfo.K = {800.0, 0.0, 600.0, 0.0, 800.0, 800.0, 0.0, 0.0, 1.0};
  camInfo.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  camInfo.P = {800.0, 0.0, 600.0, 0.0, 0.0, 800.0, 800.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  ASSERT_TRUE(renderer.updateCameraInfo(robot_model_renderer::PinholeCameraModel(camInfo)));

  robot_model_renderer::RenderErrors renderErrors;
  const auto renderResult = renderer.render(camInfo.header.stamp, renderErrors);
  ASSERT_TRUE(renderResult.has_value());

  const auto& cvImage = renderResult.value();
  ASSERT_EQ(2, cvImage.dims);
  ASSERT_EQ(1616, cvImage.rows);
  ASSERT_EQ(1212, cvImage.cols);
  ASSERT_EQ(4, cvImage.channels());
  EXPECT_EQ(cv::Vec4b(255, 255, 255, 255), cvImage.at<cv::Vec4b>(0, 0));
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(1615, 0));
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(1615, 1211));
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(0, 1211));
  EXPECT_EQ(cv::Vec4b(255, 255, 255, 255), cvImage.at<cv::Vec4b>(1616 / 2, 1212 / 2));
  EXPECT_EQ(cv::Vec4b(255, 255, 255, 255), cvImage.at<cv::Vec4b>(1616 / 2, 1211));
  EXPECT_EQ(cv::Vec4b(0, 0, 0, 0), cvImage.at<cv::Vec4b>(1616 / 2, 390));
}

int main(int argc, char **argv)
{
  setenv("LANG", "C", 1);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

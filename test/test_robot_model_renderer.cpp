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

#include <class_loader/class_loader_core.hpp>
#include <cras_cpp_common/log_utils/memory.h>
#include <cras_cpp_common/log_utils/node.h>
#include <cras_cpp_common/nodelet_utils.hpp>
#include <cras_cpp_common/param_utils/param_helper.hpp>
#include <cras_cpp_common/string_utils/ros.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <nodelet/nodelet.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

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

template<typename NodeletType = cras::Nodelet>
std::unique_ptr<NodeletType> createNodelet(const cras::LogHelperPtr& log,
  const ros::M_string& remaps = {},
  const std::shared_ptr<tf2_ros::Buffer>& tf = nullptr)
{
  // Declaration order of these variables is important to make sure they can be properly stopped and destroyed.
  auto nodelet = class_loader::impl::createInstance<nodelet::Nodelet>(
    "robot_model_renderer::RobotModelRendererNodelet", nullptr);
  if (nodelet == nullptr)
    return nullptr;

  {
    const auto paramHelper = dynamic_cast<cras::ParamHelper*>(nodelet);
    if (paramHelper != nullptr)
      paramHelper->setLogger(log);
  }

  const auto targetNodelet = dynamic_cast<NodeletType*>(nodelet);
  if (targetNodelet == nullptr)
  {
    delete nodelet;
    return nullptr;
  }

  if (tf != nullptr)
    targetNodelet->setBuffer(tf);

  nodelet->init(ros::this_node::getName(), remaps, my_argv, nullptr, nullptr);

  return std::unique_ptr<NodeletType>(targetNodelet);
}

TEST(RobotModelRendererNodelet, Default)  // NOLINT
{
  std::ifstream t(std::string(TEST_DATA_DIR) + "/robot.urdf");
  std::stringstream buffer;
  buffer << t.rdbuf();
  const auto robotDescription = buffer.str();

  ros::NodeHandle nh, pnh("~");

  pnh.deleteParam("");
  pnh.setParam("robot_description", robotDescription);

  std::vector<sensor_msgs::Image> lastImages;
  auto imageCb = [&lastImages](const sensor_msgs::Image::ConstPtr& msg)
  {
    lastImages.push_back(*msg);
  };

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // const auto log = std::make_shared<cras::MemoryLogHelper>();
  const auto log = std::make_shared<cras::NodeLogHelper>();

  ros::M_string remaps = {
    {"robot_description", ros::names::append(ros::this_node::getName(), "robot_description")},
  };
  auto nodelet = createNodelet(log, remaps);
  ASSERT_NE(nullptr, nodelet);

  auto imageSub = nh.subscribe<sensor_msgs::Image>("mask", 1, imageCb);
  for (size_t i = 0; i < 1000 && imageSub.getNumPublishers() == 0; ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for subscriber connection.");
  }
  ASSERT_GT(imageSub.getNumPublishers(), 0);

  auto tfPub = nh.advertise<tf2_msgs::TFMessage>("/tf_static", 1, true);
  for (size_t i = 0; i < 1000 && tfPub.getNumSubscribers() == 0; ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for TF publisher connection.");
  }
  ASSERT_GT(tfPub.getNumSubscribers(), 0);

  geometry_msgs::TransformStamped tf;
  tf.header.frame_id = "link1";
  tf.header.stamp = {1732502880, 585000000};
  tf.child_frame_id = "link2";
  tf.transform.translation.x = 0.3;
  tf.transform.translation.z = 1.0;
  tf.transform.rotation.w = 1.0;

  tf2_msgs::TFMessage tfMsg;
  tfMsg.transforms.push_back(tf);

  tfPub.publish(tfMsg);
  ros::WallDuration(0.3).sleep();

  auto camInfoPub = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1, false);
  for (size_t i = 0; i < 1000 && camInfoPub.getNumSubscribers() == 0; ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for camera info publisher connection.");
  }
  ASSERT_GT(camInfoPub.getNumSubscribers(), 0);

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

  camInfoPub.publish(camInfo);

  for (size_t i = 0; i < 50 && lastImages.empty() && ros::ok() && nodelet->ok(); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }

  ASSERT_FALSE(lastImages.empty());

  EXPECT_EQ("link1", lastImages[0].header.frame_id);
  EXPECT_EQ(ros::Time(1732502880, 585000000), lastImages[0].header.stamp);
  EXPECT_EQ(1212, lastImages[0].width);
  EXPECT_EQ(1616, lastImages[0].height);
  EXPECT_EQ(sensor_msgs::image_encodings::RGBA8, lastImages[0].encoding);
  EXPECT_THAT(lastImages[0].step, alignedStep(1212, 4));
  EXPECT_EQ(0, lastImages[0].is_bigendian);

  const auto cvImage = cv_bridge::toCvCopy(lastImages[0])->image;
  ASSERT_EQ(2, cvImage.size.dims());
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

  nodelet->requestStop();
}

TEST(RobotModelRendererNodelet, DefaultBGRA)  // NOLINT
{
  std::ifstream t(std::string(TEST_DATA_DIR) + "/robot.urdf");
  std::stringstream buffer;
  buffer << t.rdbuf();
  const auto robotDescription = buffer.str();

  ros::NodeHandle nh, pnh("~");

  pnh.deleteParam("");
  pnh.setParam("robot_description", robotDescription);
  pnh.setParam("image_encoding", "bgra8");

  std::vector<sensor_msgs::Image> lastImages;
  auto imageCb = [&lastImages](const sensor_msgs::Image::ConstPtr& msg)
  {
    lastImages.push_back(*msg);
  };

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // const auto log = std::make_shared<cras::MemoryLogHelper>();
  const auto log = std::make_shared<cras::NodeLogHelper>();

  ros::M_string remaps = {
    {"robot_description", ros::names::append(ros::this_node::getName(), "robot_description")},
  };
  auto nodelet = createNodelet(log, remaps);
  ASSERT_NE(nullptr, nodelet);

  auto imageSub = nh.subscribe<sensor_msgs::Image>("mask", 1, imageCb);
  for (size_t i = 0; i < 1000 && imageSub.getNumPublishers() == 0; ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for subscriber connection.");
  }
  ASSERT_GT(imageSub.getNumPublishers(), 0);

  auto tfPub = nh.advertise<tf2_msgs::TFMessage>("/tf_static", 1, true);
  for (size_t i = 0; i < 1000 && tfPub.getNumSubscribers() == 0; ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for TF publisher connection.");
  }
  ASSERT_GT(tfPub.getNumSubscribers(), 0);

  geometry_msgs::TransformStamped tf;
  tf.header.frame_id = "link1";
  tf.header.stamp = {1732502880, 585000000};
  tf.child_frame_id = "link2";
  tf.transform.translation.x = 0.3;
  tf.transform.translation.z = 1.0;
  tf.transform.rotation.w = 1.0;

  tf2_msgs::TFMessage tfMsg;
  tfMsg.transforms.push_back(tf);

  tfPub.publish(tfMsg);
  ros::WallDuration(0.3).sleep();

  auto camInfoPub = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1, false);
  for (size_t i = 0; i < 1000 && camInfoPub.getNumSubscribers() == 0; ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for camera info publisher connection.");
  }
  ASSERT_GT(camInfoPub.getNumSubscribers(), 0);

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

  camInfoPub.publish(camInfo);

  for (size_t i = 0; i < 50 && lastImages.empty() && ros::ok() && nodelet->ok(); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }

  ASSERT_FALSE(lastImages.empty());

  EXPECT_EQ("link1", lastImages[0].header.frame_id);
  EXPECT_EQ(ros::Time(1732502880, 585000000), lastImages[0].header.stamp);
  EXPECT_EQ(1212, lastImages[0].width);
  EXPECT_EQ(1616, lastImages[0].height);
  EXPECT_EQ(sensor_msgs::image_encodings::BGRA8, lastImages[0].encoding);
  EXPECT_THAT(lastImages[0].step, alignedStep(1212, 4));
  EXPECT_EQ(0, lastImages[0].is_bigendian);

  const auto cvImage = cv_bridge::toCvCopy(lastImages[0])->image;
  ASSERT_EQ(2, cvImage.size.dims());
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

  nodelet->requestStop();
}

TEST(RobotModelRendererNodelet, DefaultMono)  // NOLINT
{
  std::ifstream t(std::string(TEST_DATA_DIR) + "/robot.urdf");
  std::stringstream buffer;
  buffer << t.rdbuf();
  const auto robotDescription = buffer.str();

  ros::NodeHandle nh, pnh("~");

  pnh.deleteParam("");
  pnh.setParam("robot_description", robotDescription);
  pnh.setParam("image_encoding", "mono8");

  std::vector<sensor_msgs::Image> lastImages;
  auto imageCb = [&lastImages](const sensor_msgs::Image::ConstPtr& msg)
  {
    lastImages.push_back(*msg);
  };

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // const auto log = std::make_shared<cras::MemoryLogHelper>();
  const auto log = std::make_shared<cras::NodeLogHelper>();

  ros::M_string remaps = {
    {"robot_description", ros::names::append(ros::this_node::getName(), "robot_description")},
  };
  auto nodelet = createNodelet(log, remaps);
  ASSERT_NE(nullptr, nodelet);

  auto imageSub = nh.subscribe<sensor_msgs::Image>("mask", 1, imageCb);
  for (size_t i = 0; i < 1000 && imageSub.getNumPublishers() == 0; ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for subscriber connection.");
  }
  ASSERT_GT(imageSub.getNumPublishers(), 0);

  auto tfPub = nh.advertise<tf2_msgs::TFMessage>("/tf_static", 1, true);
  for (size_t i = 0; i < 1000 && tfPub.getNumSubscribers() == 0; ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for TF publisher connection.");
  }
  ASSERT_GT(tfPub.getNumSubscribers(), 0);

  geometry_msgs::TransformStamped tf;
  tf.header.frame_id = "link1";
  tf.header.stamp = {1732502880, 585000000};
  tf.child_frame_id = "link2";
  tf.transform.translation.x = 0.3;
  tf.transform.translation.z = 1.0;
  tf.transform.rotation.w = 1.0;

  tf2_msgs::TFMessage tfMsg;
  tfMsg.transforms.push_back(tf);

  tfPub.publish(tfMsg);
  ros::WallDuration(0.3).sleep();

  auto camInfoPub = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1, false);
  for (size_t i = 0; i < 1000 && camInfoPub.getNumSubscribers() == 0; ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for camera info publisher connection.");
  }
  ASSERT_GT(camInfoPub.getNumSubscribers(), 0);

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

  camInfoPub.publish(camInfo);

  for (size_t i = 0; i < 50 && lastImages.empty() && ros::ok() && nodelet->ok(); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }

  ASSERT_FALSE(lastImages.empty());

  EXPECT_EQ("link1", lastImages[0].header.frame_id);
  EXPECT_EQ(ros::Time(1732502880, 585000000), lastImages[0].header.stamp);
  EXPECT_EQ(1212, lastImages[0].width);
  EXPECT_EQ(1616, lastImages[0].height);
  EXPECT_EQ(sensor_msgs::image_encodings::MONO8, lastImages[0].encoding);
  EXPECT_THAT(lastImages[0].step, alignedStep(1212, 1));
  EXPECT_EQ(0, lastImages[0].is_bigendian);

  const auto cvImage = cv_bridge::toCvCopy(lastImages[0])->image;
  ASSERT_EQ(2, cvImage.size.dims());
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

  nodelet->requestStop();
}

TEST(RobotModelRendererNodelet, ColorModeWithOutline)  // NOLINT
{
  std::ifstream t(std::string(TEST_DATA_DIR) + "/robot.urdf");
  std::stringstream buffer;
  buffer << t.rdbuf();
  const auto robotDescription = buffer.str();

  ros::NodeHandle nh, pnh("~");

  pnh.deleteParam("");
  pnh.setParam("robot_description", robotDescription);
  pnh.setParam("rendering_mode", "color");
  pnh.setParam("color_mode_color", std::vector<float>{0.0, 1.0, 0.0, 1.0});
  pnh.setParam("draw_outline", true);
  pnh.setParam("outline_width", 20.0);
  pnh.setParam("outline_color", std::vector<float>{0.0, 0.0, 1.0, 1.0});

  std::vector<sensor_msgs::Image> lastImages;
  auto imageCb = [&lastImages](const sensor_msgs::Image::ConstPtr& msg)
  {
    lastImages.push_back(*msg);
  };

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // const auto log = std::make_shared<cras::MemoryLogHelper>();
  const auto log = std::make_shared<cras::NodeLogHelper>();

  ros::M_string remaps = {
    {"robot_description", ros::names::append(ros::this_node::getName(), "robot_description")},
  };
  auto nodelet = createNodelet(log, remaps);
  ASSERT_NE(nullptr, nodelet);

  auto imageSub = nh.subscribe<sensor_msgs::Image>("mask", 1, imageCb);
  for (size_t i = 0; i < 1000 && imageSub.getNumPublishers() == 0; ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for subscriber connection.");
  }
  ASSERT_GT(imageSub.getNumPublishers(), 0);

  auto tfPub = nh.advertise<tf2_msgs::TFMessage>("/tf_static", 1, true);
  for (size_t i = 0; i < 1000 && tfPub.getNumSubscribers() == 0; ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for TF publisher connection.");
  }
  ASSERT_GT(tfPub.getNumSubscribers(), 0);

  geometry_msgs::TransformStamped tf;
  tf.header.frame_id = "link1";
  tf.header.stamp = {1732502880, 585000000};
  tf.child_frame_id = "link2";
  tf.transform.translation.x = 0.3;
  tf.transform.translation.z = 1.0;
  tf.transform.rotation.w = 1.0;

  tf2_msgs::TFMessage tfMsg;
  tfMsg.transforms.push_back(tf);

  tfPub.publish(tfMsg);
  ros::WallDuration(0.3).sleep();

  auto camInfoPub = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1, false);
  for (size_t i = 0; i < 1000 && camInfoPub.getNumSubscribers() == 0; ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for camera info publisher connection.");
  }
  ASSERT_GT(camInfoPub.getNumSubscribers(), 0);

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

  camInfoPub.publish(camInfo);

  for (size_t i = 0; i < 50 && lastImages.empty() && ros::ok() && nodelet->ok(); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }

  ASSERT_FALSE(lastImages.empty());

  EXPECT_EQ("link1", lastImages[0].header.frame_id);
  EXPECT_EQ(ros::Time(1732502880, 585000000), lastImages[0].header.stamp);
  EXPECT_EQ(1212, lastImages[0].width);
  EXPECT_EQ(1616, lastImages[0].height);
  EXPECT_EQ(sensor_msgs::image_encodings::RGBA8, lastImages[0].encoding);
  EXPECT_THAT(lastImages[0].step, alignedStep(1212, 4));
  EXPECT_EQ(0, lastImages[0].is_bigendian);

  const auto cvImage = cv_bridge::toCvCopy(lastImages[0])->image;
  ASSERT_EQ(2, cvImage.size.dims());
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

  nodelet->requestStop();
}

TEST(RobotModelRendererNodelet, MaskModeWithOutlineMono)  // NOLINT
{
  std::ifstream t(std::string(TEST_DATA_DIR) + "/robot.urdf");
  std::stringstream buffer;
  buffer << t.rdbuf();
  const auto robotDescription = buffer.str();

  ros::NodeHandle nh, pnh("~");

  pnh.deleteParam("");
  pnh.setParam("robot_description", robotDescription);
  pnh.setParam("image_encoding", "mono8");
  pnh.setParam("rendering_mode", "mask");
  pnh.setParam("draw_outline", true);
  pnh.setParam("outline_width", 20.0);
  pnh.setParam("outline_color", std::vector<float>{1.0, 1.0, 1.0, 1.0});

  std::vector<sensor_msgs::Image> lastImages;
  auto imageCb = [&lastImages](const sensor_msgs::Image::ConstPtr& msg)
  {
    lastImages.push_back(*msg);
  };

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // const auto log = std::make_shared<cras::MemoryLogHelper>();
  const auto log = std::make_shared<cras::NodeLogHelper>();

  ros::M_string remaps = {
    {"robot_description", ros::names::append(ros::this_node::getName(), "robot_description")},
  };
  auto nodelet = createNodelet(log, remaps);
  ASSERT_NE(nullptr, nodelet);

  auto imageSub = nh.subscribe<sensor_msgs::Image>("mask", 1, imageCb);
  for (size_t i = 0; i < 1000 && imageSub.getNumPublishers() == 0; ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for subscriber connection.");
  }
  ASSERT_GT(imageSub.getNumPublishers(), 0);

  auto tfPub = nh.advertise<tf2_msgs::TFMessage>("/tf_static", 1, true);
  for (size_t i = 0; i < 1000 && tfPub.getNumSubscribers() == 0; ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for TF publisher connection.");
  }
  ASSERT_GT(tfPub.getNumSubscribers(), 0);

  geometry_msgs::TransformStamped tf;
  tf.header.frame_id = "link1";
  tf.header.stamp = {1732502880, 585000000};
  tf.child_frame_id = "link2";
  tf.transform.translation.x = 0.3;
  tf.transform.translation.z = 1.0;
  tf.transform.rotation.w = 1.0;

  tf2_msgs::TFMessage tfMsg;
  tfMsg.transforms.push_back(tf);

  tfPub.publish(tfMsg);
  ros::WallDuration(0.3).sleep();

  auto camInfoPub = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1, false);
  for (size_t i = 0; i < 1000 && camInfoPub.getNumSubscribers() == 0; ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for camera info publisher connection.");
  }
  ASSERT_GT(camInfoPub.getNumSubscribers(), 0);

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

  camInfoPub.publish(camInfo);

  for (size_t i = 0; i < 50 && lastImages.empty() && ros::ok() && nodelet->ok(); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }

  ASSERT_FALSE(lastImages.empty());

  EXPECT_EQ("link1", lastImages[0].header.frame_id);
  EXPECT_EQ(ros::Time(1732502880, 585000000), lastImages[0].header.stamp);
  EXPECT_EQ(1212, lastImages[0].width);
  EXPECT_EQ(1616, lastImages[0].height);
  EXPECT_EQ(sensor_msgs::image_encodings::MONO8, lastImages[0].encoding);
  EXPECT_THAT(lastImages[0].step, alignedStep(1212, 1));
  EXPECT_EQ(0, lastImages[0].is_bigendian);

  const auto cvImage = cv_bridge::toCvCopy(lastImages[0])->image;
  ASSERT_EQ(2, cvImage.size.dims());
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

  nodelet->requestStop();
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  // Remove the program name from argv because the nodelet handling code does not expect it
  argc -= 1;
  argv += 1;
  ros::removeROSArgs(argc, argv, my_argv);
  ros::init(argc, argv, "test_robot_model_renderer");

  ros::NodeHandle nh;  // Just prevent ROS being uninited when the test-private nodehandles go out of scope

  return RUN_ALL_TESTS();
}

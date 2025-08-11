// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: 2009 Willow Garage, Inc.

#pragma once

// This file is taken from rviz and slightly edited to be usable in this package.

#include <string>

#include <OgrePrerequisites.h>

#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/tf2_utils/interruptible_buffer.h>
#include <robot_model_renderer/robot/link_updater.h>
#include <ros/duration.h>
#include <ros/time.h>
#include <tf2_ros/buffer.h>

namespace robot_model_renderer
{

class TFLinkUpdater : public LinkUpdater, public cras::HasLogger
{
public:
  explicit TFLinkUpdater(const cras::LogHelperPtr& log, const std::shared_ptr<cras::InterruptibleTFBuffer>& tf,
    const std::string& fixed_frame = {}, const std::string& tf_prefix = {},
    const ros::Duration& timeout = ros::Duration(0.01));

  void setFixedFrame(const std::string& fixedFrame);

  cras::expected<void, LinkUpdateError> getLinkTransforms(const ros::Time& time, const std::string& link_name,
    Ogre::Vector3& visual_position, Ogre::Quaternion& visual_orientation,
    Ogre::Vector3& collision_position, Ogre::Quaternion& collision_orientation) const override;

private:
  std::shared_ptr<cras::InterruptibleTFBuffer> tf_;
  std::string fixed_frame_;
  std::string tf_prefix_;
  ros::Duration timeout_;
};

}

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: 2009 Willow Garage, Inc.

#pragma once

// This file is taken from rviz and slightly edited to be usable in this package.

#include <string>

#include <OgrePrerequisites.h>

#include <cras_cpp_common/expected.hpp>
#include <ros/time.h>

namespace robot_model_renderer
{

struct LinkUpdateError
{
  std::string name;
  std::string error;
  bool maySucceedLater {false};

  bool hasError() const
  {
    return !error.empty();
  };
};

class LinkUpdater
{
public:
  virtual ~LinkUpdater() = default;

  virtual cras::expected<void, LinkUpdateError> getLinkTransforms(
    const ros::Time& time, const std::string& link_name,
    Ogre::Vector3& visual_position, Ogre::Quaternion& visual_orientation,
    Ogre::Vector3& collision_position, Ogre::Quaternion& collision_orientation) const = 0;
};

}

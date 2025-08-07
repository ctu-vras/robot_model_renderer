// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: 2009 Willow Garage, Inc.

#pragma once

// This file is taken from rviz and slightly edited to be usable in this package.

#include <string>

#include <OgrePrerequisites.h>

#include <ros/time.h>

namespace robot_model_renderer
{

class LinkUpdater
{
public:
  virtual ~LinkUpdater() = default;

  virtual bool getLinkTransforms(
    const ros::Time& time, const std::string& link_name,
    Ogre::Vector3& visual_position, Ogre::Quaternion& visual_orientation,
    Ogre::Vector3& collision_position, Ogre::Quaternion& collision_orientation) const = 0;
};

}

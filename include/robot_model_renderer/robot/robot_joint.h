// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: 2013 Willow Garage, Inc.

#pragma once

// This file is taken from rviz and slightly edited to be usable in this package.

#include <string>

#include <OgrePrerequisites.h>

#include <urdf/model.h>

#include <robot_model_renderer/ogre_helpers/ogre_vector.h>

namespace robot_model_renderer
{

class Shape;
class Robot;
class RobotJoint;

/**
 * \brief Contains any data we need from a joint in the robot.
 */
class RobotJoint
{
public:
  RobotJoint(Robot* robot, const urdf::JointConstSharedPtr& joint);

  virtual ~RobotJoint();

  void setTransforms(const Ogre::Vector3& parent_link_position, const Ogre::Quaternion& parent_link_orientation);

  const std::string& getName() const
  {
    return name_;
  }

  const std::string& getParentLinkName() const
  {
    return parent_link_name_;
  }

  const std::string& getChildLinkName() const
  {
    return child_link_name_;
  }

  RobotJoint* getParentJoint() const;

  Ogre::Vector3 getPosition();

  Ogre::Quaternion getOrientation();

protected:
  bool getEnabled() const;

  Robot* robot_;
  std::string name_;  //!< Name of this joint
  std::string parent_link_name_;
  std::string child_link_name_;

  Ogre::Vector3 joint_origin_pos_;
  Ogre::Quaternion joint_origin_rot_;

  Ogre::Vector3 position_;
  Ogre::Quaternion orientation_;

  bool enabled_;
};

}

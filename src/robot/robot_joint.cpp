/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// This file is compiled from rviz and gazebo and slightly edited to be usable in this package.

#include <robot_model_renderer/robot/robot_joint.h>

#include <OgreSceneNode.h>

#include <robot_model_renderer/robot/robot.h>
#include <robot_model_renderer/robot/robot_link.h>

namespace robot_model_renderer
{

RobotJoint::RobotJoint(Robot* robot, const urdf::JointConstSharedPtr& joint)
  : robot_(robot), name_(joint->name), parent_link_name_(joint->parent_link_name),
    child_link_name_(joint->child_link_name), has_descendent_links_with_geometry_(true)
{
  std::string type = "";
  if (joint->type == urdf::Joint::UNKNOWN)
    type = "unknown";
  else if (joint->type == urdf::Joint::REVOLUTE)
    type = "revolute";
  else if (joint->type == urdf::Joint::CONTINUOUS)
    type = "continuous";
  else if (joint->type == urdf::Joint::PRISMATIC)
    type = "prismatic";
  else if (joint->type == urdf::Joint::FLOATING)
    type = "floating";
  else if (joint->type == urdf::Joint::PLANAR)
    type = "planar";
  else if (joint->type == urdf::Joint::FIXED)
    type = "fixed";

  const urdf::Vector3& pos = joint->parent_to_joint_origin_transform.position;
  const urdf::Rotation& rot = joint->parent_to_joint_origin_transform.rotation;
  joint_origin_pos_ = Ogre::Vector3(pos.x, pos.y, pos.z);
  joint_origin_rot_ = Ogre::Quaternion(rot.w, rot.x, rot.y, rot.z);
}

RobotJoint::~RobotJoint()
{
}

RobotJoint* RobotJoint::getParentJoint()
{
  RobotLink* parent_link = robot_->getLink(parent_link_name_);
  if (!parent_link)
    return nullptr;

  const std::string& parent_joint_name = parent_link->getParentJointName();
  if (parent_joint_name.empty())
    return nullptr;

  return robot_->getJoint(parent_joint_name);
}

void RobotJoint::getChildLinkState(int& links_with_geom, int& links_with_geom_checked, int& links_with_geom_unchecked,
  const bool recursive) const
{
  links_with_geom_checked = 0;
  links_with_geom_unchecked = 0;

  RobotLink* link = robot_->getLink(child_link_name_);
  assert(link);
  if (link->hasGeometry())
  {
    bool checked = true;
    links_with_geom_checked += checked ? 1 : 0;
    links_with_geom_unchecked += checked ? 0 : 1;
  }

  if (recursive)
  {
    auto child_joint_it = link->getChildJointNames().begin();
    const auto child_joint_end = link->getChildJointNames().end();
    for (; child_joint_it != child_joint_end; ++child_joint_it)
    {
      const RobotJoint* child_joint = robot_->getJoint(*child_joint_it);
      if (child_joint)
      {
        int child_links_with_geom;
        int child_links_with_geom_checked;
        int child_links_with_geom_unchecked;
        child_joint->getChildLinkState(child_links_with_geom, child_links_with_geom_checked,
                                       child_links_with_geom_unchecked, recursive);
        links_with_geom_checked += child_links_with_geom_checked;
        links_with_geom_unchecked += child_links_with_geom_unchecked;
      }
    }
  }

  links_with_geom = links_with_geom_checked + links_with_geom_unchecked;
}

bool RobotJoint::getEnabled() const
{
  if (!hasDescendentLinksWithGeometry())
    return true;
  return true;
}

void RobotJoint::updateChildVisibility()
{
  if (!hasDescendentLinksWithGeometry())
    return;

  bool visible = getEnabled();

  RobotLink* link = robot_->getLink(child_link_name_);
  if (link)
  {
    if (link->hasGeometry())
    {
      // TODO
    }
  }
}

void RobotJoint::setTransforms(
  const Ogre::Vector3& parent_link_position, const Ogre::Quaternion& parent_link_orientation)
{
  position_ = parent_link_position + parent_link_orientation * joint_origin_pos_;
  orientation_ = parent_link_orientation * joint_origin_rot_;
}

Ogre::Vector3 RobotJoint::getPosition()
{
  return position_;
}

Ogre::Quaternion RobotJoint::getOrientation()
{
  return orientation_;
}

}

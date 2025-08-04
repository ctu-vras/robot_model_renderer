/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include <robot_model_renderer/robot/robot.h>

#include <string>

#include <urdf_model/model.h>

#include <OgreEntity.h>
#include <OgreMaterial.h>
#include <OgreMaterialManager.h>
#include <OgreResourceGroupManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <robot_model_renderer/robot/robot_joint.h>
#include <robot_model_renderer/robot/robot_link.h>
#include <ros/console.h>

namespace robot_model_renderer
{
Robot::Robot(Ogre::SceneNode* root_node, Ogre::SceneManager* scene_manager, const std::string& name)
  : scene_manager_(scene_manager), root_link_(nullptr),
    visible_(true), visual_visible_(true), collision_visible_(false),
    robot_loaded_(false), name_(name), alpha_(1.0f)
{
  root_visual_node_ = root_node->createChildSceneNode();
  root_collision_node_ = root_node->createChildSceneNode();

  setVisualVisible(visual_visible_);
  setCollisionVisible(collision_visible_);
  setAlpha(1.0f);
}

Robot::~Robot()
{
  Robot::clear();

  scene_manager_->destroySceneNode(root_visual_node_);
  scene_manager_->destroySceneNode(root_collision_node_);
}

void Robot::setVisible(const bool visible)
{
  visible_ = visible;
  if (visible)
  {
    root_visual_node_->setVisible(visual_visible_);
    root_collision_node_->setVisible(collision_visible_);
    updateLinkVisibilities();
  }
  else
  {
    root_visual_node_->setVisible(false);
    root_collision_node_->setVisible(false);
    updateLinkVisibilities();
  }
}

void Robot::setVisualVisible(const bool visible)
{
  visual_visible_ = visible;
  updateLinkVisibilities();
}

void Robot::setCollisionVisible(const bool visible)
{
  collision_visible_ = visible;
  updateLinkVisibilities();
}

void Robot::updateLinkVisibilities()
{
  for (const auto& [name, link] : links_)
  {
    link->updateVisibility();
  }
}

bool Robot::isVisible() const
{
  return visible_;
}

bool Robot::isVisualVisible() const
{
  return visual_visible_;
}

bool Robot::isCollisionVisible() const
{
  return collision_visible_;
}

void Robot::setAlpha(const float a)
{
  alpha_ = a;

  for (const auto& [name, link] : links_)
  {
    link->setRobotAlpha(alpha_);
  }
}

void Robot::clear()
{
  for (const auto& [name, link] : links_)
  {
    delete link;
  }

  for (const auto& [name, joint] : joints_)
  {
    delete joint;
  }

  links_.clear();
  joints_.clear();
  root_visual_node_->removeAndDestroyAllChildren();
  root_collision_node_->removeAndDestroyAllChildren();
}

RobotLink* Robot::createLink(Robot* robot, const urdf::LinkConstSharedPtr& link, const std::string& parent_joint_name,
  const std::shared_ptr<ShapeFilter>& shape_filter)
{
  return new RobotLink(robot, link, parent_joint_name, shape_filter);
}

RobotJoint* Robot::createJoint(Robot* robot, const urdf::JointConstSharedPtr& joint)
{
  return new RobotJoint(robot, joint);
}

void Robot::load(const urdf::ModelInterface& urdf, const std::shared_ptr<ShapeFilter>& shape_filter)
{
  robot_loaded_ = false;

  // clear out any data (properties, shapes, etc) from a previously loaded robot.
  clear();

  // the root link is discovered below.  Set to NULL until found.
  root_link_ = nullptr;

  // Create properties for each link.
  // Properties are not added to display until changedLinkTreeStyle() is called (below).
  {
    for (const auto& [link_name, urdf_link] : urdf.links_)
    {
      if (!shape_filter->considerLink(link_name))
        continue;

      std::string parent_joint_name;
      if (urdf_link != urdf.getRoot() && urdf_link->parent_joint)
      {
        parent_joint_name = urdf_link->parent_joint->name;
      }

      RobotLink* link = this->createLink(this, urdf_link, parent_joint_name, shape_filter);

      if (urdf_link == urdf.getRoot())
      {
        root_link_ = link;
      }

      links_[urdf_link->name] = link;

      link->setRobotAlpha(alpha_);
    }
  }

  // Create properties for each joint.
  {
    for (const auto& [joint_name, urdf_joint] : urdf.joints_)
    {
      RobotJoint* joint = this->createJoint(this, urdf_joint);

      joints_[urdf_joint->name] = joint;
    }
  }

  // robot is now loaded
  robot_loaded_ = true;

  setVisualVisible(isVisualVisible());
  setCollisionVisible(isCollisionVisible());
}

RobotLink* Robot::getLink(const std::string& name) const
{
  const auto it = links_.find(name);
  if (it == links_.end())
  {
    ROS_WARN("Link [%s] does not exist", name.c_str());
    return nullptr;
  }

  return it->second;
}

RobotJoint* Robot::getJoint(const std::string& name) const
{
  const auto it = joints_.find(name);
  if (it == joints_.end())
  {
    ROS_WARN("Joint [%s] does not exist", name.c_str());
    return nullptr;
  }

  return it->second;
}

void Robot::update(const LinkUpdater& updater, const ros::Time& time)
{
  for (const auto& [link_name, link] : links_)
  {
    Ogre::Vector3 visual_position, collision_position;
    Ogre::Quaternion visual_orientation, collision_orientation;
    if (updater.getLinkTransforms(time, link->getName(), visual_position, visual_orientation,
                                  collision_position, collision_orientation))
    {
      link->setToNormalMaterial();

      // Check if visual_orientation, visual_position, collision_orientation, and collision_position are
      // NaN.
      if (visual_orientation.isNaN())
      {
        ROS_ERROR_THROTTLE(1.0,
          "visual orientation of %s contains NaNs. Skipping render as long as the orientation is invalid.",
          link->getName().c_str());
        continue;
      }
      if (visual_position.isNaN())
      {
        ROS_ERROR_THROTTLE(1.0,
          "visual position of %s contains NaNs. Skipping render as long as the position is invalid.",
          link->getName().c_str());
        continue;
      }
      if (collision_orientation.isNaN())
      {
        ROS_ERROR_THROTTLE(1.0,
          "collision orientation of %s contains NaNs. Skipping render as long as the orientation is invalid.",
          link->getName().c_str());
        continue;
      }
      if (collision_position.isNaN())
      {
        ROS_ERROR_THROTTLE(1.0,
          "collision position of %s contains NaNs. Skipping render as long as the position is invalid.",
          link->getName().c_str());
        continue;
      }
      link->setTransforms(visual_position, visual_orientation, collision_position, collision_orientation);

      for (const auto& jointName : link->getChildJointNames())
      {
        const auto joint = getJoint(jointName);
        if (joint)
        {
          joint->setTransforms(visual_position, visual_orientation);
        }
      }
    }
    else
    {
      link->setToErrorMaterial();
    }
  }
}

void Robot::setPosition(const Ogre::Vector3& position)
{
  root_visual_node_->setPosition(position);
  root_collision_node_->setPosition(position);
}

void Robot::setOrientation(const Ogre::Quaternion& orientation)
{
  root_visual_node_->setOrientation(orientation);
  root_collision_node_->setOrientation(orientation);
}

void Robot::setScale(const Ogre::Vector3& scale)
{
  root_visual_node_->setScale(scale);
  root_collision_node_->setScale(scale);
}

void Robot::setMaskMode()
{
  for (const auto& [link_name, link] : links_)
    link->setMaskMode();
}

void Robot::unsetMaskMode()
{
  for (const auto& [link_name, link] : links_)
    link->unsetMaskMode();
}

void Robot::setColorMode(const float red, const float green, const float blue)
{
  for (const auto& [link_name, link] : links_)
    link->setColorMode(red, green, blue);
}

void Robot::unsetColorMode()
{
  for (const auto& [link_name, link] : links_)
    link->unsetColorMode();
}

const Ogre::Vector3& Robot::getPosition()
{
  return root_visual_node_->getPosition();
}

const Ogre::Quaternion& Robot::getOrientation()
{
  return root_visual_node_->getOrientation();
}

}

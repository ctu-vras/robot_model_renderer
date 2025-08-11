// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: 2008 Willow Garage, Inc.

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
#include <robot_model_renderer/robot/shape_filter.h>
#include <robot_model_renderer/robot/shape_inflation_registry.h>
#include <ros/console.h>

namespace robot_model_renderer
{

bool RobotErrors::hasError(bool includeMaterialErrors) const
{
  const auto linkHasError = [includeMaterialErrors](const LinkError& e)
  {
    return e.hasError(includeMaterialErrors);
  };
  if (std::any_of(linkErrors.begin(), linkErrors.end(), linkHasError))
    return true;

  return std::any_of(jointErrors.begin(), jointErrors.end(), [](const JointError& e) { return e.hasError(); });
}

bool UpdateErrors::hasError() const
{
  if (!this->error.empty())
    return true;

  return std::any_of(linkErrors.begin(), linkErrors.end(), [](const auto& e) { return e.second.hasError(); });
}

std::string UpdateErrors::toString() const
{
  if (!this->error.empty())
    return this->error;

  if (linkErrors.empty())
    return "";

  std::vector<std::string> errors;
  for (const auto& [linkName, linkError] : linkErrors)
    if (linkError.hasError())
      errors.push_back(linkName);
  return cras::join(errors, ",");
}

Robot::Robot(
  const cras::LogHelperPtr& log, Ogre::SceneNode* root_node, Ogre::SceneManager* scene_manager, const std::string& name)
  : cras::HasLogger(log), scene_manager_(scene_manager), root_link_(nullptr),
    visible_(true), visual_visible_(true), collision_visible_(false), robot_loaded_(false), name_(name), alpha_(1.0f)
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

const RobotErrors& Robot::getErrors() const
{
  return this->errors_;
}

const UpdateErrors& Robot::getUpdateErrors() const
{
  return this->update_errors_;
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

cras::expected<RobotLink*, LinkError> Robot::createLink(Robot* robot, const urdf::LinkConstSharedPtr& link,
  const std::string& parent_joint_name, const std::shared_ptr<ShapeFilter>& shape_filter,
  const std::shared_ptr<ShapeInflationRegistry>& shape_inflation_registry)
{
  const auto result = new RobotLink(this->log, robot, link, parent_joint_name, shape_filter, shape_inflation_registry);
  if (result->getErrors().hasError(false))
    return cras::make_unexpected(result->getErrors());
  return result;
}

cras::expected<RobotJoint*, JointError> Robot::createJoint(Robot* robot, const urdf::JointConstSharedPtr& joint)
{
  const auto result = new RobotJoint(this->log, robot, joint);
  if (result->getError().hasError())
    return cras::make_unexpected(result->getError());
  return result;
}

cras::expected<void, RobotErrors> Robot::load(
  const urdf::ModelInterface& urdf, const std::shared_ptr<ShapeFilter>& shape_filter,
  const std::shared_ptr<ShapeInflationRegistry>& shape_inflation_registry)
{
  this->robot_loaded_ = false;

  // clear out any data (properties, shapes, etc) from a previously loaded robot.
  clear();

  // the root link is discovered below.  Set to NULL until found.
  this->root_link_ = nullptr;

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

      const auto maybeLink = this->createLink(
        this, urdf_link, parent_joint_name, shape_filter, shape_inflation_registry);

      if (!maybeLink.has_value())
      {
        this->errors_.linkErrors.push_back(maybeLink.error());
      }
      else
      {
        const auto link = maybeLink.value();

        if (urdf_link == urdf.getRoot())
        {
          this->root_link_ = link;
        }

        this->links_[urdf_link->name] = link;

        link->setRobotAlpha(this->alpha_);
      }
    }
  }

  // Create properties for each joint.
  {
    for (const auto& [joint_name, urdf_joint] : urdf.joints_)
    {
      auto maybeJoint = this->createJoint(this, urdf_joint);

      if (!maybeJoint.has_value())
      {
        this->errors_.jointErrors.push_back(maybeJoint.error());
      }
      else
      {
        this->joints_[urdf_joint->name] = maybeJoint.value();
      }
    }
  }

  // robot is now loaded
  this->robot_loaded_ = true;

  setVisualVisible(isVisualVisible());
  setCollisionVisible(isCollisionVisible());

  if (this->errors_.hasError(false))
    return cras::make_unexpected(this->errors_);
  return {};
}

RobotLink* Robot::getLink(const std::string& name) const
{
  const auto it = links_.find(name);
  if (it == links_.end())
  {
    CRAS_WARN_NAMED("robot_model", "Link [%s] does not exist", name.c_str());
    return nullptr;
  }

  return it->second;
}

RobotJoint* Robot::getJoint(const std::string& name) const
{
  const auto it = joints_.find(name);
  if (it == joints_.end())
  {
    CRAS_WARN_NAMED("robot_model", "Joint [%s] does not exist", name.c_str());
    return nullptr;
  }

  return it->second;
}

cras::expected<void, UpdateErrors> Robot::update(LinkUpdater& updater, const ros::Time& time)
{
  UpdateErrors errors;

  updater.prepareForTime(time);

  for (const auto& [link_name, link] : links_)
  {
    Ogre::Vector3 visual_position, collision_position;
    Ogre::Quaternion visual_orientation, collision_orientation;
    const auto transformResult = updater.getLinkTransforms(time, link->getName(), visual_position, visual_orientation,
      collision_position, collision_orientation);
    if (transformResult.has_value())
    {
      link->setToNormalMaterial();

      // Check if visual_orientation, visual_position, collision_orientation, and collision_position are
      // NaN.
      std::string error;
      if (visual_orientation.isNaN())
      {
        error = cras::format(
          "visual orientation of %s contains NaNs. Skipping render as long as the orientation is invalid.",
          link->getName().c_str());
        CRAS_ERROR_THROTTLE_NAMED(1.0, "link_updater", "%s", error.c_str());
      }
      else if (visual_position.isNaN())
      {
        error = cras::format(
          "visual position of %s contains NaNs. Skipping render as long as the position is invalid.",
          link->getName().c_str());
        CRAS_ERROR_THROTTLE_NAMED(1.0, "link_updater", "%s", error.c_str());
      }
      else if (collision_orientation.isNaN())
      {
        error = cras::format(
          "collision orientation of %s contains NaNs. Skipping render as long as the orientation is invalid.",
          link->getName().c_str());
        CRAS_ERROR_THROTTLE_NAMED(1.0, "link_updater", "%s", error.c_str());
      }
      else if (collision_position.isNaN())
      {
        error = cras::format(
          "collision position of %s contains NaNs. Skipping render as long as the position is invalid.",
          link->getName().c_str());
        CRAS_ERROR_THROTTLE_NAMED(1.0, "link_updater", "%s", error.c_str());
      }

      if (!error.empty())
      {
        errors.linkErrors[link->getName()].name = link->getName();
        errors.linkErrors[link->getName()].error = error;
      }
      else
      {
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
    }
    else
    {
      link->setToErrorMaterial();
      errors.linkErrors[link->getName()] = transformResult.error();
    }
  }
  this->update_errors_ = errors;

  if (errors.hasError())
    return cras::make_unexpected(errors);
  return {};
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

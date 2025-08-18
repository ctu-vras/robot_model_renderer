// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: 2008 Willow Garage, Inc.

#pragma once

// This file is taken from rviz and slightly edited to be usable in this package.

#include <map>
#include <memory>
#include <string>

#include <OgrePrerequisites.h>
#include <OgreQuaternion.h>

#include <urdf/model.h>

#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/log_utils.h>
#include <robot_model_renderer/ogre_helpers/ogre_vector.hpp>
#include <robot_model_renderer/robot/link_updater.hpp>
#include <robot_model_renderer/robot/robot_link.hpp>
#include <robot_model_renderer/robot/robot_joint.hpp>
#include <robot_model_renderer/robot/shape_filter.hpp>
#include <robot_model_renderer/robot/shape_inflation_registry.hpp>

namespace robot_model_renderer
{

struct RobotErrors
{
  std::vector<LinkError> linkErrors;
  std::vector<JointError> jointErrors;

  bool hasError(bool includeMaterialErrors = true) const;
};

struct UpdateErrors
{
  std::string error;
  std::unordered_map<std::string, LinkUpdateError> linkErrors;

  bool hasError() const;
  std::string toString() const;
};

/**
 * \brief A helper class to draw a representation of a robot, as specified by a URDF.
 *
 * Can display either the visual models of the robot or the collision models.
 */
class Robot : public cras::HasLogger
{
public:
  Robot(const cras::LogHelperPtr& log, Ogre::SceneNode* root_node, Ogre::SceneManager* scene_manager,
    const std::string& name);

  virtual ~Robot();

  /**
   * \brief Loads meshes/primitives from a robot description.  Calls clear() before loading.
   *
   * \param[in] urdf The robot description to read from
   * \param[in] shape_filter Filter for the shapes to load
   * \param[in] shape_inflation_registry Registry of per-shape scaling and padding
   */
  virtual cras::expected<void, RobotErrors> load(
    const urdf::ModelInterface& urdf, const std::shared_ptr<ShapeFilter>& shape_filter,
    const std::shared_ptr<ShapeInflationRegistry>& shape_inflation_registry);

  /**
   * \brief Clears all data loaded from a URDF
   */
  virtual void clear();

  virtual cras::expected<void, UpdateErrors> update(LinkUpdater& updater, const ros::Time& time);

  /**
   * \brief Set the robot as a whole to be visible or not
   *
   * \param[in] visible Should we be visible?
   */
  virtual void setVisible(bool visible);

  /**
   * \brief Set whether the visual meshes of the robot should be visible
   *
   * \param[in] visible Whether the visual meshes of the robot should be visible
   */
  void setVisualVisible(bool visible);

  /**
   * \brief Set whether the collision meshes/primitives of the robot should be visible
   *
   * \param[in] visible Whether the collision meshes/primitives should be visible
   */
  void setCollisionVisible(bool visible);

  /**
   * \return Whether anything is visible
   */
  bool isVisible() const;

  /**
   * \return Whether the visual representation is set to be visible. To be visible this and isVisible()
   *         must both be true.
   */
  bool isVisualVisible() const;

  /**
   * \return Whether the collision representation is set to be visible. To be visible this and isVisible()
   *         must both be true.
   */
  bool isCollisionVisible() const;

  const RobotErrors& getErrors() const;

  const UpdateErrors& getUpdateErrors() const;

  void setAlpha(float a);

  float getAlpha() const
  {
    return alpha_;
  }

  RobotLink* getRootLink() const
  {
    return root_link_;
  }

  RobotLink* getLink(const std::string& name) const;

  RobotJoint* getJoint(const std::string& name) const;

  typedef std::map<std::string, RobotLink*> M_NameToLink;
  typedef std::map<std::string, RobotJoint*> M_NameToJoint;

  const M_NameToLink& getLinks() const
  {
    return links_;
  }

  const M_NameToJoint& getJoints() const
  {
    return joints_;
  }

  const std::string& getName()
  {
    return name_;
  }

  Ogre::SceneNode* getVisualNode()
  {
    return root_visual_node_;
  }

  Ogre::SceneNode* getCollisionNode()
  {
    return root_collision_node_;
  }

  Ogre::SceneManager* getSceneManager()
  {
    return scene_manager_;
  }

  virtual void setPosition(const Ogre::Vector3& position);

  virtual void setOrientation(const Ogre::Quaternion& orientation);

  virtual void setMaskMode();

  virtual void unsetMaskMode();

  virtual void setColorMode(float red, float green, float blue);

  virtual void unsetColorMode();

  virtual const Ogre::Vector3& getPosition();

  virtual const Ogre::Quaternion& getOrientation();

  virtual cras::expected<RobotLink*, LinkError> createLink(Robot* robot, const urdf::LinkConstSharedPtr& link,
    const std::string& parent_joint_name, const std::shared_ptr<ShapeFilter>& shape_filter,
    const std::shared_ptr<ShapeInflationRegistry>& shape_inflation_registry);

  virtual cras::expected<RobotJoint*, JointError> createJoint(Robot* robot, const urdf::JointConstSharedPtr& joint);

protected:
  /**
   * \brief Call RobotLink::updateVisibility() on each link.
   */
  void updateLinkVisibilities();

  Ogre::SceneManager* scene_manager_;

  M_NameToLink links_;  //!< Map of name to link info, stores all loaded links.
  M_NameToJoint joints_;  //!< Map of name to joint info, stores all loaded joints.
  RobotLink* root_link_;

  Ogre::SceneNode* root_visual_node_;  //!< Node all our visual nodes are children of
  Ogre::SceneNode* root_collision_node_;  //!< Node all our collision nodes are children of

  bool visible_;  //!< Should we show anything at all? (affects visual, collision, axes, and trails)
  bool visual_visible_;  //!< Should we show the visual representation?
  bool collision_visible_;  //!< Should we show the collision representation?

  bool robot_loaded_;  //!< true after robot model is loaded.

  RobotErrors errors_;  //!< Errors from load()
  UpdateErrors update_errors_;  //!< Errors from update()

  std::string name_;
  float alpha_;
};

}

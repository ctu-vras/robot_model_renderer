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

#pragma once

// This file is taken from rviz and slightly edited to be usable in this package.

#include <string>
#include <map>

#include <OgrePrerequisites.h>
#include <OgreQuaternion.h>

#include <urdf/model.h>

#include <robot_model_renderer/ogre_helpers/ogre_vector.h>
#include <robot_model_renderer/robot/link_updater.h>

namespace robot_model_renderer
{

class Object;
class Robot;
class RobotLink;
class RobotJoint;

/**
 * \brief A helper class to draw a representation of a robot, as specified by a URDF.
 *
 * Can display either the visual models of the robot or the collision models.
 */
class Robot
{
public:
  Robot(Ogre::SceneNode* root_node, Ogre::SceneManager* scene_manager, const std::string& name);

  virtual ~Robot();

  /**
   * \brief Loads meshes/primitives from a robot description.  Calls clear() before loading.
   *
   * \param[in] urdf The robot description to read from
   * \param[in] visual Whether or not to load the visual representation
   * \param[in] collision Whether or not to load the collision representation
   */
  virtual void load(const urdf::ModelInterface& urdf, bool visual, bool collision);

  /**
   * \brief Clears all data loaded from a URDF
   */
  virtual void clear();

  virtual void update(const LinkUpdater& updater, const ros::Time& time);

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

  Ogre::SceneNode* getOtherNode()
  {
    return root_other_node_;
  }

  Ogre::SceneManager* getSceneManager()
  {
    return scene_manager_;
  }

  virtual void setPosition(const Ogre::Vector3& position);

  virtual void setOrientation(const Ogre::Quaternion& orientation);

  virtual void setScale(const Ogre::Vector3& scale);

  virtual void setMaskMode();

  virtual void unsetMaskMode();

  virtual void setColorMode(float red, float green, float blue);

  virtual void unsetColorMode();

  virtual const Ogre::Vector3& getPosition();

  virtual const Ogre::Quaternion& getOrientation();

  virtual RobotLink* createLink(Robot* robot, const urdf::LinkConstSharedPtr& link,
    const std::string& parent_joint_name, bool visual, bool collision);

  virtual RobotJoint* createJoint(Robot* robot, const urdf::JointConstSharedPtr& joint);

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
  Ogre::SceneNode* root_other_node_;

  bool visible_;  //!< Should we show anything at all? (affects visual, collision, axes, and trails)
  bool visual_visible_;  //!< Should we show the visual representation?
  bool collision_visible_;  //!< Should we show the collision representation?

  bool robot_loaded_;  //!< true after robot model is loaded.

  std::string name_;
  float alpha_;
};

}

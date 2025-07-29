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

#include <map>
#include <string>
#include <utility>
#include <vector>

#include <OgreMaterial.h>
#include <OgrePrerequisites.h>
#include <OgreQuaternion.h>
#include <OgreSharedPtr.h>

#include <urdf/model.h>
#include <urdf_model/pose.h>

#include <robot_model_renderer/ogre_helpers/object.h>
#include <robot_model_renderer/ogre_helpers/ogre_vector.h>

namespace Ogre
{

class Any;

}

namespace robot_model_renderer
{

class Shape;
class Robot;
class RobotJoint;

/**
 * \brief Contains any data we need from a link in the robot.
 */
class RobotLink
{
  enum MaterialMode
  {
    ORIGINAL = 0,
    COLOR = 1,
    ERROR = 2,
  };

public:
  RobotLink(Robot* robot, const urdf::LinkConstSharedPtr& link, const std::string& parent_joint_name,
    bool visual, bool collision);

  virtual ~RobotLink();

  virtual void setRobotAlpha(float a);

  virtual void setTransforms(const Ogre::Vector3& visual_position, const Ogre::Quaternion& visual_orientation,
    const Ogre::Vector3& collision_position, const Ogre::Quaternion& collision_orientation);

  const std::string& getName() const
  {
    return name_;
  }

  const std::string& getParentJointName() const
  {
    return parent_joint_name_;
  }

  const std::vector<std::string>& getChildJointNames() const
  {
    return child_joint_names_;
  }

  Ogre::SceneNode* getVisualNode() const
  {
    return visual_node_;
  }

  Ogre::SceneNode* getCollisionNode() const
  {
    return collision_node_;
  }

  Robot* getRobot() const
  {
    return robot_;
  }

  std::string getGeometryErrors() const;

  void setToErrorMaterial();

  void setToNormalMaterial();

  void setColor(float red, float green, float blue);

  void unsetColor();

  Ogre::Vector3 getPosition() const;

  Ogre::Quaternion getOrientation() const;

  bool hasGeometry() const;

  /**
   * \brief If set to true, the link will only render to the depth channel and be in render group 0, so it is rendered
   *        before anything else. Thus, it will occlude other objects without being visible.
   *
   * \param[in] onlyRenderDepth Whether to render only depth.
   */
  void setOnlyRenderDepth(bool onlyRenderDepth);

  bool getOnlyRenderDepth() const
  {
    return only_render_depth_;
  }

  /**
   * \brief Update the visibility of the link elements: visual mesh, collision mesh, trail, and axes.
   *
   * Called by Robot when changing visual and collision visibilities, since each link may be enabled or disabled.
   */
  void updateVisibility();

private:
  void updateAlpha();

  void setMaterialMode(unsigned char mode_flags);

  void setRenderQueueGroup(Ogre::uint8 group);

  bool getEnabled() const;

  void createEntityForGeometryElement(const urdf::LinkConstSharedPtr& link, const urdf::Geometry& geom,
    const urdf::MaterialSharedPtr& material, const urdf::Pose& origin, Ogre::SceneNode* scene_node,
    Ogre::Entity*& entity);

  void addError(const char* format, ...);

  void createVisual(const urdf::LinkConstSharedPtr& link);

  void createCollision(const urdf::LinkConstSharedPtr& link);

  Ogre::MaterialPtr getMaterialForLink(const urdf::LinkConstSharedPtr& link, urdf::MaterialConstSharedPtr material);

protected:
  Robot* robot_;
  Ogre::SceneManager* scene_manager_;

  std::string name_;  //!< Name of this link
  std::string parent_joint_name_;
  std::vector<std::string> child_joint_names_;

  // maintain the original material of each SubEntity to restore it after unsetColor()
  using M_SubEntityToMaterial = std::map<Ogre::SubEntity*, std::pair<Ogre::MaterialPtr, Ogre::MaterialPtr>>;
  M_SubEntityToMaterial materials_;
  Ogre::MaterialPtr default_material_;
  std::string default_material_name_;

  //! The entities representing the visual mesh of this link (if they exist)
  std::vector<Ogre::Entity*> visual_meshes_;
  //! The entities representing the collision mesh of this link (if they exist)
  std::vector<Ogre::Entity*> collision_meshes_;

  Ogre::SceneNode* visual_node_;  //!< The scene node the visual meshes are attached to
  Ogre::SceneNode* collision_node_;  //!< The scene node the collision meshes are attached to

  float robot_alpha_;  //!< Alpha value from top-level robot alpha Property (set via setRobotAlpha()).

  bool only_render_depth_;

  std::string joint_name_;

  bool enabled_;
  std::string errors_;

  Ogre::MaterialPtr color_material_;
  unsigned char material_mode_flags_;
};

}

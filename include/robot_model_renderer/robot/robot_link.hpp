// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: 2008 Willow Garage, Inc.

#pragma once

// This file is taken from rviz and slightly edited to be usable in this package.

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <OgreMaterial.h>
#include <OgrePrerequisites.h>
#include <OgreQuaternion.h>
#include <OgreSharedPtr.h>

#include <urdf/model.h>
#include <urdf_model/pose.h>

#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/optional.hpp>
#include <robot_model_renderer/ogre_helpers/ogre_vector.hpp>
#include <robot_model_renderer/robot/shape_filter.hpp>
#include <robot_model_renderer/robot/shape_inflation_registry.hpp>

namespace Ogre
{

class Any;

}

namespace robot_model_renderer
{

class Shape;
class Robot;
class RobotJoint;

struct MaterialError
{
  std::string name;
  std::string error;
  cras::optional<std::string> filename;

  bool hasError() const;
};

struct GeometryError
{
  std::string name;
  std::string error;
  cras::optional<std::string> filename;
  cras::optional<MaterialError> materialError;

  bool hasError(bool includeMaterialErrors = true) const;
};

struct LinkError
{
  std::string link;
  std::string error;
  std::vector<GeometryError> visualErrors;
  std::vector<GeometryError> collisionErrors;

  bool hasError(bool includeMaterialErrors = true) const;
};

/**
 * \brief Contains any data we need from a link in the robot.
 */
class RobotLink : public cras::HasLogger
{
  enum MaterialMode
  {
    ORIGINAL = 0,
    COLOR = 1,
    ERROR = 2,
    MASK = 4,
  };

public:
  RobotLink(const cras::LogHelperPtr& log, Robot* robot, const urdf::LinkConstSharedPtr& link,
    const std::string& parent_joint_name, const std::shared_ptr<ShapeFilter>& shape_filter,
    const std::shared_ptr<ShapeInflationRegistry>& shape_inflation_registry);

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

  const LinkError& getErrors() const;

  void setToErrorMaterial();

  void setToNormalMaterial();

  void setMaskMode();

  void unsetMaskMode();

  void setColorMode(float red, float green, float blue);

  void unsetColorMode();

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

  Ogre::Entity* createEntityForGeometryElement(const urdf::LinkConstSharedPtr& link,
    const urdf::Geometry& geom, const urdf::MaterialSharedPtr& material, const urdf::Pose& origin,
    Ogre::SceneNode* scene_node, const ScaleAndPadding& inflation, GeometryError& error);

  std::vector<GeometryError> createVisuals(const urdf::LinkConstSharedPtr& link,
    const std::shared_ptr<ShapeFilter>& shape_filter,
    const std::shared_ptr<ShapeInflationRegistry>& shape_inflation_registry);

  std::vector<GeometryError> createCollisions(const urdf::LinkConstSharedPtr& link,
    const std::shared_ptr<ShapeFilter>& shape_filter,
    const std::shared_ptr<ShapeInflationRegistry>& shape_inflation_registry);

  Ogre::MaterialPtr getMaterialForLink(const urdf::LinkConstSharedPtr& link, urdf::MaterialConstSharedPtr material,
    MaterialError& error);

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
  LinkError errors_;

  Ogre::MaterialPtr color_material_;
  Ogre::MaterialPtr mask_material_;
  unsigned char material_mode_flags_;
};

}

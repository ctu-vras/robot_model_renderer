// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: 2008 Willow Garage, Inc.

#pragma once

// This file is taken from rviz and minimally edited (just code style and different namespace).

#include <OgreMaterial.h>
#include <OgreSharedPtr.h>

#include <robot_model_renderer/ogre_helpers/object.h>
#include <robot_model_renderer/ogre_helpers/ogre_vector.h>

namespace Ogre
{

class SceneManager;
class SceneNode;
class Any;
class Entity;

}

namespace robot_model_renderer
{

class Shape : public Object
{
public:
  enum Type
  {
    Cone,
    Cube,
    Cylinder,
    Sphere,
    Mesh,
  };

  /**
   * \brief Constructor
   *
   * \param[in] shape_type The type of the shape.
   * \param[in] scene_manager The scene manager this object is associated with
   * \param[in] parent_node A scene node to use as the parent of this object.  If NULL, uses the root scene node.
   */
  Shape(Type shape_type, Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node = nullptr);

  ~Shape() override;

  Type getType() const
  {
    return type_;
  }

  /**
   * \brief Set the offset for this shape
   *
   * The default is no offset, which puts the pivot point directly in the center of the object.
   *
   * \param[in] offset Amount to offset the center of the object from the pivot point
   */
  void setOffset(const Ogre::Vector3& offset);

  void setColor(float r, float g, float b, float a) override;

  void setColor(const Ogre::ColourValue& c);

  void setPosition(const Ogre::Vector3& position) override;

  void setOrientation(const Ogre::Quaternion& orientation) override;

  void setScale(const Ogre::Vector3& scale) override;

  const Ogre::Vector3& getPosition() override;

  const Ogre::Quaternion& getOrientation() override;

  /**
   * \brief Get the root scene node (pivot node) for this object
   *
   * \return The root scene node of this object
   */
  Ogre::SceneNode* getRootNode() const
  {
    return scene_node_;
  }

  void setUserData(const Ogre::Any& data) override;

  Ogre::Entity* getEntity() const
  {
    return entity_;
  }

  Ogre::MaterialPtr getMaterial() const
  {
    return material_;
  }

  static Ogre::Entity* createEntity(const std::string& name, Type shape_type, Ogre::SceneManager* scene_manager);

protected:
  Ogre::SceneNode* scene_node_;
  Ogre::SceneNode* offset_node_;
  Ogre::Entity* entity_;
  Ogre::MaterialPtr material_;
  std::string material_name_;

  Type type_;
};

}

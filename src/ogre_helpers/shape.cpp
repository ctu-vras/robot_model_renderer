// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: 2008 Willow Garage, Inc.

// This file is taken from rviz and edited (removed most of the file).

#include <robot_model_renderer/ogre_helpers/shape.hpp>

#include <cstdint>

#include <OgreEntity.h>
#include <OgreSceneManager.h>

#include <ros/assert.h>

namespace robot_model_renderer
{

Ogre::Entity* Shape::createEntity(const std::string& name, const Type shape_type, Ogre::SceneManager* scene_manager)
{
  if (shape_type == Mesh)
    return nullptr;  // the entity is initialized after the vertex data was specified

  std::string mesh_name;
  switch (shape_type)
  {
  case Cone:
    mesh_name = "rviz_cone.mesh";
    break;

  case Cube:
    mesh_name = "rviz_cube.mesh";
    break;

  case Cylinder:
    mesh_name = "rviz_cylinder.mesh";
    break;

  case Sphere:
    mesh_name = "rviz_sphere.mesh";
    break;

  default:
    ROS_BREAK();
  }

  return scene_manager->createEntity(name, mesh_name);
}

}

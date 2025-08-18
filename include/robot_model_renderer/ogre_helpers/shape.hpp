// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: 2008 Willow Garage, Inc.

#pragma once

// This file is taken from rviz and edited (most of the file removed).

#include <string>

#include <OgrePrerequisites.h>

namespace Ogre
{

class SceneManager;
class Entity;

}

namespace robot_model_renderer
{

class Shape
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

  static Ogre::Entity* createEntity(const std::string& name, Type shape_type, Ogre::SceneManager* scene_manager);
};

}

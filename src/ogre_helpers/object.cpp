// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: 2008 Willow Garage, Inc.

// This file is taken from rviz and minimally edited (just code style and different namespace).

#include <robot_model_renderer/ogre_helpers/object.h>

#include <OgreSceneManager.h>

namespace robot_model_renderer
{

Object::Object(Ogre::SceneManager* scene_manager) : scene_manager_(scene_manager)
{
}

Object::~Object() = default;

}

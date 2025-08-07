// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: 2018 Fizyr BV
// SPDX-FileCopyrightText: 2019 Bielefeld University

#pragma once

// This file is taken from rviz and minimally edited (just code style and different namespace).

#include <string>

#include <robot_model_renderer/ogre_helpers/version_check.h>

#include <OgreSimpleRenderable.h>
#include <OgreSceneNode.h>

#if OGRE_VERSION < OGRE_VERSION_CHECK(1, 10, 8)
#include <OgreSceneManager.h>
#else
#include <OgreMaterialManager.h>
#endif

namespace robot_model_renderer
{

/* This header provides helper functions to maintain compatibility with Ogre versions 1.9 ... 1.12+.
 *
 * setMaterial() allows setting the material of a renderable by either name or MaterialPtr.
 * OGRE 1.10.8 added: renderable.setMaterial(const Ogre::MaterialPtr &)
 * OGRE 1.11 removed: renderable.setMaterial(const std::string       &)
 *
 * removeAndDestroyChildNode(parent, child) allows removal of a SceneNode*.
 * OGRE 1.10.8 added: SceneNode::removeAndDestroyChild(SceneNode* child)
 */

#if OGRE_VERSION < OGRE_VERSION_CHECK(1, 10, 8)
inline void setMaterial(Ogre::SimpleRenderable& renderable, const std::string& material_name)
{
  renderable.setMaterial(material_name);
}

inline void setMaterial(Ogre::SimpleRenderable& renderable, const Ogre::MaterialPtr& material)
{
  renderable.setMaterial(material->getName());
}
#else
inline void setMaterial(Ogre::SimpleRenderable& renderable, const std::string& material_name)
{
  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().getByName(material_name);
  // OGRE 1.11 also deprecated their own SharedPtr class and switched to std::shared_ptr.
  // Checking for nullptr with .get() works in both versions.
  if (!material.get())
  {
    OGRE_EXCEPT(Ogre::Exception::ERR_ITEM_NOT_FOUND, "Could not find material " + material_name,
                "SimpleRenderable::setMaterial");
  }
  renderable.setMaterial(material);
}

inline void setMaterial(Ogre::SimpleRenderable& renderable, const Ogre::MaterialPtr& material)
{
  renderable.setMaterial(material);
}
#endif

#if OGRE_VERSION < OGRE_VERSION_CHECK(1, 10, 8)
inline void removeAndDestroyChildNode(Ogre::SceneNode* parent, Ogre::SceneNode* child)
{
  child->removeAndDestroyAllChildren();
  parent->removeChild(child);
  child->getCreator()->destroySceneNode(child);
}
#else
inline void removeAndDestroyChildNode(Ogre::SceneNode* parent, Ogre::SceneNode* child)
{
  parent->removeAndDestroyChild(child);
}
#endif

}

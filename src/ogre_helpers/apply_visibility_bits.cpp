// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: 2012 Willow Garage, Inc.

// This file is taken from rviz and minimally edited (just code style and different namespace).

#include <robot_model_renderer/ogre_helpers/apply_visibility_bits.h>

#include <cstdint>

#include <OgreMovableObject.h>
#include <OgreSceneNode.h>
#include <OgreIteratorWrapper.h>

namespace robot_model_renderer
{

void applyVisibilityBits(const uint32_t bits, Ogre::SceneNode* node)
{
  if (!node)
  {
    return;
  }
  // Loop over all objects attached to this node.
  Ogre::SceneNode::ObjectIterator obj_it = node->getAttachedObjectIterator();
  while (obj_it.hasMoreElements())
  {
    Ogre::MovableObject* obj = obj_it.getNext();
    obj->setVisibilityFlags(bits);
  }
  // Loop over and recurse into all child nodes.
  Ogre::SceneNode::ChildNodeIterator child_it = node->getChildIterator();
  while (child_it.hasMoreElements())
  {
    const auto child = dynamic_cast<Ogre::SceneNode*>(child_it.getNext());
    applyVisibilityBits(bits, child);
  }
}

}

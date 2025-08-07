// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: 2008 Willow Garage, Inc.

// This file is taken from rviz and minimally edited (just code style and different namespace).

#include <robot_model_renderer/ogre_helpers/initialization.h>

#include <OgreRoot.h>

namespace robot_model_renderer
{

void cleanupOgre()
{
  delete Ogre::Root::getSingletonPtr();
}

// This should be folded into RenderSystem, but it should work fine as is, so I'm leaving it for now.
void initializeResources(const V_string& resource_paths)
{
  auto path_it = resource_paths.begin();
  const auto path_end = resource_paths.end();
  for (; path_it != path_end; ++path_it)
  {
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
        *path_it, "FileSystem", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  }

  Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
}

}

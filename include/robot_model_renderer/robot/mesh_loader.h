// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: 2010 Willow Garage, Inc.

#pragma once

// This file is taken from rviz and slightly edited to be usable in this package.

#include <string>

#include <OgreMesh.h>
#include <OgreSkeleton.h>

namespace robot_model_renderer
{

Ogre::MeshPtr loadMeshFromResource(const std::string& resource_path, bool enable_shadow_buffers = false);

Ogre::SkeletonPtr loadSkeletonFromResource(const std::string& resource_path, bool enable_shadow_buffers = false);

}

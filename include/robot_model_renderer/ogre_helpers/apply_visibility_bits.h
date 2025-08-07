// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: 2012 Willow Garage, Inc.

#pragma once

// This file is taken from rviz and minimally edited (just code style and different namespace).

#include <cstdint>

namespace Ogre
{

class SceneNode;

}

namespace robot_model_renderer
{

void applyVisibilityBits(uint32_t bits, Ogre::SceneNode* node);

}

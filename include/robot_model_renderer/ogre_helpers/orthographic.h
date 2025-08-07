// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: 2008 Willow Garage, Inc.

#pragma once

// This file is taken from rviz and minimally edited (just code style and different namespace).

namespace Ogre
{

class Matrix4;

}

namespace robot_model_renderer
{

void buildScaledOrthoMatrix(
  Ogre::Matrix4& proj, float left, float right, float bottom, float top, float near, float far);

}

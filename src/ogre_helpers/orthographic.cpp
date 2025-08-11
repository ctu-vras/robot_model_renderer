// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: 2012 Willow Garage, Inc.

// This file is taken from rviz and minimally edited (just code style and different namespace).

#include <robot_model_renderer/ogre_helpers/orthographic.hpp>

#include <OgreMatrix4.h>

namespace robot_model_renderer
{

void buildScaledOrthoMatrix(Ogre::Matrix4& proj,
  const float left, const float right, const float bottom, const float top, const float near, const float far)
{
  const float invw = 1 / (right - left);
  const float invh = 1 / (top - bottom);
  const float invd = 1 / (far - near);

  proj = Ogre::Matrix4::ZERO;
  proj[0][0] = 2 * invw;
  proj[0][3] = -(right + left) * invw;
  proj[1][1] = 2 * invh;
  proj[1][3] = -(top + bottom) * invh;
  proj[2][2] = -2 * invd;
  proj[2][3] = -(far + near) * invd;
  proj[3][3] = 1;
}

}

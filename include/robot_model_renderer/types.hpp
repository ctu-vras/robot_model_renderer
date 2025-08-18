// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

/**
 * \file
 * \brief Common types.
 * \author Martin Pecka
 */


namespace robot_model_renderer
{

/**
 * \brief Mode of robot model rendering.
 */
enum class RenderingMode
{
  NORMAL,  //!< Normal mode, visual meshes use their textures, collision meshes use red or link color.
  COLOR,  //!< All meshes are rendered with the specified color (with lighting effects).
  MASK,  //!< All meshes are rendered as a binary mask (robot = white, background = black).
};

}

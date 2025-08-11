// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

#include <OgrePixelFormat.h>

namespace robot_model_renderer
{

  int ogrePixelFormatToCvMatType(const Ogre::PixelFormat& pf);

}

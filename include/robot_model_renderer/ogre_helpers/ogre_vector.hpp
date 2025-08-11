// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Willow Garage, Inc.

#pragma once

// This file is taken from rviz and minimally edited (just code style and different namespace).

#include <OgrePrerequisites.h>
#include <robot_model_renderer/ogre_helpers/version_check.hpp>

#if OGRE_VERSION < OGRE_VERSION_CHECK(1, 12, 0)
#include <OgreVector2.h>
#include <OgreVector3.h>
#else
#include <OgreVector.h>
#endif

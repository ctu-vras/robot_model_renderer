#pragma once

#include <OgrePrerequisites.h>
#include <robot_model_renderer/ogre_helpers/version_check.h>

#if OGRE_VERSION < OGRE_VERSION_CHECK(1, 12, 0)
#include <OgreVector2.h>
#include <OgreVector3.h>
#else
#include <OgreVector.h>
#endif

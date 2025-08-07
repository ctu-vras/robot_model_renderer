// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: 2011 Willow Garage, Inc.

#pragma once

// This file is taken from rviz and minimally edited (just code style and different namespace).

#include <cstdint>

#include <OgreRoot.h>

namespace Ogre
{

class SceneManager;

}

namespace robot_model_renderer
{

class RenderSystem
{
public:
#if defined(Q_OS_MAC) || defined(Q_OS_WIN)
  typedef size_t WindowIDType;
#else
  typedef unsigned long WindowIDType;  // NOLINT(runtime/int)
#endif

  explicit RenderSystem(int force_gl_version = 0, bool use_antialiasing = true);

  Ogre::RenderWindow* makeRenderWindow(
    WindowIDType window_id, unsigned int width, unsigned int height, double pixel_ratio = 1.0);

  Ogre::Root* root() const
  {
    return ogre_root_;
  }

  // @brief return OpenGl Version as integer, e.g. 320 for OpenGl 3.20
  int getGlVersion() const
  {
    return gl_version_;
  }

  // @brief return GLSL Version as integer, e.g. 150 for GLSL 1.50
  int getGlslVersion() const
  {
    return glsl_version_;
  }

private:
  void loadOgrePlugins();

  // helper for makeRenderWindow()
  Ogre::RenderWindow* tryMakeRenderWindow(
    const std::string& name, unsigned int width, unsigned int height, const Ogre::NameValuePairList* params,
    int max_attempts);

  // Find and configure the render system.
  void setupRenderSystem();
  void setupResources();
  void detectGlVersion();

  // ID for a dummy window of size 1x1, used to keep Ogre happy.
  WindowIDType dummy_window_id_;

  Ogre::Root* ogre_root_;

  int gl_version_;
  int glsl_version_;
  bool use_anti_aliasing_;
  int force_gl_version_;
};

}

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: 2011 Willow Garage, Inc.

#pragma once

// This file is taken from rviz and minimally edited (just code style and different namespace).

#include <cstdint>
#include <mutex>

#include <OgreLog.h>
#include <OgreRoot.h>

#include <cras_cpp_common/log_utils.h>
#include <robot_model_renderer/ogre_helpers/ogre_logging.h>

namespace Ogre
{

class SceneManager;

}

namespace robot_model_renderer
{

class RenderSystem : cras::HasLogger
{
public:
#if defined(Q_OS_MAC) || defined(Q_OS_WIN)
  typedef size_t WindowIDType;
#else
  typedef unsigned long WindowIDType;  // NOLINT(runtime/int)
#endif

  explicit RenderSystem(const cras::LogHelperPtr& log,  int force_gl_version = 0, bool use_antialiasing = true);
  ~RenderSystem();

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

  Ogre::RenderWindow* window() const
  {
    return ogre_window_;
  }

  /**
   * \brief A LockGuard for rendering using the given render system. You have to hold it any time your code interacts
   *        with the GLX backend of the OGRE render system.
   */
  class RenderSystemLock : std::lock_guard<std::mutex>
  {
  public:
    explicit RenderSystemLock(RenderSystem* render_system);
    ~RenderSystemLock();
  private:
    RenderSystem* render_system_;
    Ogre::Log* previous_logger_;
  };

  /**
   * \brief Create a lock for rendering operations.
   * \return The rendering lock.
   */
  RenderSystemLock lock();

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

  void setContextCurrent();
  void endContextCurrent();

  // ID for a dummy window of size 1x1, used to keep Ogre happy.
  static WindowIDType dummy_window_id_;

  RosLogListener log_listener_;

  Ogre::Root* ogre_root_;
  Ogre::RenderWindow* ogre_window_;
  Ogre::Log* ogre_log_;

  int gl_version_;
  int glsl_version_;
  bool use_anti_aliasing_;
  int force_gl_version_;
  bool did_init_ogre_root_;

  static std::mutex render_system_mutex_;
  static bool render_system_inited_;
};

}

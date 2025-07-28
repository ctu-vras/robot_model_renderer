/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// This file is taken from rviz and minimally edited (just code style and different namespace).

#include <robot_model_renderer/ogre_helpers/render_system.h>

#include <GL/glx.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>

#include <robot_model_renderer/ogre_helpers/version_check.h>
#include <OgreRenderWindow.h>
#include <OgreSceneManager.h>

#include <robot_model_renderer/ogre_helpers/env_config.h>
#include <robot_model_renderer/ogre_helpers/ogre_logging.h>
#include <ros/console.h>
#include <ros/package.h> // This dependency should be moved out of here, it is just used for a search path.

namespace robot_model_renderer
{

RenderSystem* RenderSystem::instance_ = nullptr;

int RenderSystem::force_gl_version_ = 0;

bool RenderSystem::use_anti_aliasing_ = true;

RenderSystem* RenderSystem::get()
{
  if (instance_ == nullptr)
  {
    instance_ = new RenderSystem();
  }
  return instance_;
}

void createColorMaterial(const std::string& name, const Ogre::ColourValue& color, const bool use_self_illumination)
{
  Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create(
    name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  mat->setAmbient(color * 0.5f);
  mat->setDiffuse(color);
  if (use_self_illumination)
  {
    mat->setSelfIllumination(color);
  }
  mat->setLightingEnabled(true);
  mat->setReceiveShadows(false);
}

void createColorMaterials()
{
  createColorMaterial("RVIZ/Red", Ogre::ColourValue(1.0f, 0.0f, 0.0f, 1.0f), true);
  createColorMaterial("RVIZ/Green", Ogre::ColourValue(0.0f, 1.0f, 0.0f, 1.0f), true);
  createColorMaterial("RVIZ/Blue", Ogre::ColourValue(0.0f, 0.0f, 1.0f, 1.0f), true);
  createColorMaterial("RVIZ/Cyan", Ogre::ColourValue(0.0f, 1.0f, 1.0f, 1.0f), true);
  createColorMaterial("RVIZ/ShadedRed", Ogre::ColourValue(1.0f, 0.0f, 0.0f, 1.0f), false);
  createColorMaterial("RVIZ/ShadedGreen", Ogre::ColourValue(0.0f, 1.0f, 0.0f, 1.0f), false);
  createColorMaterial("RVIZ/ShadedBlue", Ogre::ColourValue(0.0f, 0.0f, 1.0f, 1.0f), false);
  createColorMaterial("RVIZ/ShadedCyan", Ogre::ColourValue(0.0f, 1.0f, 1.0f, 1.0f), false);
}

void RenderSystem::forceGlVersion(int version)
{
  force_gl_version_ = version;
  ROS_INFO_STREAM("Forcing OpenGl version " << (float)version / 100.0 << ".");
}

void RenderSystem::disableAntiAliasing()
{
  use_anti_aliasing_ = false;
  ROS_INFO("Disabling Anti-Aliasing");
}

RenderSystem::RenderSystem()
{
  OgreLogging::useRosLog();
  OgreLogging::configureLogging();

  setupDummyWindowId();
  ogre_root_ = new Ogre::Root("");
  loadOgrePlugins();
  setupRenderSystem();
  ogre_root_->initialise(false);
  makeRenderWindow(dummy_window_id_, 1, 1);
  detectGlVersion();
  setupResources();
  Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
  createColorMaterials();
}

void RenderSystem::setupDummyWindowId()
{
  Display* display = XOpenDisplay(nullptr);

  if (display == nullptr)
  {
    ROS_WARN("$DISPLAY is invalid, falling back on default :0");
    display = XOpenDisplay(":0");

    if (display == nullptr)
    {
      ROS_FATAL("Can't open default or :0 display. Try setting DISPLAY environment variable.");
      throw std::runtime_error("Can't open default or :0 display!\n");
    }
  }

  int screen = DefaultScreen(display);

  int attribList[] = {GLX_RGBA, GLX_DOUBLEBUFFER, GLX_DEPTH_SIZE, 16, GLX_STENCIL_SIZE, 8, None};

  XVisualInfo* visual = glXChooseVisual(display, screen, (int*)attribList);

  dummy_window_id_ = XCreateSimpleWindow(display, RootWindow(display, screen), 0, 0, 1, 1, 0, 0, 0);

  GLXContext context = glXCreateContext(display, visual, nullptr, 1);

  glXMakeCurrent(display, dummy_window_id_, context);
}

void RenderSystem::loadOgrePlugins()
{
  std::string plugin_prefix = get_ogre_plugin_path() + "/";
#ifdef Q_OS_MAC
  plugin_prefix += "lib";
#endif
  ogre_root_->loadPlugin(plugin_prefix + "RenderSystem_GL");
#if OGRE_VERSION >= OGRE_VERSION_CHECK(1, 11, 0)
  ogre_root_->loadPlugin(plugin_prefix + "Codec_FreeImage");
#endif
}

void RenderSystem::detectGlVersion()
{
  if (force_gl_version_)
  {
    gl_version_ = force_gl_version_;
  }
  else
  {
    Ogre::RenderSystem* renderSys = ogre_root_->getRenderSystem();
    const Ogre::RenderSystemCapabilities* caps = renderSys->createRenderSystemCapabilities();
    ROS_INFO("OpenGL device: %s", caps->getDeviceName().c_str());
    int major = caps->getDriverVersion().major;
    int minor = caps->getDriverVersion().minor;
    gl_version_ = major * 100 + minor * 10;
  }

  switch (gl_version_)
  {
  case 200:
    glsl_version_ = 110;
    break;
  case 210:
    glsl_version_ = 120;
    break;
  case 300:
    glsl_version_ = 130;
    break;
  case 310:
    glsl_version_ = 140;
    break;
  case 320:
    glsl_version_ = 150;
    break;
  default:
    if (gl_version_ > 320)
    {
      glsl_version_ = gl_version_;
    }
    else
    {
      glsl_version_ = 0;
    }
    break;
  }
  ROS_INFO("OpenGl version: %.1f (GLSL %.1f).", (float)gl_version_ / 100.0, (float)glsl_version_ / 100.0);
}

void RenderSystem::setupRenderSystem()
{
  Ogre::RenderSystem* renderSys;
  // Get the list of available renderers.
  const Ogre::RenderSystemList* rsList = &(ogre_root_->getAvailableRenderers());

  // Look for the OpenGL one, which we require.
  renderSys = nullptr;
  for (unsigned int i = 0; i < rsList->size(); i++)
  {
    renderSys = rsList->at(i);
    if (renderSys->getName().compare("OpenGL Rendering Subsystem") == 0)
    {
      break;
    }
  }

  if (renderSys == nullptr)
  {
    throw std::runtime_error("Could not find the opengl rendering subsystem!\n");
  }

  // We operate in windowed mode
  renderSys->setConfigOption("Full Screen", "No");

  // Set the Full Screen Anti-Aliasing factor.
  if (use_anti_aliasing_)
  {
    renderSys->setConfigOption("FSAA", "4");
  }

  ogre_root_->setRenderSystem(renderSys);
}

void RenderSystem::setupResources()
{
  std::string path = ros::package::getPath(ROS_PACKAGE_NAME);
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
      path + "/ogre_media", "FileSystem", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
      path + "/ogre_media/models", "FileSystem",
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
      path + "/ogre_media/materials/programs", "FileSystem",
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
      path + "/ogre_media/materials/scripts", "FileSystem",
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  // Add paths exported to the "media_export" package.
  std::vector<std::string> media_paths;
  ros::package::getPlugins("media_export", "ogre_media_path", media_paths);
  std::string delim(":");
  for (auto iter = media_paths.begin(); iter != media_paths.end(); ++iter)
  {
    if (!iter->empty())
    {
      std::string path;
      int pos1 = 0;
      int pos2 = iter->find(delim);
      while (pos2 != (int)std::string::npos)
      {
        path = iter->substr(pos1, pos2 - pos1);
        ROS_DEBUG("adding resource location: '%s'\n", path.c_str());
        Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
            path, "FileSystem", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        pos1 = pos2 + 1;
        pos2 = iter->find(delim, pos2 + 1);
      }
      path = iter->substr(pos1, iter->size() - pos1);
      ROS_DEBUG("adding resource location: '%s'\n", path.c_str());
      Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
          path, "FileSystem", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    }
  }
}

static bool x_baddrawable_error = false;

Ogre::RenderWindow* RenderSystem::makeRenderWindow(
  WindowIDType window_id, unsigned int width, unsigned int height, double pixel_ratio)
{
  static int windowCounter = 0; // Every RenderWindow needs a unique name, oy.

  Ogre::NameValuePairList params;
  Ogre::RenderWindow* window = nullptr;

  params["externalWindowHandle"] = Ogre::StringConverter::toString(window_id);
  params["parentWindowHandle"] = Ogre::StringConverter::toString(window_id);

  params["externalGLControl"] = true;

  // Enable antialiasing
  if (use_anti_aliasing_)
  {
    params["FSAA"] = "4";
  }

  // Set the macAPI for Ogre based on the Qt implementation
  params["contentScalingFactor"] = std::to_string(pixel_ratio);

  std::ostringstream stream;
  stream << "OgreWindow(" << windowCounter++ << ")";

  window = tryMakeRenderWindow(stream.str(), width, height, &params, 100);

  if (window == nullptr)
  {
    ROS_ERROR("Unable to create the rendering window after 100 tries.");
    assert(false);
  }

  if (window)
  {
    window->setActive(true);
    // window->setVisible(true);
    window->setAutoUpdated(false);
  }

  return window;
}

Ogre::RenderWindow* RenderSystem::tryMakeRenderWindow(
  const std::string& name, const unsigned int width, const unsigned int height, const Ogre::NameValuePairList* params,
  const int max_attempts)
{
  Ogre::RenderWindow* window = nullptr;
  int attempts = 0;

  while (window == nullptr && (attempts++) < max_attempts)
  {
    try
    {
      window = ogre_root_->createRenderWindow(name, width, height, false, params);

      // If the driver bug happened, tell Ogre we are done with that
      // window and then try again.
      if (x_baddrawable_error)
      {
        ogre_root_->detachRenderTarget(window);
        window = nullptr;
        x_baddrawable_error = false;
      }
    }
    catch (const std::exception& ex)
    {
      std::cerr << "rviz::RenderSystem: error creating render window: " << ex.what() << std::endl;
      window = nullptr;
    }
  }

  if (window && attempts > 1)
  {
    ROS_INFO("Created render window after %d attempts.", attempts);
  }

  return window;
}

}

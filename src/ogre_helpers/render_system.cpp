// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: 2011 Willow Garage, Inc.

// This file is taken from rviz and minimally edited (just code style and different namespace).

#include <robot_model_renderer/ogre_helpers/render_system.h>

#include <mutex>
#include <string>

#include <OgreRenderWindow.h>
#include <OgreSceneManager.h>
#include <RenderSystems/GL/OgreGLContext.h>
#include <RenderSystems/GL/OgreGLRenderSystem.h>

#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/string_utils.hpp>
#include <robot_model_renderer/ogre_helpers/env_config.h>
#include <robot_model_renderer/ogre_helpers/ogre_logging.h>
#include <robot_model_renderer/ogre_helpers/version_check.h>
#include <ros/console.h>
#include <ros/package.h>

namespace robot_model_renderer
{

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

RenderSystem::WindowIDType RenderSystem::dummy_window_id_ = 0;
std::mutex RenderSystem::render_system_mutex_ = {};
bool RenderSystem::render_system_inited_ = false;

RenderSystem::RenderSystem(const cras::LogHelperPtr& log, int force_gl_version, bool use_antialiasing) :
  cras::HasLogger(log), log_listener_(log), ogre_root_(nullptr), ogre_window_(nullptr), ogre_log_(nullptr),
  gl_version_(0), glsl_version_(0), use_anti_aliasing_(use_antialiasing), force_gl_version_(force_gl_version),
  did_init_ogre_root_(false)
{
  ogre_log_ = OgreLogging::configureLogging("Ogre.log", &this->log_listener_);

  std::lock_guard<std::mutex> l(render_system_mutex_);

  const auto previous_logger = Ogre::LogManager::getSingleton().setDefaultLog(ogre_log_);

  if (Ogre::Root::getSingletonPtr() == nullptr)
  {
    ogre_root_ = new Ogre::Root("");
    loadOgrePlugins();
    setupRenderSystem();
    ogre_root_->initialise(false);
    ogre_window_ = makeRenderWindow(0, 1, 1);
    detectGlVersion();
    setupResources();
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
    createColorMaterials();
    did_init_ogre_root_ = true;
    render_system_inited_ = true;
    endContextCurrent();
    CRAS_INFO_NAMED("renderer", "Initialized OGRE GL context.");
  }
  else
  {
    if (!render_system_inited_)
    {
      CRAS_FATAL("Ogre::Root singleton has been instantiated by some other code running in this process. "
                 "This will probably lead to errors or segfaults. robot_model_renderer is the only nodelet that "
                 "can run OpenGL/OGRE code in one nodelet manager. Sharing the same nodelet manager with other "
                 "OpenGL/OGRE programs will most probably fail.");
    }
    ogre_root_ = Ogre::Root::getSingletonPtr();
    ogre_root_->getRenderSystem()->registerThread();
    detectGlVersion();
  }

  Ogre::LogManager::getSingleton().setDefaultLog(previous_logger);
}

RenderSystem::~RenderSystem()
{
  std::lock_guard<std::mutex> l(render_system_mutex_);

  if (!did_init_ogre_root_)
    ogre_root_->getRenderSystem()->unregisterThread();

  Ogre::LogManager::getSingleton().destroyLog(ogre_log_);
}

RenderSystem::RenderSystemLock::RenderSystemLock(RenderSystem* render_system) :
  std::lock_guard<std::mutex>(RenderSystem::render_system_mutex_), render_system_(render_system)
{
  render_system_->setContextCurrent();
  previous_logger_ = Ogre::LogManager::getSingleton().setDefaultLog(render_system_->ogre_log_);
}

RenderSystem::RenderSystemLock::~RenderSystemLock()
{
  Ogre::LogManager::getSingleton().setDefaultLog(previous_logger_);
  render_system_->endContextCurrent();
}

RenderSystem::RenderSystemLock RenderSystem::lock()
{
  return RenderSystemLock(this);
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
    CRAS_INFO_NAMED("renderer", "OpenGL device: %s", caps->getDeviceName().c_str());
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
  CRAS_INFO_NAMED("renderer", "OpenGl version: %.1f (GLSL %.1f).", gl_version_ / 100.0, glsl_version_ / 100.0);
}

void RenderSystem::setContextCurrent()
{
  const auto gl_render_system = static_cast<Ogre::GLRenderSystem*>(ogre_root_->getRenderSystem());
  gl_render_system->_getMainContext()->setCurrent();
}

void RenderSystem::endContextCurrent()
{
  const auto gl_render_system = static_cast<Ogre::GLRenderSystem*>(ogre_root_->getRenderSystem());
  gl_render_system->_getMainContext()->endCurrent();
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
      while (pos2 != std::string::npos)
      {
        path = iter->substr(pos1, pos2 - pos1);
        CRAS_DEBUG_NAMED("renderer", "adding resource location: '%s'\n", path.c_str());
        Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
            path, "FileSystem", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        pos1 = pos2 + 1;
        pos2 = iter->find(delim, pos2 + 1);
      }
      path = iter->substr(pos1, iter->size() - pos1);
      CRAS_DEBUG_NAMED("renderer", "adding resource location: '%s'\n", path.c_str());
      Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
          path, "FileSystem", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    }
  }
}

static bool x_baddrawable_error = false;

Ogre::RenderWindow* RenderSystem::makeRenderWindow(
  WindowIDType window_id, unsigned int width, unsigned int height, double pixel_ratio)
{
  static int windowCounter = 0;  // Every RenderWindow needs a unique name, oy.

  Ogre::NameValuePairList params;
  Ogre::RenderWindow* window = nullptr;

  auto wid = window_id != 0 ? window_id : dummy_window_id_;
  if (wid != 0)
  {
    params["externalWindowHandle"] = Ogre::StringConverter::toString(wid);
    params["parentWindowHandle"] = Ogre::StringConverter::toString(wid);
    params["externalGLControl"] = "true";
  }
  else
  {
    params["hidden"] = "true";
  }

  // Enable antialiasing
  if (use_anti_aliasing_)
  {
    params["FSAA"] = "4";
  }

  // Set the macAPI for Ogre based on the Qt implementation
  params["contentScalingFactor"] = cras::to_string(pixel_ratio);

  std::ostringstream stream;
  stream << "OgreWindow(" << windowCounter++ << ")";

  window = tryMakeRenderWindow(stream.str(), width, height, &params, 100);

  if (window == nullptr)
  {
    CRAS_ERROR_NAMED("renderer", "Unable to create the rendering window after 100 tries.");
    assert(false);
  }

  if (window)
  {
    window->setActive(true);
    // window->setVisible(true);
    window->setAutoUpdated(false);
    if (window_id == 0 && dummy_window_id_ == 0)
      window->getCustomAttribute("WINDOW", &dummy_window_id_);
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
      CRAS_ERROR_NAMED("renderer", "rviz::RenderSystem: error creating render window: %s", ex.what());
      window = nullptr;
    }
  }

  if (window && attempts > 1)
  {
    CRAS_INFO_NAMED("renderer", "Created render window after %d attempts.", attempts);
  }

  return window;
}

}

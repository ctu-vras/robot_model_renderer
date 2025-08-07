// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: 2012 Willow Garage, Inc.

// This file is taken from rviz and minimally edited (just code style and different namespace).

#include <robot_model_renderer/ogre_helpers/ogre_logging.h>

#include <OgreLog.h>
#include <OgreLogManager.h>

#include <ros/console.h>

namespace robot_model_renderer
{

class RosLogListener : public Ogre::LogListener
{
public:
  RosLogListener() : min_lml(Ogre::LML_CRITICAL)
  {
  }

  ~RosLogListener() override = default;

  void messageLogged(const Ogre::String& message, Ogre::LogMessageLevel lml, bool /*maskDebug*/,
    const Ogre::String& /*logName*/, bool& skipThisMessage) override
  {
    if (!skipThisMessage)
    {
      if (lml >= min_lml)
      {
        ROS_LOG((ros::console::levels::Level)(lml - 1), ROSCONSOLE_DEFAULT_NAME, "%s", message.c_str());
      }
    }
  }
  Ogre::LogMessageLevel min_lml;
};

OgreLogging::Preference OgreLogging::preference_ = OgreLogging::NoLogging;
std::string OgreLogging::filename_;

void OgreLogging::useRosLog()
{
  preference_ = StandardOut;
}

void OgreLogging::useLogFile(const std::string& filename)
{
  preference_ = FileLogging;
  filename_ = filename;
}

void OgreLogging::noLog()
{
  preference_ = NoLogging;
}

void OgreLogging::configureLogging()
{
  static RosLogListener ll;
  Ogre::LogManager* log_manager = Ogre::LogManager::getSingletonPtr();
  if (log_manager == nullptr)
  {
    log_manager = new Ogre::LogManager();
  }
  Ogre::Log* l = log_manager->createLog(filename_, false, false, preference_ == NoLogging);
  l->addListener(&ll);

  // Printing to standard out is what Ogre does if you don't do any LogManager calls.
  if (preference_ == StandardOut)
  {
    ll.min_lml = Ogre::LML_NORMAL;
  }
}

}

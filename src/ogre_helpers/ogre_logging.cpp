// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: 2012 Willow Garage, Inc.

// This file is taken from rviz and minimally edited (just code style and different namespace).

#include <OgreException.h>
#include <robot_model_renderer/ogre_helpers/ogre_logging.h>

#include <OgreLog.h>
#include <OgreLogManager.h>

#include <cras_cpp_common/log_utils.h>
#include <ros/assert.h>
#include <ros/console.h>

namespace robot_model_renderer
{

ros::console::levels::Level ogreLogLevelToRosconsole(const Ogre::LogMessageLevel lml)
{
  switch (lml)
  {
    case Ogre::LML_TRIVIAL:
    case Ogre::LML_NORMAL:
      return ros::console::levels::Debug;
    case Ogre::LML_CRITICAL:
      return ros::console::levels::Error;
    default:
      ROS_ASSERT_MSG(false, "Unhandled enum case %i", lml);
      return ros::console::levels::Debug;
  }
}

RosLogListener::RosLogListener(const cras::LogHelperPtr& log) : cras::HasLogger(log)
{
}

RosLogListener::~RosLogListener() = default;

void RosLogListener::messageLogged(const Ogre::String& message, const Ogre::LogMessageLevel lml, bool /*maskDebug*/,
                                   const Ogre::String& /*logName*/, bool& skipThisMessage)
{
  if (skipThisMessage)
    return;

  const auto ros_level = ogreLogLevelToRosconsole(lml);
  CRAS_LOG(getCrasLogger(), ros_level, std::string(ROSCONSOLE_DEFAULT_NAME) + ".renderer", "%s", message.c_str());
}

Ogre::Log* OgreLogging::configureLogging(const std::string& filename, Ogre::LogListener* listener)
{
  // Create log manager
  Ogre::LogManager* log_manager = Ogre::LogManager::getSingletonPtr();
  if (log_manager == nullptr)
    log_manager = new Ogre::LogManager();

  // Create default file log
  try
  {
    log_manager->getLog(filename);
  }
  catch (const Ogre::InvalidParametersException&)
  {
    log_manager->createLog(filename, true, false, filename.empty());
  }

  // Create ROS log with listener if needed
  if (listener == nullptr)
    return nullptr;

  static int log_number = 0; log_number += 1;
  const auto log = Ogre::LogManager::getSingleton().createLog(
    ROSCONSOLE_PACKAGE_NAME + std::to_string(log_number), false, false, true);
  log->addListener(listener);
  return log;
}

}

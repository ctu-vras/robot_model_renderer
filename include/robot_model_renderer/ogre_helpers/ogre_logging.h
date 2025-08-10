// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: 2012 Willow Garage, Inc.

#pragma once

// This file is taken from rviz and minimally edited (just code style and different namespace).

#include <string>

#include <OgreLog.h>

#include <cras_cpp_common/log_utils.h>

namespace robot_model_renderer
{

ros::console::levels::Level ogreLogLevelToRosconsole(Ogre::LogMessageLevel lml);

class RosLogListener : public Ogre::LogListener, public cras::HasLogger
{
public:
  explicit RosLogListener(const cras::LogHelperPtr& log);
  ~RosLogListener() override;

  void messageLogged(const Ogre::String& message, Ogre::LogMessageLevel lml, bool maskDebug,
                     const Ogre::String& logName, bool& skipThisMessage) override;
};

/**
 * \brief Convenience interface to Ogre logging.
 */
class OgreLogging
{
public:
  /**
   * \brief Configure the default logger for Ogre::LogManager.
   *
   * \param[in] filename File to log to. If empty, no logging is done.
   * \param[in] listener Log listener for additional logs.
   * \return If listener is not null, an instance of a logger that only forwards logging calls to the listener.
   *
   * \note This must be called before Ogre::Root is instantiated!
   */
  static Ogre::Log* configureLogging(const std::string& filename = "Ogre.log", Ogre::LogListener* listener = nullptr);
};

}

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: 2012 Willow Garage, Inc.

#pragma once

// This file is taken from rviz and minimally edited (just code style and different namespace).

#include <string>

namespace robot_model_renderer
{

/**
 * \brief Convenience interface to Ogre logging.
 *
 * This all-static class wraps Ogre::LogManager into 3 easy options:
 * no logging, standard out, or file logging.  The option-selection
 * calls (useStandardOut(), useLogFile(), and noLog() must be called
 * before configureLogging().  configureLogging(), in turn, must be
 * called before any Ogre::Root object is instantiated.
 * configureLogging() is called at the right time by the RenderSystem
 * constructor, so you generally won't need to call it explicitly. */
class OgreLogging
{
public:
  /**
   * \brief Configure Ogre to write output to the ROS logger.
   */
  static void useRosLog();

  /**
   * \brief Configure Ogre to write output to the given log file name.
   *
   * \param[in] filename The log filename (default is Ogre.log). If file name is a relative path, it will be relative to
   *                     the directory which is current when the program is run.
   */
  static void useLogFile(const std::string& filename = "Ogre.log");

  /**
   * \brief Disable Ogre logging entirely.  This is the default.
   */
  static void noLog();

  /**
   * \brief Configure the Ogre::LogManager to give the currently selected behavior.
   *
   * \note This must be called before Ogre::Root is instantiated!
   */
  static void configureLogging();

private:
  typedef enum
  {
    StandardOut,
    FileLogging,
    NoLogging
  } Preference;

  static Preference preference_;  //!< The logging preference.
  static std::string filename_;  //!< The log filename.
};

}

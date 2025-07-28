/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

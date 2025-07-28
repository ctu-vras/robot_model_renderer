/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

// This file is taken from rviz and slightly edited to be usable in this package.

#include <robot_model_renderer/robot/tf_link_updater.h>

#include <string>

#include <OgreQuaternion.h>

#include <robot_model_renderer/ogre_helpers/ogre_vector.h>
#include <rosconsole/macros_generated.h>
#include <tf2_ros/buffer.h>

namespace robot_model_renderer
{

std::string concat(const std::string& prefix, const std::string& frame)
{
  if (prefix.empty())
    return frame;

  std::string composite = prefix;
  composite.append("/");
  composite.append(frame);
  return composite;
}

TFLinkUpdater::TFLinkUpdater(tf2_ros::Buffer* tf, const std::string& fixed_frame, const std::string& tf_prefix)
  : tf_(tf), fixed_frame_(fixed_frame), tf_prefix_(tf_prefix)
{
}

void TFLinkUpdater::setFixedFrame(const std::string& fixedFrame)
{
  this->fixed_frame_ = fixedFrame;
}

bool TFLinkUpdater::getLinkTransforms(const ros::Time& time, const std::string& link_name,
  Ogre::Vector3& visual_position, Ogre::Quaternion& visual_orientation,
  Ogre::Vector3& collision_position, Ogre::Quaternion& collision_orientation) const
{
  if (fixed_frame_.empty())
  {
    ROS_ERROR("Fixed frame has not been set.");
    return false;
  }

  const auto link_name_prefixed = concat(tf_prefix_, link_name);

  if (!tf_->canTransform(fixed_frame_, link_name_prefixed, time, ros::Duration(0.01)))
  {
    ROS_WARN_STREAM("No transform from [" << link_name_prefixed << "] to [" << fixed_frame_ << "]");
    return false;
  }

  const auto tf = tf_->lookupTransform(fixed_frame_, link_name_prefixed, time);

  const Ogre::Vector3 position(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
  const Ogre::Quaternion orientation(
    tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z);

  // Collision/visual transforms are the same in this case
  visual_position = position;
  visual_orientation = orientation;
  collision_position = position;
  collision_orientation = orientation;

  return true;
}

}

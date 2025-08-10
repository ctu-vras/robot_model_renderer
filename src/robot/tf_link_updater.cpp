// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: 2009 Willow Garage, Inc.

// This file is taken from rviz and slightly edited to be usable in this package.

#include <robot_model_renderer/robot/tf_link_updater.h>

#include <string>

#include <OgreQuaternion.h>

#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/tf2_utils/interruptible_buffer.h>
#include <robot_model_renderer/ogre_helpers/ogre_vector.h>
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

TFLinkUpdater::TFLinkUpdater(const cras::LogHelperPtr& log, const std::shared_ptr<cras::InterruptibleTFBuffer>& tf,
  const std::string& fixed_frame, const std::string& tf_prefix, const ros::Duration& timeout)
  : cras::HasLogger(log), tf_(tf), fixed_frame_(fixed_frame), tf_prefix_(tf_prefix), timeout_(timeout)
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
    CRAS_ERROR_NAMED("link_updater", "Fixed frame has not been set.");
    return false;
  }

  const auto link_name_prefixed = concat(tf_prefix_, link_name);

  if (!tf_->canTransform(fixed_frame_, link_name_prefixed, time, timeout_))
  {
    CRAS_WARN_STREAM_THROTTLE_NAMED(1.0, "link_updater",
      "No transform from [" << link_name_prefixed << "] to [" << fixed_frame_ << "]");
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

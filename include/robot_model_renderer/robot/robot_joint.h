/*
 * Copyright (c) 2013, Willow Garage, Inc.
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

#pragma once

#include <string>
#include <map>


#include <robot_model_renderer/ogre_helpers/ogre_vector.h>
#include <OgreQuaternion.h>
#include <OgreAny.h>
#include <OgreMaterial.h>

#include <urdf/model.h>
#include <urdf_model/pose.h>

#include <robot_model_renderer/ogre_helpers/object.h>

#include <OgrePrerequisites.h>

namespace robot_model_renderer
{
class Shape;
class Robot;
class RobotJoint;


/**
 * \struct RobotJoint
 * \brief Contains any data we need from a joint in the robot.
 */
class RobotJoint
{
public:
  RobotJoint(Robot* robot, const urdf::JointConstSharedPtr& joint);
  virtual ~RobotJoint();


  void setTransforms(const Ogre::Vector3& parent_link_position, const Ogre::Quaternion& parent_link_orientation);

  const std::string& getName() const
  {
    return name_;
  }
  const std::string& getParentLinkName() const
  {
    return parent_link_name_;
  }
  const std::string& getChildLinkName() const
  {
    return child_link_name_;
  }
  RobotJoint* getParentJoint();

  Ogre::Vector3 getPosition();
  Ogre::Quaternion getOrientation();

  void setRobotAlpha(float /*unused*/)
  {
  }

  bool hasDescendentLinksWithGeometry() const
  {
    return has_decendent_links_with_geometry_;
  }

private:
  void updateChildVisibility();

private:
  bool getEnabled() const;

  // determine the state of child link(s)
  void
  getChildLinkState(int& links_with_geom,           // returns # of children with geometry
                    int& links_with_geom_checked,   // returns # of enabled children with geometry
                    int& links_with_geom_unchecked, // returns # of disabled children with geometry
                    bool recursive) const; // True: all descendant links.  False: just single child link.


protected:
  Robot* robot_;
  std::string name_; ///< Name of this joint
  std::string parent_link_name_;
  std::string child_link_name_;

private:
  Ogre::Vector3 joint_origin_pos_;
  Ogre::Quaternion joint_origin_rot_;
  bool has_decendent_links_with_geometry_;

  Ogre::Vector3 position_;
  Ogre::Quaternion orientation_;
};

} // namespace robot_model_renderer

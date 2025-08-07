// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: 2008 Willow Garage, Inc.

// This file is taken from rviz and minimally edited (just code style and different namespace).

#include <robot_model_renderer/ogre_helpers/camera_base.h>

#include <cstdint>
#include <sstream>

#include <OgreCamera.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreQuaternion.h>

#include <robot_model_renderer/ogre_helpers/ogre_vector.h>

namespace robot_model_renderer
{

CameraBase::CameraBase(Ogre::SceneManager* scene_manager) : scene_manager_(scene_manager), relative_node_(nullptr)
{
  std::stringstream ss;
  static uint32_t count = 0;
  ss << "CameraBase" << count++;
  camera_ = scene_manager_->createCamera(ss.str());
}

CameraBase::~CameraBase()
{
  scene_manager_->destroyCamera(camera_);
}

void CameraBase::setRelativeNode(Ogre::SceneNode* node)
{
  relative_node_ = node;

  relativeNodeChanged();

  update();
}

void CameraBase::setPosition(const Ogre::Vector3& position)
{
  setPosition(position.x, position.y, position.z);
}

void CameraBase::setOrientation(const Ogre::Quaternion& orientation)
{
  setOrientation(orientation.x, orientation.y, orientation.z, orientation.w);
}

}

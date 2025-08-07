// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: 2008 Willow Garage, Inc.

#pragma once

// This file is taken from rviz and minimally edited (just code style and different namespace).

#include <OgrePrerequisites.h>

namespace Ogre
{

class Any;

}

namespace robot_model_renderer
{

/**
 * \brief Base class for visible objects, providing a minimal generic interface.
 */
class Object
{
public:

  /**
   * \param[in] scene_manager The scene manager this object should be part of.
   */
  explicit Object(Ogre::SceneManager* scene_manager);

  virtual ~Object();

  /**
   * \brief Set the position of this object
   *
   * \param[in] position The position to be set.
   */
  virtual void setPosition(const Ogre::Vector3& position) = 0;

  /**
   * \brief Set the orientation of the object
   *
   * \param[in] orientation The quaternion orientation to be set.
   */
  virtual void setOrientation(const Ogre::Quaternion& orientation) = 0;

  /**
   * \brief Set the scale of the object. Always relative to the identity orientation of the object.
   *
   * \param[in] scale The scale vector to be set.
   */
  virtual void setScale(const Ogre::Vector3& scale) = 0;

  /**
   * \brief Set the color of the object.  Values are in the range [0, 1]
   *
   * \param[in] r Red component
   * \param[in] g Green component
   * \param[in] b Blue component
   * \param[in] a Alpha component
   */
  virtual void setColor(float r, float g, float b, float a) = 0;

  /**
   * \brief Get the local position of this object
   *
   * \return The position
   */
  virtual const Ogre::Vector3& getPosition() = 0;

  /**
   * \brief Get the local orientation of this object
   *
   * \return The orientation
   */
  virtual const Ogre::Quaternion& getOrientation() = 0;

  /**
   * \brief Set the user data on this object
   *
   * \param[in] data The user data.
   */
  virtual void setUserData(const Ogre::Any& data) = 0;

protected:
  Ogre::SceneManager* scene_manager_; //!< Ogre scene manager this object is part of
};

}

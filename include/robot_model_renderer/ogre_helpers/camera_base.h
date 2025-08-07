// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: 2008 Willow Garage, Inc.

#pragma once

// This file is taken from rviz and minimally edited (just code style and different namespace).

#include <OgreQuaternion.h>

#include <robot_model_renderer/ogre_helpers/ogre_vector.h>

namespace Ogre
{

class Camera;
class SceneNode;
class SceneManager;

}

namespace robot_model_renderer
{
/**
 * \brief Generic interface for a camera
 *
 * Provides an interface that a camera can override, providing interchangeability between different camera
 * types.
 * Specific implementation is left to the child class.
 */
class CameraBase
{
public:
  /**
   * \param[in] scene_manager Scene manager this camera is displaying
   */
  explicit CameraBase(Ogre::SceneManager* scene_manager);
  virtual ~CameraBase();

  /**
   * \brief Get the Ogre camera associated with this camera object
   *
   * \return The Ogre camera associated with this camera object
   */
  Ogre::Camera* getOgreCamera() const
  {
    return camera_;
  }

  /**
   * \brief Set a scene node that all camera transformations should be relative to
   *
   * \param[in] node The node
   */
  void setRelativeNode(Ogre::SceneNode* node);

  /**
   * \brief Called when the relative node changes
   */
  virtual void relativeNodeChanged()
  {
  }

  virtual void update() = 0;

  /**
   * \brief Set the position of the camera
   *
   * \param[in] position The position
   */
  virtual void setPosition(const Ogre::Vector3& position);

  /**
   * \brief Set the orientation of the camera
   *
   * \param[in] orientation The orientation
   */
  virtual void setOrientation(const Ogre::Quaternion& orientation);

  /**
   * \brief Yaw the camera.
   *
   * Calls to yaw are cumulative, so:
   *   yaw(PI);
   *   yaw(PI);
   *
   * is equivalent to
   *  yaw(2*PI);
   *
   * \param[in] angle Angle to yaw, in radians
   */
  virtual void yaw(float angle) = 0;

  /**
   * \brief Pitch the camera.
   *
   * Calls to pitch are cumulative, so:
   *   pitch(PI);
   *   pitch(PI);
   *
   * is equivalent to
   *  pitch(2*PI);
   *
   * \param[in] angle Angle to pitch, in radians
   */

  virtual void pitch(float angle) = 0;

  /**
   * \brief Roll the camera.
   *
   * Calls to roll are cumulative, so:
   *   roll(PI);
   *   roll(PI);
   *
   * is equivalent to
   *  roll(2*PI);
   *
   * \param[in] angle Angle to roll, in radians
   */
  virtual void roll(float angle) = 0;

  /**
   * \brief Set the orientation of the camera from a quaternion
   *
   * \param[in] x Orientation quaternion x component.
   * \param[in] y Orientation quaternion y component.
   * \param[in] z Orientation quaternion z component.
   * \param[in] w Orientation quaternion w component.
   */
  virtual void setOrientation(float x, float y, float z, float w) = 0;

  /**
   * \brief Set the position of the camera
   *
   * \param[in] x Position x component.
   * \param[in] y Position y component.
   * \param[in] z Position z component.
   */
  virtual void setPosition(float x, float y, float z) = 0;

  /**
   * \brief Set the position/orientation of this camera from another camera.
   *
   * \param[in] camera The camera to set from
   */
  virtual void setFrom(CameraBase* camera) = 0;

  /**
   * \brief Get the position of this camera
   *
   * \return The position of this camera
   */

  virtual Ogre::Vector3 getPosition() = 0;

  /**
   * \brief Get the orientation of this camera
   *
   * \return The orientation of this camera
   */
  virtual Ogre::Quaternion getOrientation() = 0;

  /**
   * \brief Point the camera at the specified point
   *
   * \param[in] point The point to look at
   */
  virtual void lookAt(const Ogre::Vector3& point) = 0;

  /**
   * \brief Move the camera relative to its orientation
   *
   * \param[in] x Distance to move along the X-axis
   * \param[in] y Distance to move along the Y-axis
   * \param[in] z Distance to move along the Z-axis
   */
  virtual void move(float x, float y, float z) = 0;

  virtual void mouseLeftDown(int /*x*/, int /*y*/)
  {
  }

  virtual void mouseMiddleDown(int /*x*/, int /*y*/)
  {
  }

  virtual void mouseRightDown(int /*x*/, int /*y*/)
  {
  }

  virtual void mouseLeftUp(int /*x*/, int /*y*/)
  {
  }

  virtual void mouseMiddleUp(int /*x*/, int /*y*/)
  {
  }

  virtual void mouseRightUp(int /*x*/, int /*y*/)
  {
  }

  /**
   * \brief Handle a left mouse button drag
   *
   * \param[in] diff_x Pixels the mouse has moved in the (window space) x direction
   * \param[in] diff_y Pixels the mouse has moved in the (window space) y direction
   * \param[in] ctrl Whether the Ctrl key is pressed
   * \param[in] alt Whether the Alt key is pressed
   * \param[in] shift Whether the Shift key is pressed
   */
  virtual void mouseLeftDrag(int diff_x, int diff_y, bool ctrl, bool alt, bool shift) = 0;

  /**
   * \brief Handle a middle mouse button drag
   *
   * \param[in] diff_x Pixels the mouse has moved in the (window space) x direction
   * \param[in] diff_y Pixels the mouse has moved in the (window space) y direction
   * \param[in] ctrl Whether the Ctrl key is pressed
   * \param[in] alt Whether the Alt key is pressed
   * \param[in] shift Whether the Shift key is pressed
   */
  virtual void mouseMiddleDrag(int diff_x, int diff_y, bool ctrl, bool alt, bool shift) = 0;

  /**
   * \brief Handle a right mouse button drag
   *
   * \param[in] diff_x Pixels the mouse has moved in the (window space) x direction
   * \param[in] diff_y Pixels the mouse has moved in the (window space) y direction
   * \param[in] ctrl Whether the Ctrl key is pressed
   * \param[in] alt Whether the Alt key is pressed
   * \param[in] shift Whether the Shift key is pressed
   */
  virtual void mouseRightDrag(int diff_x, int diff_y, bool ctrl, bool alt, bool shift) = 0;

  /**
   * \brief Handle a scrollwheel change
   *
   * \param[in] diff Number of "units" the scrollwheel has moved
   * \param[in] ctrl Whether the Ctrl key is pressed
   * \param[in] alt Whether the Alt key is pressed
   * \param[in] shift Whether the Shift key is pressed
   *
   * \todo Probably need to pass in how many units there are in a "click" of the wheel
   */
  virtual void scrollWheel(int diff, bool ctrl, bool alt, bool shift) = 0;

  /**
   * \brief Loads the camera's configuration from the supplied string (generated through toString())
   * \param[in] str The string to load from
   */
  virtual void fromString(const std::string& str) = 0;

  /**
   * \brief Returns a string representation of the camera's configuration
   */
  virtual std::string toString() = 0;

protected:
  Ogre::Camera* camera_;  //!< Ogre camera associated with this camera object
  Ogre::SceneManager* scene_manager_;  //!< Scene manager this camera is part of

  Ogre::SceneNode* relative_node_;  //!< The node relative to which the camera is positioned
};

}

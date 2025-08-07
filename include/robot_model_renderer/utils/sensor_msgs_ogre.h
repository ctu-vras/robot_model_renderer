// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

#include <string>

#include <OgrePixelFormat.h>

namespace robot_model_renderer
{
/**
 * \brief Convert an sensor_msgs/Image encoding to the most suitable OGRE pixel format.
 * \param[in] encoding The sensor_msgs encoding from image_encodings.h .
 * \return The corresponding OGRE pixel format.
 * \throws std::runtime_error If there is no conversion.
 */
Ogre::PixelFormat sensorMsgsEncodingToOgrePixelFormat(const std::string& encoding);
}
// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

#include <sensor_msgs/CameraInfo.h>

namespace robot_model_renderer
{

bool areCameraInfosAlmostEqual(const sensor_msgs::CameraInfo& c1, const sensor_msgs::CameraInfo& c2, double eps = 1e-6);

}

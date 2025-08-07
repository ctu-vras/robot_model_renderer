// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <robot_model_renderer/utils/sensor_msgs.h>

#include <algorithm>
#include <cmath>

#include <sensor_msgs/CameraInfo.h>

namespace robot_model_renderer
{

bool areCameraInfosAlmostEqual(const sensor_msgs::CameraInfo& c1, const sensor_msgs::CameraInfo& c2, const double eps)
{
  if (c1.width != c2.width || c1.height != c2.height)
    return false;

  if (c1.header.frame_id != c2.header.frame_id)
    return false;

  if (c1.roi.x_offset != c2.roi.x_offset || c1.roi.y_offset != c2.roi.y_offset)
    return false;

  if (c1.roi.do_rectify != c2.roi.do_rectify)
    return false;

  if (c1.distortion_model != c2.distortion_model)
    return false;

  // Binning 0 and 1 are equivalent

  if (c1.binning_x != 0 && c2.binning_x != 0 && c1.binning_x != c2.binning_x)
    return false;
  if (c1.binning_x == 0 && c2.binning_x != 0 && c2.binning_x != 1)
    return false;
  if (c1.binning_x != 0 && c2.binning_x == 0 && c1.binning_x != 1)
    return false;

  if (c1.binning_y != 0 && c2.binning_y != 0 && c1.binning_y != c2.binning_y)
    return false;
  if (c1.binning_y == 0 && c2.binning_y != 0 && c2.binning_y != 1)
    return false;
  if (c1.binning_y != 0 && c2.binning_y == 0 && c1.binning_y != 1)
    return false;

  // ROI width 0 and image width are equivalent

  if (c1.roi.width != 0 && c2.roi.width != 0 && c1.roi.width != c2.roi.width)
    return false;
  if (c1.roi.width == 0 && c2.roi.width != 0 && c2.roi.width != c2.width)
    return false;
  if (c1.roi.width != 0 && c2.roi.width == 0 && c1.roi.width != c1.width)
    return false;

  if (c1.roi.height != 0 && c2.roi.height != 0 && c1.roi.height != c2.roi.height)
    return false;
  if (c1.roi.height == 0 && c2.roi.height != 0 && c2.roi.height != c2.height)
    return false;
  if (c1.roi.height != 0 && c2.roi.height == 0 && c1.roi.height != c1.height)
    return false;

  const auto almostEqual = [eps](const double d1, const double d2)
  {
    return std::abs(d1 - d2) <= eps;
  };

  if (!std::equal(c1.D.cbegin(), c1.D.cend(), c2.D.cbegin(), almostEqual))
    return false;

  if (!std::equal(c1.K.cbegin(), c1.K.cend(), c2.K.cbegin(), almostEqual))
    return false;

  if (!std::equal(c1.R.cbegin(), c1.R.cend(), c2.R.cbegin(), almostEqual))
    return false;

  if (!std::equal(c1.P.cbegin(), c1.P.cend(), c2.P.cbegin(), almostEqual))
    return false;

  return true;
}

}

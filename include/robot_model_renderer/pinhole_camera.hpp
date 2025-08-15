// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

/**
 * \file
 * \brief Extension of image_geometry PinholeCameraModel that allows better support for the distortion shader.
 * \author Martin Pecka
 */

#include <memory>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>

namespace robot_model_renderer
{

/**
 * \brief Extension of image_geometry PinholeCameraModel that allows better support for the distortion shader.
 */
class PinholeCameraModel : public image_geometry::PinholeCameraModel
{
  struct ExtraCache;
  std::unique_ptr<ExtraCache> extraCache;

public:
  PinholeCameraModel();
  explicit PinholeCameraModel(const sensor_msgs::CameraInfo& msg);

  PinholeCameraModel(const PinholeCameraModel& other);
  PinholeCameraModel& operator=(const PinholeCameraModel& other);

  virtual ~PinholeCameraModel();
  virtual void initUnrectificationMaps() const;
  virtual cv::Rect rectifyRoi(const cv::Rect& roi_raw, const cv::Mat& P) const;
  cv::Rect rectifyRoi(const cv::Rect& roi_raw) const;
  virtual cv::Rect unrectifyRoi(const cv::Rect& roi_rect) const;
  virtual void unrectifyImage(const cv::Mat& rectified, cv::Mat& raw, int interpolation) const;
  void unrectifyImage(const cv::Mat& rectified, cv::Mat& raw) const;
  virtual cv::Size getRectifiedResolution() const;
  virtual PinholeCameraModel getModelForResolution(const cv::Size& res) const;
  virtual PinholeCameraModel getScaled(double scale) const;

  virtual const cv::Mat& getReducedUnrectifyFloatMap() const;
  virtual cv::Mat getUnrectifyFloatMap(const cv::InputArray& newCameraMatrix, const cv::Size& size) const;

  // Shim for missing 3-arg rectifyPoint on Melodic. For Noetic, this is just fast-forward.
  virtual cv::Point2d _rectifyPoint(const cv::Point2d& uv_raw, const cv::Matx33d& K, const cv::Matx34d& P) const;
};

}

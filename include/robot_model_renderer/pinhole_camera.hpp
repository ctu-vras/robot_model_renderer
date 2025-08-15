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
  virtual cv::Rect rectifyRoi(const cv::Rect& roi_raw, const cv::Mat& P = cv::Mat()) const;
  virtual cv::Rect unrectifyRoi(const cv::Rect& roi_rect) const;
  virtual void unrectifyImage(const cv::Mat& rectified, cv::Mat& raw, int interpolation = cv::INTER_LINEAR) const;
  virtual cv::Size getRectifiedResolution() const;
  virtual PinholeCameraModel getModelForResolution(const cv::Size& res) const;

  virtual const cv::Mat& getReducedUnrectifyFloatMap() const;
  virtual cv::Mat getUnrectifyFloatMap(const cv::InputArray& newCameraMatrix, const cv::Size& size) const;
};

}

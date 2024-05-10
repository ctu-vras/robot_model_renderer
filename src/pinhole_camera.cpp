#include <opencv2/calib3d.hpp>
#include <robot_model_renderer/pinhole_camera.h>

namespace image_geometry
{
enum DistortionState { NONE, CALIBRATED, UNKNOWN };
enum DistortionModel { EQUIDISTANT, PLUMB_BOB_OR_RATIONAL_POLYNOMIAL, UNKNOWN_MODEL };

struct PinholeCameraModel::Cache
{
  DistortionState distortion_state;
  DistortionModel distortion_model;

  cv::Mat_<double> K_binned, P_binned; // Binning applied, but not cropping

  mutable bool full_maps_dirty;
  mutable cv::Mat full_map1, full_map2;

  mutable bool reduced_maps_dirty;
  mutable cv::Mat reduced_map1, reduced_map2;

  mutable bool unrectify_full_maps_dirty;
  mutable cv::Mat unrectify_full_map1, unrectify_full_map2;

  mutable bool unrectify_reduced_maps_dirty;
  mutable cv::Mat unrectify_reduced_map1, unrectify_reduced_map2;

  mutable bool rectified_roi_dirty;
  mutable cv::Rect rectified_roi;

  Cache()
    : distortion_state(UNKNOWN),
      distortion_model(UNKNOWN_MODEL),
      full_maps_dirty(true),
      reduced_maps_dirty(true),
      unrectify_full_maps_dirty(true),
      unrectify_reduced_maps_dirty(true),
      rectified_roi_dirty(true)
  {
  }
};
}

namespace robot_model_renderer
{

struct PinholeCameraModel::ExtraCache
{
  mutable bool unrectify_full_float_maps_dirty {true};
  mutable cv::Mat unrectify_full_float_map;

  mutable bool unrectify_reduced_float_maps_dirty {true};
  mutable cv::Mat unrectify_reduced_float_map;
};

void initInverseRectificationMap(cv::InputArray _cameraMatrix, cv::InputArray _distCoeffs,
                              cv::InputArray _matR, cv::InputArray _newCameraMatrix,
                              const cv::Size& size, int m1type, cv::OutputArray _map1, cv::OutputArray _map2 )
{
    // Parameters
    cv::Mat cameraMatrix = _cameraMatrix.getMat(), distCoeffs = _distCoeffs.getMat();
    cv::Mat matR = _matR.getMat(), newCameraMatrix = _newCameraMatrix.getMat();

    // Check m1type validity
    if( m1type <= 0 )
        m1type = CV_16SC2;
    CV_Assert( m1type == CV_16SC2 || m1type == CV_32FC1 || m1type == CV_32FC2 );

    // Init Maps
    _map1.create( size, m1type );
    cv::Mat map1 = _map1.getMat(), map2;
    if( m1type != CV_32FC2 )
    {
        _map2.create( size, m1type == CV_16SC2 ? CV_16UC1 : CV_32FC1 );
        map2 = _map2.getMat();
    }
    else {
        _map2.release();
    }

    // Init camera intrinsics
    cv::Mat_<double> A = cv::Mat_<double>(cameraMatrix), Ar;
    if( !newCameraMatrix.empty() )
        Ar =cv:: Mat_<double>(newCameraMatrix);
    else
        Ar = cv::getDefaultNewCameraMatrix( A, size, true );
    CV_Assert( A.size() == cv::Size(3,3) );
    CV_Assert( Ar.size() == cv::Size(3,3) || Ar.size() == cv::Size(4, 3));

    // Init rotation matrix
    cv::Mat_<double> R = cv::Mat_<double>::eye(3, 3);
    if( !matR.empty() )
    {
        R = cv::Mat_<double>(matR);
        //Note, do not inverse
    }
    CV_Assert( cv::Size(3,3) == R.size() );

    // Init distortion vector
    if( !distCoeffs.empty() ){
        distCoeffs = cv::Mat_<double>(distCoeffs);

        // Fix distortion vector orientation
        if( distCoeffs.rows != 1 && !distCoeffs.isContinuous() ) {
            distCoeffs = distCoeffs.t();
        }
    }

    // Validate distortion vector size
    CV_Assert(  distCoeffs.empty() || // Empty allows cv::undistortPoints to skip distortion
                distCoeffs.size() == cv::Size(1, 4) || distCoeffs.size() == cv::Size(4, 1) ||
                distCoeffs.size() == cv::Size(1, 5) || distCoeffs.size() == cv::Size(5, 1) ||
                distCoeffs.size() == cv::Size(1, 8) || distCoeffs.size() == cv::Size(8, 1) ||
                distCoeffs.size() == cv::Size(1, 12) || distCoeffs.size() == cv::Size(12, 1) ||
                distCoeffs.size() == cv::Size(1, 14) || distCoeffs.size() == cv::Size(14, 1));

    // Create objectPoints
    std::vector<cv::Point2i> p2i_objPoints;
    std::vector<cv::Point2f> p2f_objPoints;
    p2i_objPoints.reserve(size.width * size.height);
    p2f_objPoints.reserve(size.width * size.height);
    for (int r = 0; r < size.height; r++)
    {
        for (int c = 0; c < size.width; c++)
        {
            p2i_objPoints.emplace_back(c, r);
            p2f_objPoints.emplace_back(static_cast<float>(c), static_cast<float>(r));
        }
    }

    // Undistort
    std::vector<cv::Point2f> p2f_objPoints_undistorted;
    cv::undistortPoints(
        p2f_objPoints,
        p2f_objPoints_undistorted,
        A,
        distCoeffs,
        cv::Mat::eye(cv::Size(3, 3), CV_32FC1), // R
        cv::Mat::eye(cv::Size(3, 3), CV_32FC1) // P = New K
    );

    // Rectify
    std::vector<cv::Point2f> p2f_sourcePoints_pinHole;
    cv::perspectiveTransform(
        p2f_objPoints_undistorted,
        p2f_sourcePoints_pinHole,
        R
    );

    // Project points back to camera coordinates.
    std::vector<cv::Point2f> p2f_sourcePoints;
    cv::undistortPoints(
        p2f_sourcePoints_pinHole,
        p2f_sourcePoints,
        cv::Mat::eye(cv::Size(3, 3), CV_32FC1), // K
        cv::Mat::zeros(cv::Size(1, 4), CV_32FC1), // Distortion
        cv::Mat::eye(cv::Size(3, 3), CV_32FC1), // R
        Ar // New K
    );

    // Copy to map
    if (m1type == CV_16SC2) {
        for (size_t i=0; i < p2i_objPoints.size(); i++) {
            map1.at<cv::Vec2s>(p2i_objPoints[i].y, p2i_objPoints[i].x) = cv::Vec2s(cv::saturate_cast<short>(p2f_sourcePoints[i].x), cv::saturate_cast<short>(p2f_sourcePoints[i].y));
        }
    } else if (m1type == CV_32FC2) {
        for (size_t i=0; i < p2i_objPoints.size(); i++) {
            map1.at<cv::Vec2f>(p2i_objPoints[i].y, p2i_objPoints[i].x) = cv::Vec2f(p2f_sourcePoints[i]);
        }
    } else { // m1type == CV_32FC1
        for (size_t i=0; i < p2i_objPoints.size(); i++) {
            map1.at<float>(p2i_objPoints[i].y, p2i_objPoints[i].x) = p2f_sourcePoints[i].x;
            map2.at<float>(p2i_objPoints[i].y, p2i_objPoints[i].x) = p2f_sourcePoints[i].y;
        }
    }
}

PinholeCameraModel::PinholeCameraModel() : extraCache(std::make_unique<ExtraCache>())
{
}

PinholeCameraModel::PinholeCameraModel(const sensor_msgs::CameraInfo& msg)
{
  this->fromCameraInfo(msg);
}

PinholeCameraModel::~PinholeCameraModel() = default;


PinholeCameraModel::PinholeCameraModel(const PinholeCameraModel& other)
  : image_geometry::PinholeCameraModel(other)
{
}

PinholeCameraModel& PinholeCameraModel::operator=(const PinholeCameraModel& other)
{
  if (this == &other)
    return *this;
  image_geometry::PinholeCameraModel::operator=(other);
  return *this;
}

cv::Mat PinholeCameraModel::getUnrectifyFloatMap(const cv::InputArray& newCameraMatrix, const cv::Size& size) const
{
  // Create the full-size map at the binned resolution
  cv::Size binned_resolution = fullResolution();
  binned_resolution.width /= binningX();
  binned_resolution.height /= binningY();

  cv::Matx33d K_binned;
  if (binningX() == 1 && binningY() == 1)
  {
    K_binned = K_full_;
  }
  else
  {
    K_binned = K_full_;
    if (binningX() > 1)
    {
      double scale_x = 1.0 / binningX();
      K_binned(0, 0) *= scale_x;
      K_binned(0, 2) *= scale_x;
    }
    if (binningY() > 1)
    {
      double scale_y = 1.0 / binningY();
      K_binned(1, 1) *= scale_y;
      K_binned(1, 2) *= scale_y;
    }
  }

  // If a larger matrix is requested, we use its cx, cy to center the resulting image in the larger frame
  // const auto& newMat = newCameraMatrix.getMat();
  // if (!newMat.empty() && !size.empty())
  // {
  //   cv::Matx33d newMat3(newMat);
  //   K_binned(0, 2) = newMat3(0, 2);
  //   K_binned(1, 2) = newMat3(1, 2);
  // }

  cv::Mat map;
  cv::Mat map2;
  initInverseRectificationMap(K_binned, D_, R_, newCameraMatrix, size, CV_32FC2, map, map2);

  return map;
}

void PinholeCameraModel::initUnrectificationMaps() const
{
  if (extraCache->unrectify_full_float_maps_dirty)
  {
    // Create the full-size map at the binned resolution
    cv::Size binned_resolution = fullResolution();
    binned_resolution.width /= binningX();
    binned_resolution.height /= binningY();

    cv::Matx34d P_binned;
    if (binningX() == 1 && binningY() == 1)
    {
      P_binned = P_full_;
    }
    else
    {
      P_binned = P_full_;
      if (binningX() > 1)
      {
        double scale_x = 1.0 / binningX();
        P_binned(0, 0) *= scale_x;
        P_binned(0, 2) *= scale_x;
        P_binned(0, 3) *= scale_x;
      }
      if (binningY() > 1)
      {
        double scale_y = 1.0 / binningY();
        P_binned(1, 1) *= scale_y;
        P_binned(1, 2) *= scale_y;
        P_binned(1, 3) *= scale_y;
      }
    }

    extraCache->unrectify_full_float_map = this->getUnrectifyFloatMap(P_binned, binned_resolution);

    // if (extraCache->unrectify_full_float_map.size() != binned_resolution)
    //   extraCache->unrectify_full_float_map = cv::Mat(binned_resolution.height, binned_resolution.width, CV_32FC2);
    // for (size_t x = 0; x < binned_resolution.width; x++)
    // {
    //   for (size_t y = 0; y < binned_resolution.height; y++)
    //   {
    //     cv::Point2f uv_raw(x, y), uv_rect;
    //     uv_rect = rectifyPoint(uv_raw, K_binned, P_binned);
    //     extraCache->unrectify_full_float_map.at<cv::Point2f>(y, x) = uv_rect;
    //   }
    // }
    extraCache->unrectify_full_float_maps_dirty = false;
  }

  if (cache_->unrectify_full_maps_dirty)
  {
    // Note: m1type=CV_16SC2 to use fast fixed-point maps (see cv::remap)
    convertMaps(extraCache->unrectify_full_float_map,
                cv::Mat(),
                cache_->unrectify_full_map1,
                cache_->unrectify_full_map2,
                CV_16SC2);
    cache_->unrectify_full_maps_dirty = false;
  }

  if (cache_->unrectify_reduced_maps_dirty || extraCache->unrectify_reduced_float_maps_dirty)
  {
    /// @todo Use rectified ROI
    cv::Rect roi(cam_info_.roi.x_offset,
                 cam_info_.roi.y_offset,
                 cam_info_.roi.width,
                 cam_info_.roi.height);
    if (roi.x != 0 || roi.y != 0 ||
      (roi.height != 0 && roi.height != (int)cam_info_.height) ||
      (roi.width != 0 && roi.width != (int)cam_info_.width))
    {
      // map1 contains integer (x,y) offsets, which we adjust by the ROI offset
      // map2 contains LUT index for subpixel interpolation, which we can leave as-is
      roi.x /= binningX();
      roi.y /= binningY();
      roi.width /= binningX();
      roi.height /= binningY();
      if (cache_->unrectify_reduced_maps_dirty)
      {
        cache_->unrectify_reduced_map1 = cache_->unrectify_full_map1(roi) -
          cv::Scalar(roi.x, roi.y);
        cache_->unrectify_reduced_map2 = cache_->unrectify_full_map2(roi);
      }
      if (extraCache->unrectify_reduced_float_maps_dirty)
      {
        extraCache->unrectify_reduced_float_map = extraCache->unrectify_full_float_map - cv::Scalar(roi.x, roi.y);
      }
    }
    else
    {
      // Otherwise we're rectifying the full image
      if (cache_->unrectify_reduced_maps_dirty)
      {
        cache_->unrectify_reduced_map1 = cache_->unrectify_full_map1;
        cache_->unrectify_reduced_map2 = cache_->unrectify_full_map2;
      }
      if (extraCache->unrectify_reduced_float_maps_dirty)
      {
        extraCache->unrectify_reduced_float_map = extraCache->unrectify_full_float_map;
      }
    }
    cache_->unrectify_reduced_maps_dirty = false;
    extraCache->unrectify_reduced_float_maps_dirty = false;
  }
}

cv::Rect PinholeCameraModel::rectifyRoi(const cv::Rect& roi_raw) const
{
  assert( initialized() );

  double oX0 = std::numeric_limits<double>::infinity();
  double oX1 = -std::numeric_limits<double>::infinity();
  double oY0 = std::numeric_limits<double>::infinity();
  double oY1 = -std::numeric_limits<double>::infinity();

  const int N = 5;
  for (size_t dy = 0; dy < N; dy++)
  {
    for (size_t dx = 0; dx < N; dx++)
    {
      cv::Point2d raw_pt {
        roi_raw.x + static_cast<double>(dx) / (N-1) * roi_raw.width,
        roi_raw.y + static_cast<double>(dy) / (N-1) * roi_raw.height};
      cv::Point2d rect_pt = rectifyPoint(raw_pt);
      if (!std::isfinite(rect_pt.x) || !std::isfinite(rect_pt.y))
        continue;
      oX0 = std::min(oX0, rect_pt.x);
      oX1 = std::max(oX1, rect_pt.x);
      oY0 = std::min(oY0, rect_pt.y);
      oY1 = std::max(oY1, rect_pt.y);
    }
  }

  if (!std::isfinite(oX0) || !std::isfinite(oY0) || !std::isfinite(oX1) || !std::isfinite(oY1))
    return cv::Rect();
  return cv::Rect(std::ceil(oX0), std::ceil(oY0), std::floor(oX1 - oX0), std::floor(oY1 - oY0));
}

cv::Rect PinholeCameraModel::unrectifyRoi(const cv::Rect& roi_rect) const
{
  assert( initialized() );

  double oX0 = std::numeric_limits<double>::infinity();
  double oX1 = -std::numeric_limits<double>::infinity();
  double oY0 = std::numeric_limits<double>::infinity();
  double oY1 = -std::numeric_limits<double>::infinity();

  const int N = 5;
  for (size_t dy = 0; dy < N; dy++)
  {
    for (size_t dx = 0; dx < N; dx++)
    {
      cv::Point2d rect_pt {
        roi_rect.x + static_cast<double>(dx) / (N-1) * roi_rect.width,
        roi_rect.y + static_cast<double>(dy) / (N-1) * roi_rect.height};
      cv::Point2d raw_pt = unrectifyPoint(rect_pt);
      oX0 = std::min(oX0, raw_pt.x);
      oX1 = std::max(oX1, raw_pt.x);
      oY0 = std::min(oY0, raw_pt.y);
      oY1 = std::max(oY1, raw_pt.y);
    }
  }

  return cv::Rect(std::floor(oX0), std::floor(oY0), std::ceil(oX1 - oX0), std::ceil(oY1 - oY0));
}


void PinholeCameraModel::unrectifyImage(const cv::Mat& rectified, cv::Mat& raw, int interpolation) const
{
  assert( initialized() );

  switch (cache_->distortion_state) {
    case image_geometry::NONE:
      rectified.copyTo(raw);
    break;
    case image_geometry::CALIBRATED:
      initUnrectificationMaps();
    if (rectified.depth() == CV_32F || rectified.depth() == CV_64F)
    {
      cv::remap(rectified, raw, cache_->unrectify_reduced_map1, cache_->unrectify_reduced_map2, interpolation, cv::BORDER_CONSTANT, std::numeric_limits<float>::quiet_NaN());
    }
    else {
      cv::remap(rectified, raw, cache_->unrectify_reduced_map1, cache_->unrectify_reduced_map2, interpolation);
    }
    break;
    default:
      assert(cache_->distortion_state == image_geometry::UNKNOWN);
    throw image_geometry::Exception("Cannot call rectifyImage when distortion is unknown.");
  }
}

cv::Size PinholeCameraModel::getRectifiedResolution() const
{
  cv::Rect outer = this->rectifyRoi(cv::Rect(0, 0, this->reducedResolution().width, this->reducedResolution().height));
  if (!outer.empty())
  {
    outer.x /= this->binningX();
    outer.y /= this->binningY();
    outer.width /= this->binningX();
    outer.height /= this->binningY();
  }
  return outer.size();
}

PinholeCameraModel PinholeCameraModel::getModelForResolution(const cv::Size& res) const
{
  const cv::Matx33d K = cv::getOptimalNewCameraMatrix(
      this->intrinsicMatrix(), this->distortionCoeffs(), this->reducedResolution(), 1.0, res);

  // Adjust the projection matrix by the newly computed K matrix
  // TODO this will probably not work with stereo cams that have P different from K|t.
  auto P = cv::Mat(this->projectionMatrix());
  cv::copyTo(K, P(cv::Range(0, 3), cv::Range(0, 3)), cv::Mat());

  auto newCamInfo = this->cameraInfo();

  newCamInfo.width = res.width;
  newCamInfo.height = res.height;

  std::copy(K.val, K.val + 9, newCamInfo.K.begin());
  std::copy(P.begin<double>(), P.end<double>(), newCamInfo.P.begin());

  if (newCamInfo.roi.width == this->fullResolution().width)
  {
    newCamInfo.roi.width = 0;
  }
  else if (newCamInfo.roi.width != 0)
  {
    const double ratio = static_cast<double>(res.width) / this->reducedResolution().width;
    newCamInfo.roi.x_offset *= ratio;
    newCamInfo.roi.width *= ratio;
  }

  if (newCamInfo.roi.height == this->fullResolution().height)
  {
    newCamInfo.roi.height = 0;
  }
  else if (newCamInfo.roi.height != 0)
  {
    const double ratio = static_cast<double>(res.height) / this->reducedResolution().height;
    newCamInfo.roi.y_offset *= ratio;
    newCamInfo.roi.height *= ratio;
  }

  return PinholeCameraModel(newCamInfo);
}

const cv::Mat& PinholeCameraModel::getReducedUnrectifyFloatMap() const
{
  this->initUnrectificationMaps();
  return this->extraCache->unrectify_reduced_float_map;
}

} // namespace robot_model_renderer

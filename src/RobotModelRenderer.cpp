// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: 2012 Willow Garage, Inc.
// SPDX-FileCopyrightText: Czech Technical University in Prague

// Parts of this file are taken from rviz

/**
 * \file
 * \brief Renderer of robot model from URDF.
 * \author Martin Pecka
 */

#include <set>
#include <string>

#include <boost/bind/bind.hpp>

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreRectangle2D.h>
#include <OgreRenderSystem.h>
#include <OgreRenderWindow.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTextureManager.h>
#include <OgreViewport.h>
#include <OgreTechnique.h>
#include <OgreCamera.h>
#include <OgreHardwarePixelBuffer.h>

#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/set_utils.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <robot_model_renderer/RobotModelRenderer.hpp>
#include <robot_model_renderer/compositors/OgreCameraDistortion.hpp>
#include <robot_model_renderer/compositors/OgreInvertColors.hpp>
#include <robot_model_renderer/compositors/OgreOutline.hpp>
#include <robot_model_renderer/ogre_helpers/render_system.hpp>
#include <robot_model_renderer/ogre_helpers/compatibility.hpp>
#include <robot_model_renderer/utils/validate_floats.hpp>
#include <robot_model_renderer/utils/ogre_opencv.hpp>
#include <robot_model_renderer/utils/sensor_msgs.hpp>

namespace robot_model_renderer
{

bool RenderErrors::hasError() const
{
  return updateErrors.hasError();
}

std::string RenderErrors::toString() const
{
  if (!updateErrors.hasError())
    return "";
  return "Links failed to update: [" + updateErrors.toString() + "]";
}

RobotModelRenderer::RobotModelRenderer(
  const cras::LogHelperPtr& log, const urdf::Model& model, LinkUpdater* linkUpdater, RobotErrors& errors,
  const RobotModelRendererConfig& config, Ogre::SceneManager* sceneManager, Ogre::SceneNode* sceneNode,
  Ogre::Camera* camera) :
    cras::HasLogger(log), linkUpdater(linkUpdater), config(config), isDistorted(false), render_system_(log),
    scene_manager_(sceneManager), scene_node_(sceneNode), camera_(camera), distortionPass_(log, false),
    invertColorsPass_(log, config.invertAlpha),
    outlinePass_(log, config.outlineWidth, config.outlineColor, config.outlineFromClosestColor),
    cvImageType(ogrePixelFormatToCvMatType(config.pixelFormat))
{
  if (sceneManager == nullptr && sceneNode != nullptr)
    throw std::runtime_error("When sceneManager is not passed, sceneNode has to be null too.");

  if (sceneManager == nullptr && camera != nullptr)
    throw std::runtime_error("When sceneManager is not passed, camera has to be null too.");

  auto renderLock {render_system_.lock()};

  if (sceneManager == nullptr)
  {
#if (OGRE_VERSION < OGRE_VERSION_CHECK(13, 0, 0))
    scene_manager_ = render_system_.root()->createSceneManager(Ogre::ST_GENERIC);
#else
    scene_manager_ = render_system_.root()->createSceneManager();
#endif
    CRAS_DEBUG_NAMED("renderer", "Created scene manager");
  }

  if (this->config.setupDefaultLighting)
  {
    this->default_light_ = this->scene_manager_->createLight("MainLight");
    this->default_light_->setType(Ogre::Light::LT_POINT);
    this->default_light_->setDiffuseColour(Ogre::ColourValue(1.0f, 1.0f, 1.0f));
    this->scene_manager_->setAmbientLight(Ogre::ColourValue(.5, .5, .5));
  }

  if (sceneNode == nullptr)
    this->scene_node_ = this->scene_manager_->getRootSceneNode()->createChildSceneNode();

  if (camera == nullptr)
  {
    this->camera_ = this->scene_manager_->createCamera("RobotModelCamera");
    if (this->config.nearClipDistance > 0.0f)
      this->camera_->setNearClipDistance(this->config.nearClipDistance);
    if (this->config.farClipDistance > 0.0f)
      this->camera_->setFarClipDistance(this->config.farClipDistance);
    // convert vision (Z-forward) frame to ogre frame (Z-out)
    this->camera_->setOrientation(Ogre::Quaternion(Ogre::Degree(180), Ogre::Vector3::UNIT_X));
    CRAS_DEBUG_NAMED("renderer", "Created scene camera");
  }

  distortionPass_.SetCamera(camera_);
  outlinePass_.SetCamera(camera_);
  invertColorsPass_.SetCamera(camera_);

  const auto setModelResult = this->setModel(model);
  if (setModelResult.has_value())
    CRAS_INFO_NAMED("renderer", "Robot model renderer initialized");
  else
    errors = setModelResult.error();
}

RobotModelRenderer::~RobotModelRenderer() = default;

cras::expected<void, RobotErrors> RobotModelRenderer::setModel(const urdf::Model& model)
{
  this->robot_ = std::make_unique<Robot>(this->log, this->scene_node_, this->scene_manager_, "robot");
  auto loadResult = this->robot_->load(model, this->config.shapeFilter, this->config.shapeInflationRegistry);
  if (!loadResult.has_value())
    return loadResult;

  this->setVisualVisible(this->config.shapeFilter->isVisualAllowed());
  this->setCollisionVisible(this->config.shapeFilter->isCollisionAllowed());

  if (this->config.renderingMode == RenderingMode::MASK)
    this->robot_->setMaskMode();
  else if (this->config.renderingMode == RenderingMode::COLOR)
    this->robot_->setColorMode(
      this->config.colorModeColor.r, this->config.colorModeColor.g, this->config.colorModeColor.b);

  CRAS_INFO_NAMED("renderer", "Loaded robot model %s with %zu links and %zu joints",
    model.getName().c_str(), this->robot_->getLinks().size(), this->robot_->getJoints().size());

  return {};
}

void RobotModelRenderer::setNearClipDistance(const double nearClip)
{
  this->config.nearClipDistance = nearClip;
  this->camera_->setNearClipDistance(static_cast<Ogre::Real>(nearClip));
}

void RobotModelRenderer::setFarClipDistance(const double farClip)
{
  this->config.farClipDistance = farClip;
  this->camera_->setFarClipDistance(static_cast<Ogre::Real>(farClip));
}

void RobotModelRenderer::setVisualVisible(const bool visible)
{
  if (this->robot_ != nullptr)
    this->robot_->setVisualVisible(visible);
}

void RobotModelRenderer::setCollisionVisible(const bool visible)
{
  if (this->robot_ != nullptr)
    this->robot_->setCollisionVisible(visible);
}

void RobotModelRenderer::setPixelFormat(const Ogre::PixelFormat& pf)
{
  this->config.pixelFormat = pf;
  this->cvImageType = ogrePixelFormatToCvMatType(pf);
}

void RobotModelRenderer::updateOgreCamera()
{
  const auto& cam = this->isDistorted ? this->rectifiedCameraModel : this->origCameraModel;
  const auto& resolution = cam.reducedResolution();
  const auto& P = cam.projectionMatrix();

  const auto img_width = resolution.width;
  const auto img_height = resolution.height;

  const auto fx = P(0, 0);
  const auto fy = P(1, 1);

  // calculate the projection matrix
  const auto cx = P(0, 2);
  const auto cy = P(1, 2);

  const auto far_plane = this->camera_->getFarClipDistance();
  const auto near_plane = this->camera_->getNearClipDistance();

  Ogre::Matrix4 proj_matrix {Ogre::Matrix4::ZERO};

  proj_matrix[0][0] = 2.0f * fx / img_width;
  proj_matrix[1][1] = 2.0f * fy / img_height;

  proj_matrix[0][2] = 2.0f * (0.5f - cx / img_width);
  proj_matrix[1][2] = 2.0f * (cy / img_height - 0.5f);

  proj_matrix[2][2] = -(far_plane + near_plane) / (far_plane - near_plane);
  proj_matrix[2][3] = -2.0f * far_plane * near_plane / (far_plane - near_plane);

  proj_matrix[3][2] = -1.0f;

  this->camera_->setCustomProjectionMatrix(true, proj_matrix);
}

bool RobotModelRenderer::updateCameraInfo(const robot_model_renderer::PinholeCameraModel& model)
{
  if (model.reducedResolution().height == 0 || model.reducedResolution().width == 0)
  {
    CRAS_ERROR_THROTTLE_NAMED(1.0, "camera_info",
      "Could not determine width/height of image due to malformed CameraInfo (either width or height is 0)");
    return false;
  }

  if (model.intrinsicMatrix()(0, 0) == 0.0 || model.intrinsicMatrix()(1, 1) == 0)
  {
    CRAS_ERROR_THROTTLE_NAMED(1.0, "camera_info", "Camera info contains invalid intrinsic matrix.");
    return false;
  }

  if (model.projectionMatrix()(0, 0) == 0.0 || model.projectionMatrix()(1, 1) == 0)
  {
    CRAS_ERROR_THROTTLE_NAMED(1.0, "camera_info", "Camera info contains invalid projection matrix.");
    return false;
  }

  const auto numD = model.distortionCoeffs().size().area();
  if (numD != 0 && numD != 4 && numD != 5 && numD != 8 && numD != 12 && numD != 14)
  {
    CRAS_ERROR_THROTTLE_NAMED(1.0, "camera_info", "Unsupported number of distortion coeffs.");
    return false;
  }

  this->isDistorted = this->config.doDistort && numD > 0 &&
    std::any_of(model.distortionCoeffs().begin(), model.distortionCoeffs().end(),
      [](const double x) { return fabs(x) > 1e-6; });

  cv::Size rectifiedRes;
  if (this->isDistorted)
  {
    rectifiedRes = model.getRectifiedResolution();
    if (rectifiedRes.empty())
    {
      CRAS_ERROR_THROTTLE_NAMED(1.0, "camera_info", "Could not determine rectified image dimensions.");
      return false;
    }
    // Do not allow the rectified image to be smaller (for pincushion distortion).
    if (rectifiedRes.area() < model.reducedResolution().area())
      rectifiedRes = model.reducedResolution();
  }

  if (areCameraInfosAlmostEqual(model.cameraInfo(), this->origCameraModel.cameraInfo(), 0))
    return true;

  CRAS_DEBUG_NAMED("camera_info", "Updating new camera info");

  const auto prevOrigCamMsg = this->origCameraModel.cameraInfo();
  this->origCameraModel = model;

  if (this->isDistorted)
  {
    const auto prevRectCamMsg = this->rectifiedCameraModel.cameraInfo();
    this->rectifiedCameraModel = this->origCameraModel.getModelForResolution(rectifiedRes);

    if (this->config.gpuDistortion && !distortionPass_.SetCameraModel(this->rectifiedCameraModel))
    {
      this->origCameraModel.fromCameraInfo(prevOrigCamMsg);
      this->rectifiedCameraModel.fromCameraInfo(prevRectCamMsg);
      return false;
    }
  }

  this->reset();

  const auto& cam = this->isDistorted ? this->rectifiedCameraModel : this->origCameraModel;
  const auto res = cam.reducedResolution();

  {
    auto renderLock {render_system_.lock()};

    tex_ = render_system_.root()->getTextureManager()->createManual(
      "MainRenderTarget", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D,
      res.width, res.height, 32, 0, this->config.pixelFormat, Ogre::TU_RENDERTARGET);
    rt_ = tex_->getBuffer()->getRenderTarget();

    viewPort_ = rt_->addViewport(camera_);
    viewPort_->setBackgroundColour(this->config.backgroundColor);
    viewPort_->setClearEveryFrame(true);

    if (this->isDistorted && this->config.gpuDistortion)
      distortionPass_.CreateRenderPass();

    if (this->config.drawOutline)
      outlinePass_.CreateRenderPass();

    if (this->config.invertColors)
      invertColorsPass_.CreateRenderPass();
  }

  this->updateOgreCamera();

  return true;
}

cras::expected<cv::Mat, std::string> RobotModelRenderer::render(const ros::Time& time, RenderErrors& errors)
{
  const auto updateResult = robot_->update(*this->linkUpdater, time);
  if (!updateResult.has_value())
  {
    errors.updateErrors = updateResult.error();

    if (config.allLinksRequired)
      return cras::make_unexpected("Some links' transforms could not be updated.");

    for (const auto& [linkName, error] : updateResult.error().linkErrors)
    {
      if (config.requiredLinks.find(linkName) != config.requiredLinks.end())
        return cras::make_unexpected(cras::format("Update of required link [%s] failed.", linkName.c_str()));
    }
  }

  const auto rectRows = static_cast<int>(rt_->getHeight());
  const auto rectCols = static_cast<int>(rt_->getWidth());

  cv::Mat rectImg(rectRows, rectCols, this->cvImageType);
  const Ogre::PixelBox pb(rt_->getWidth(), rt_->getHeight(), 1, this->config.pixelFormat, rectImg.data);

  {
    auto renderLock {render_system_.lock()};

    rt_->update();  // Perform the actual rendering
    rt_->copyContentsToMemory(pb);
  }

  // If distortion is not applied, this is all, we have a rectified image of correct size
  if (!this->isDistorted)
    return rectImg;

  // If distortion is done on GPU, this will already be raw image
  auto rawImg = rectImg;

  // Otherwise, perform the distortion on CPU now
  if (!this->config.gpuDistortion)
  {
    // This will be the raw image, but still with the size of the rectified one. It will get cropped later.
    cv::Mat unrectImg(rectImg.rows, rectImg.cols, this->cvImageType);

    this->rectifiedCameraModel.unrectifyImage(rectImg, unrectImg);
    rawImg = unrectImg;
  }

  // The raw image can be larger than the original resolution, so we need to crop it appropriately
  const auto origRows = this->origCameraModel.reducedResolution().height;
  const auto origCols = this->origCameraModel.reducedResolution().width;

  // Compute the offset of the top left corner of the desired output image in the possibly larger rendered image.
  const auto topLeft = this->rectifiedCameraModel.unrectifyPoint({0, 0});
  const auto colOffset = std::max(0, static_cast<int>(topLeft.x));
  const auto rowOffset = std::max(0, static_cast<int>(topLeft.y));

  return rawImg(cv::Rect(colOffset, rowOffset, origCols, origRows));
}

void RobotModelRenderer::reset()
{
  auto renderLock {render_system_.lock()};

  // Unregister the previously created render passes if any
  invertColorsPass_.Destroy();
  outlinePass_.Destroy();
  distortionPass_.Destroy();

  if (rt_)
  {
    rt_->removeAllViewports();
    rt_ = nullptr;
  }

  viewPort_ = nullptr;

  if (!tex_.isNull())
  {
    render_system_.root()->getTextureManager()->remove(tex_->getHandle());
    tex_.setNull();
  }
}

}

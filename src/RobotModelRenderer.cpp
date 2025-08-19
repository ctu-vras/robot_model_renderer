// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: 2012 Willow Garage, Inc.
// SPDX-FileCopyrightText: Czech Technical University in Prague

// Parts of this file are taken from rviz

/**
 * \file
 * \brief Renderer of robot model from URDF.
 * \author Martin Pecka
 */

#include <robot_model_renderer/RobotModelRenderer.hpp>

#include <set>
#include <string>

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

#include <opencv2/calib3d/calib3d.hpp>

#include <cras_cpp_common/set_utils.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <robot_model_renderer/compositors/OgreCameraDistortion.hpp>
#include <robot_model_renderer/compositors/OgreInvertColors.hpp>
#include <robot_model_renderer/compositors/OgreOutline.hpp>
#include <robot_model_renderer/ogre_helpers/render_system.hpp>
#include <robot_model_renderer/ogre_helpers/compatibility.hpp>
#include <robot_model_renderer/utils/validate_floats.hpp>
#include <robot_model_renderer/utils/ogre_opencv.hpp>
#include <robot_model_renderer/utils/sensor_msgs.hpp>
#include <robot_model_renderer/utils/sensor_msgs_ogre.hpp>

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
    staticImagePass_(log, config.staticMaskImage, sensorMsgsEncodingToOgrePixelFormat(config.staticMaskImageEncoding),
      config.pixelFormat, config.staticMaskIsBackground, config.renderingMode, config.colorModeColor),
    cvImageType(ogrePixelFormatToCvMatType(config.pixelFormat))
{
  if (sceneManager == nullptr && sceneNode != nullptr)
    throw std::runtime_error("When sceneManager is not passed, sceneNode has to be null too.");

  if (sceneManager == nullptr && camera != nullptr)
    throw std::runtime_error("When sceneManager is not passed, camera has to be null too.");

  {
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
      static size_t i = 0;
      ++i;
      this->camera_ = this->scene_manager_->createCamera("RobotModelCamera" + std::to_string(i));
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

    if (this->hasOverlays())
    {
#if (OGRE_VERSION < OGRE_VERSION_CHECK(13, 0, 0))
      this->overlay_scene_manager_ = this->render_system_.root()->createSceneManager(Ogre::ST_GENERIC);
#else
      this->overlay_scene_manager_ = this->render_system_.root()->createSceneManager();
#endif

      this->overlay_scene_node_ = this->overlay_scene_manager_->getRootSceneNode()->createChildSceneNode();

      this->overlay_.bind(new Ogre::Rectangle2D(true));
      this->overlay_->setCorners(-1, 1, 1, -1);
      this->overlay_->setBoundingBox(Ogre::AxisAlignedBox::BOX_INFINITE);
      this->overlay_scene_node_->createChildSceneNode()->attachObject(this->overlay_.get());

      this->overlay_camera_ = this->overlay_scene_manager_->createCamera("OverlayCamera");

      this->staticImagePass_.SetCamera(this->overlay_camera_);
    }

    invertColorsPass_.SetCamera(RobotModelRenderer::hasOverlays() ? overlay_camera_ : camera_);
  }

  const auto setModelResult = RobotModelRenderer::setModel(model);
  if (setModelResult.has_value())
    CRAS_INFO_NAMED("renderer", "Robot model renderer initialized");
  else
    errors = setModelResult.error();
}

RobotModelRenderer::~RobotModelRenderer()
{
  RobotModelRenderer::reset();

  if (scene_manager_)
  {
    if (default_light_)
    {
      scene_manager_->destroyLight(default_light_);
      default_light_ = nullptr;
    }

    if (camera_)
    {
      scene_manager_->destroyCamera(camera_);
      camera_ = nullptr;
    }

    // Delete the robot instance before scene_manager_ to allow it to deregister its nodes
    if (robot_)
      robot_.reset();

    if (scene_node_)
    {
      scene_manager_->destroySceneNode(scene_node_);
      scene_node_ = nullptr;
    }

    render_system_.root()->destroySceneManager(scene_manager_);
    scene_manager_ = nullptr;
  }

  if (overlay_scene_manager_)
  {
    if (overlay_camera_)
    {
      overlay_scene_manager_->destroyCamera(overlay_camera_);
      overlay_camera_ = nullptr;
    }

    if (overlay_scene_node_)
    {
      overlay_scene_manager_->destroySceneNode(overlay_scene_node_);
      overlay_scene_node_ = nullptr;
    }

    overlay_.setNull();

    render_system_.root()->destroySceneManager(overlay_scene_manager_);
    overlay_scene_manager_ = nullptr;
  }
}

cras::expected<void, RobotErrors> RobotModelRenderer::setModel(const urdf::Model& model)
{
  auto renderLock {render_system_.lock()};

  this->robot_ = std::make_unique<Robot>(this->log, this->scene_node_, this->scene_manager_, "robot");
  auto loadResult = this->robot_->load(model, this->config.shapeFilter, this->config.shapeInflationRegistry);
  if (!loadResult.has_value())
    return loadResult;

  this->setVisualVisible(this->config.shapeFilter ? this->config.shapeFilter->isVisualAllowed() : true);
  this->setCollisionVisible(this->config.shapeFilter ? this->config.shapeFilter->isCollisionAllowed() : false);

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
  const auto& cam = this->isDistorted ? this->rectifiedCameraModel : this->renderingCameraModel;
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

  if (far_plane == 0)  // Infinite far plane
  {
    proj_matrix[2][2] = Ogre::Frustum::INFINITE_FAR_PLANE_ADJUST - 1;
    proj_matrix[2][3] = near_plane * (Ogre::Frustum::INFINITE_FAR_PLANE_ADJUST - 2);
  }
  else
  {
    proj_matrix[2][2] = -(far_plane + near_plane) / (far_plane - near_plane);
    proj_matrix[2][3] = -2.0f * far_plane * near_plane / (far_plane - near_plane);
  }

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

  if (areCameraInfosAlmostEqual(model.cameraInfo(), this->origCameraModel.cameraInfo(), 0))
    return true;

  CRAS_DEBUG_NAMED("camera_info", "Updating new camera info");

  // Invalidate cache if camera geometry changes and static rendering is enabled
  if (this->config.renderedImageIsStatic && this->cached_image_.total() > 0)
  {
    this->cached_image_ = cv::Mat();
    CRAS_DEBUG_NAMED("camera_info", "Invalidating cached image due to camera geometry change");
  }

  const auto prevOrigCamMsg = this->origCameraModel.cameraInfo();
  this->origCameraModel = model;

  // Determine the scale of the rendered image
  double scale = this->config.renderImageScale;

  // Ignore scalings very close to 1.0
  if (std::abs(scale - 1.0) < 1e-3)
    scale = 1.0;

  // Do not allow upscaling
  if (scale > 1.0)
  {
    CRAS_WARN_THROTTLE_NAMED(1.0, "renderer", "Setting the model rendering scale to a value larger than 1.0 is not "
      "supported (and it does not make much sense). Setting to 1.0 instead.");
    scale = 1.0;
  }

  const auto maxTexSize = this->config.maxRenderImageSize == 0u ? this->render_system_.getGlMaxTextureSize() :
    std::min(static_cast<int>(this->config.maxRenderImageSize), this->render_system_.getGlMaxTextureSize());

  if (scale * model.reducedResolution().width > maxTexSize || scale * model.reducedResolution().height > maxTexSize)
  {
    CRAS_WARN_ONCE_NAMED("renderer", "Image size %i x %i is larger than the maximum supported size %i x %i . Scaling "
      "down the image.", static_cast<int>(scale * model.reducedResolution().width),
      static_cast<int>(scale * model.reducedResolution().height), maxTexSize, maxTexSize);

    scale = static_cast<double>(maxTexSize) /
      std::max(model.reducedResolution().width, model.reducedResolution().height);
  }

  const auto prevRenderCamMsg = this->renderingCameraModel.cameraInfo();
  this->renderingCameraModel = model.getScaled(scale);

  cv::Size rectifiedRes;
  if (this->isDistorted)
  {
    rectifiedRes = this->renderingCameraModel.getRectifiedResolution();
    if (rectifiedRes.width == 0 || rectifiedRes.height == 0)
    {
      CRAS_ERROR_THROTTLE_NAMED(1.0, "camera_info", "Could not determine rectified image dimensions.");
      this->origCameraModel.fromCameraInfo(prevOrigCamMsg);
      this->renderingCameraModel.fromCameraInfo(prevRenderCamMsg);
      return false;
    }
    this->idealRectifiedCameraResolution = rectifiedRes;
    // Do not allow the rectified image to be smaller (for pincushion distortion).
    if (rectifiedRes.area() < this->renderingCameraModel.reducedResolution().area())
      rectifiedRes = this->renderingCameraModel.reducedResolution();

    if (this->config.gpuDistortion && (rectifiedRes.width > maxTexSize || rectifiedRes.height > maxTexSize))
    {
      CRAS_WARN_ONCE_NAMED("renderer", "Distorted image size %i x %i is larger than the maximum supported size "
        "%i x %i . Scaling down the image.", rectifiedRes.width, rectifiedRes.height, maxTexSize, maxTexSize);

      const double ratio = std::max(
        rectifiedRes.width / static_cast<double>(maxTexSize), rectifiedRes.height / static_cast<double>(maxTexSize));
      rectifiedRes = cv::Size(std::floor(rectifiedRes.width / ratio), std::floor(rectifiedRes.height / ratio));
    }

    const auto prevRectCamMsg = this->rectifiedCameraModel.cameraInfo();
    this->rectifiedCameraModel = this->renderingCameraModel.getModelForResolution(rectifiedRes);

    if (this->config.gpuDistortion && !distortionPass_.SetCameraModel(this->rectifiedCameraModel))
    {
      this->origCameraModel.fromCameraInfo(prevOrigCamMsg);
      this->renderingCameraModel.fromCameraInfo(prevRenderCamMsg);
      this->rectifiedCameraModel.fromCameraInfo(prevRectCamMsg);
      return false;
    }
  }

  this->reset();

  const auto& cam = this->isDistorted ? this->rectifiedCameraModel : this->renderingCameraModel;
  const auto res = cam.reducedResolution();

  {
    auto renderLock {render_system_.lock()};

    CRAS_INFO_THROTTLE_NAMED(1.0, "renderer",
      "Creating offscreen render texture with resolution %ix%i.", res.width, res.height);

    static size_t num = 0;
    num++;
    const auto numStr = std::to_string(num);

    tex_ = render_system_.root()->getTextureManager()->createManual(
      "MainRenderTarget" + numStr, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D,
      res.width, res.height, 1, 0, this->config.pixelFormat, Ogre::TU_RENDERTARGET);
    rt_ = tex_->getBuffer()->getRenderTarget();

    viewPort_ = rt_->addViewport(camera_);
    viewPort_->setBackgroundColour(this->config.backgroundColor);
    viewPort_->setClearEveryFrame(true);

    if (this->isDistorted && this->config.gpuDistortion)
      distortionPass_.CreateRenderPass();

    if (this->config.drawOutline)
    {
      // If we render a downscaled image, scale the outline so that it appears the same size regardless of downscaling
      const auto outlineWidth = this->config.outlineWidth *
        (static_cast<double>(res.width) / this->origCameraModel.reducedResolution().width);
      outlinePass_.SetOutlineWidth(static_cast<float>(outlineWidth));
      outlinePass_.CreateRenderPass();
    }

    if (this->config.invertColors && !this->hasOverlays())
      invertColorsPass_.CreateRenderPass();

    if (this->hasOverlays())
    {
      const auto origRes = this->origCameraModel.reducedResolution();

      CRAS_INFO_THROTTLE_NAMED(1.0, "renderer",
        "Creating overlay offscreen render texture with resolution %ix%i.", origRes.width, origRes.height);

      overlay_tex_ = render_system_.root()->getTextureManager()->createManual(
        "OverlayRenderTarget" + numStr, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D,
        origRes.width, origRes.height, 1, 0, this->config.pixelFormat, Ogre::TU_RENDERTARGET);
      overlay_rt_ = overlay_tex_->getBuffer()->getRenderTarget();

      overlay_viewPort_ = overlay_rt_->addViewport(overlay_camera_);
      overlay_viewPort_->setBackgroundColour(this->config.backgroundColor);

      overlay_scene_tex_ = render_system_.root()->getTextureManager()->createManual(
        "OverlaySceneTex" + numStr, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D,
        origRes.width, origRes.height, 1, 0, this->config.pixelFormat, Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

      overlay_material_ = Ogre::MaterialManager::getSingleton().create(
        "OverlayMat" + numStr, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
      overlay_material_->getTechnique(0)->getPass(0)->setLightingEnabled(false);
      overlay_material_->getTechnique(0)->getPass(0)->createTextureUnitState("OverlaySceneTex" + numStr);
      overlay_material_->setDepthWriteEnabled(false);
      overlay_material_->setDepthCheckEnabled(false);
      overlay_material_->setCullingMode(Ogre::CULL_NONE);
      overlay_->setMaterial("OverlayMat" + numStr);

      if (this->config.staticMaskImage.total() > 0)
        staticImagePass_.CreateRenderPass();

      if (this->config.invertColors)
        invertColorsPass_.CreateRenderPass();
    }
  }

  this->updateOgreCamera();

  return true;
}

cras::expected<cv::Mat, std::string> RobotModelRenderer::render(const ros::Time& time, RenderErrors& errors)
{
  // Check if we can use cached rendering for static images
  if (this->config.renderedImageIsStatic && this->cached_image_.total() > 0)
    return this->cached_image_.clone();

  auto renderResult = this->renderInner(time, errors);
  if (!renderResult.has_value())
    return renderResult;

  cv::Mat finalImage;

  if (!this->hasOverlays())
  {
    finalImage = *renderResult;
  }
  else
  {
    finalImage = cv::Mat(renderResult->rows, renderResult->cols, this->cvImageType);
    {
      auto renderedImg = renderResult->isContinuous() ? *renderResult : renderResult->clone();

      auto renderLock {render_system_.lock()};

      // Draw the previously rendered scene onto overlay_scene_tex_ .
      {
        const Ogre::PixelBox renderPb(renderedImg.cols, renderedImg.rows, 1, this->config.pixelFormat,
          renderedImg.data);
        overlay_scene_tex_->getBuffer()->blitFromMemory(renderPb);
      }

      overlay_rt_->update();

      const Ogre::PixelBox pb(overlay_tex_->getWidth(), overlay_tex_->getHeight(), 1, this->config.pixelFormat,
        finalImage.data);
      overlay_rt_->copyContentsToMemory(pb);
    }
  }

  if (this->config.renderedImageIsStatic)
    this->cached_image_ = finalImage.clone();

  return finalImage;
}

cras::expected<cv::Mat, std::string> RobotModelRenderer::renderInner(const ros::Time& time, RenderErrors& errors)
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

  // If distortion is not applied, this is all, we already have a rectified image
  if (!this->isDistorted)
  {
    // If the rendered image has the correct size, directly return it; otherwise it needs to be upscaled
    if (this->origCameraModel.reducedResolution() == this->renderingCameraModel.reducedResolution())
      return rectImg;

    cv::Mat scaledImg;
    cv::resize(rectImg, scaledImg, this->origCameraModel.reducedResolution(), 0, 0,
      this->config.upscalingInterpolation);
    return scaledImg;
  }

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

  // Rectified resolution is either same or smaller than the ideal. It cannot be larger. So scale is up to 1.0.
  const auto& idealRes = this->idealRectifiedCameraResolution;
  auto scale = std::max(
    rectCols / static_cast<double>(idealRes.width), rectRows / static_cast<double>(idealRes.height));

  // Don't care about small scalings
  if (std::abs(scale - 1.0) < 1e-3)
    scale = 1.0;

  // The raw image can be larger than the original resolution, so we need to crop it appropriately; also, account for
  // rectified image scaling.
  const auto origRows = static_cast<int>(this->renderingCameraModel.reducedResolution().height * scale);
  const auto origCols = static_cast<int>(this->renderingCameraModel.reducedResolution().width * scale);

  // Compute the offset of the centers of the larger rectified image and the original raw image. This will be the offset
  // to apply when cropping back to the original size.
  const auto cropX = std::max<int>(0, this->rectifiedCameraModel.cx() - this->renderingCameraModel.cx() * scale);
  const auto cropY = std::max<int>(0, this->rectifiedCameraModel.cy() - this->renderingCameraModel.cy() * scale);
  const auto cropCols = std::min(rawImg.cols - cropX, origCols);
  const auto cropRows = std::min(rawImg.rows - cropY, origRows);

  auto croppedImg = rawImg(cv::Rect(cropX, cropY, cropCols, cropRows));

  if (croppedImg.cols == this->origCameraModel.reducedResolution().width)
    return croppedImg;

  // TODO if the original image fits into texture memory, the cropping and scaling could be done in a shader
  cv::Mat scaledCroppedImg;
  // There might be double scaling applied here - one from distortion texture (if it was smaller than it should)
  // and one from renderingCameraModel (if the rendering resolution is smaller than original image). However, it is
  // perfectly okay to "merge" them both here, so we directly scale from the distorted image to the original size.
  cv::resize(croppedImg, scaledCroppedImg, this->origCameraModel.reducedResolution(), 0, 0,
    this->config.upscalingInterpolation);
  return scaledCroppedImg;
}

bool RobotModelRenderer::hasOverlays() const
{
  return this->config.staticMaskImage.total() > 0;
}

void RobotModelRenderer::reset()
{
  auto renderLock {render_system_.lock()};

  // Unregister the previously created render passes if any
  invertColorsPass_.Destroy();
  outlinePass_.Destroy();
  distortionPass_.Destroy();
  staticImagePass_.Destroy();

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

  if (overlay_rt_)
  {
    overlay_rt_->removeAllViewports();
    overlay_rt_ = nullptr;
  }

  overlay_viewPort_ = nullptr;

  if (!overlay_tex_.isNull())
  {
    render_system_.root()->getTextureManager()->remove(overlay_tex_->getHandle());
    overlay_tex_.setNull();
  }

  if (!overlay_scene_tex_.isNull())
  {
    render_system_.root()->getTextureManager()->remove(overlay_scene_tex_->getHandle());
    overlay_scene_tex_.setNull();
  }

  if (!overlay_material_.isNull())
  {
    Ogre::MaterialManager::getSingleton().remove(overlay_material_->getName());
  }

  cached_image_ = cv::Mat();
}

}

/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

#include <cras_cpp_common/set_utils.hpp>
#include <image_transport/camera_common.h>
#include <image_geometry/pinhole_camera_model.h>
#include <robot_model_renderer/RobotModelRenderer.h>
#include <robot_model_renderer/compositors/OgreDistortionPass.hh>
#include <robot_model_renderer/compositors/OgreInvertColors.hh>
#include <robot_model_renderer/compositors/OgreOutline.hh>
#include <robot_model_renderer/ogre_helpers/render_system.h>
#include <robot_model_renderer/ogre_helpers/compatibility.h>
#include <robot_model_renderer/utils/validate_floats.h>
#include <robot_model_renderer/utils/ogre_opencv.h>
#include <robot_model_renderer/utils/sensor_msgs.h>
#include <sensor_msgs/image_encodings.h>

namespace robot_model_renderer
{

RobotModelRenderer::RobotModelRenderer(const urdf::Model& model, const LinkUpdater* linkUpdater,
  const RobotModelRendererConfig& config, Ogre::SceneManager* sceneManager, Ogre::SceneNode* sceneNode,
  Ogre::Camera* camera) :
    linkUpdater(linkUpdater), config(config), isDistorted(false),
    scene_manager_(sceneManager), scene_node_(sceneNode), camera_(camera), distortionPass_(false),
    invertColorsPass_(config.invertAlpha),
    outlinePass_(config.outlineWidth, config.outlineColor, config.outlineFromClosestColor),
    cvImageType(ogrePixelFormatToCvMatType(config.pixelFormat))
{
  if (sceneManager == nullptr && sceneNode != nullptr)
    throw std::runtime_error("When sceneManager is not passed, sceneNode has to be null too.");

  if (sceneManager == nullptr && camera != nullptr)
    throw std::runtime_error("When sceneManager is not passed, camera has to be null too.");

  if (sceneManager == nullptr)
  {
#if (OGRE_VERSION < OGRE_VERSION_CHECK(13, 0, 0))
    scene_manager_ = RenderSystem::get()->root()->createSceneManager(Ogre::ST_GENERIC);
#else
    scene_manager_ = RenderSystem::get()->root()->createSceneManager();
#endif
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
  }

  distortionPass_.SetCamera(camera_);
  outlinePass_.SetCamera(camera_);
  invertColorsPass_.SetCamera(camera_);

  this->setModel(model);
}

RobotModelRenderer::~RobotModelRenderer() = default;

void RobotModelRenderer::setModel(const urdf::Model& model)
{
  this->robot_ = std::make_unique<Robot>(this->scene_node_, this->scene_manager_, "robot");
  this->robot_->load(model, this->config.shapeFilter);

  this->setVisualVisible(this->config.shapeFilter->isVisualAllowed());
  this->setCollisionVisible(this->config.shapeFilter->isCollisionAllowed());

  if (this->config.renderingMode == RenderingMode::MASK)
    this->robot_->setMaskMode();
  else if (this->config.renderingMode == RenderingMode::COLOR)
    this->robot_->setColorMode(
      this->config.colorModeColor.r, this->config.colorModeColor.g, this->config.colorModeColor.b);
}

void RobotModelRenderer::setNearClipDistance(const double nearClip)
{
  this->camera_->setNearClipDistance(static_cast<Ogre::Real>(nearClip));
}

void RobotModelRenderer::setFarClipDistance(const double farClip)
{
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
    ROS_ERROR_THROTTLE_NAMED(1.0, "Camera Info",
      "Could not determine width/height of image due to malformed CameraInfo (either width or height is 0)");
    return false;
  }

  if (model.intrinsicMatrix()(0, 0) == 0.0 || model.intrinsicMatrix()(1, 1) == 0)
  {
    ROS_ERROR_THROTTLE_NAMED(1.0, "Camera Info", "Camera info contains invalid intrinsic matrix.");
    return false;
  }

  if (model.projectionMatrix()(0, 0) == 0.0 || model.projectionMatrix()(1, 1) == 0)
  {
    ROS_ERROR_THROTTLE_NAMED(1.0, "Camera Info", "Camera info contains invalid projection matrix.");
    return false;
  }

  const auto numD = model.distortionCoeffs().size().area();
  if (numD != 0 && numD != 4 && numD != 5 && numD != 8 && numD != 12 && numD != 14)
  {
    ROS_ERROR_THROTTLE_NAMED(1.0, "Camera Info", "Unsupported number of distortion coeffs.");
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
      ROS_ERROR_THROTTLE_NAMED(1.0, "Camera Info", "Could not determine rectified image dimensions.");
      return false;
    }
    // Do not allow the rectified image to be smaller (for pincushion distortion).
    if (rectifiedRes.area() < model.reducedResolution().area())
      rectifiedRes = model.reducedResolution();
  }

  if (areCameraInfosAlmostEqual(model.cameraInfo(), this->origCameraModel.cameraInfo(), 0))
    return true;

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

  tex_ = Ogre::TextureManager::getSingleton().createManual(
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

  this->updateOgreCamera();

  return true;
}

cv::Mat RobotModelRenderer::render(const ros::Time& time)
{
  robot_->update(*this->linkUpdater, time);

  rt_->update();  // Perform the actual rendering

  const auto rectRows = static_cast<int>(rt_->getHeight());
  const auto rectCols = static_cast<int>(rt_->getWidth());

  cv::Mat rectImg(rectRows, rectCols, this->cvImageType);

  const Ogre::PixelBox pb(rt_->getWidth(), rt_->getHeight(), 1, this->config.pixelFormat, rectImg.data);
  rt_->copyContentsToMemory(pb);

  // If distortion is not applied, this is all, we have a rectified image of correct size
  if (!this->isDistorted)
    return rectImg;

  // If distortion is done on GPU, this will already be raw image
  auto rawImg = rectImg;

  // Otherwise, perform the distortion on CPU now
  if (!this->config.gpuDistortion)
  {
    // This will be the raw image, but still with the size of the rectified one. It will get cropped later.
    cv::Mat unrectImg(rectRows, rectCols, this->cvImageType);

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
    Ogre::TextureManager::getSingleton().remove(tex_->getHandle());
    tex_.setNull();
  }
}

} // namespace robot_model_renderer

#include <memory>
#include <thread>

#include <urdf/model.h>

#include <cras_cpp_common/nodelet_utils.hpp>
#include <cras_cpp_common/param_utils.hpp>
#include <image_transport/image_transport.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <robot_model_renderer/ogre_helpers/render_system.h>
#include <robot_model_renderer/robot/shape_filter.h>
#include <robot_model_renderer/RosCameraRobotModelRenderer.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/ColorRGBA.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

namespace cras
{

DEFINE_CONVERTING_GET_PARAM(std_msgs::ColorRGBA, std::vector<float>, "[0-1]", [](const std::vector<float>& v)
{
  if (v.size() != 3 && v.size() != 4 && v.size() != 1) \
    throw std::runtime_error( \
      cras::format("Cannot load ColorRGBA parameter from an array of length %lu", v.size())); \
  std_msgs::ColorRGBA m; \
  for (const auto& c : v) if (c < 0 || c > 1) throw std::runtime_error("Color values have to be in range [0-1]."); \
  m.r = v[0]; \
  m.g = v.size() > 1 ? v[1] : v[0]; \
  m.b = v.size() > 2 ? v[2] : v[0]; \
  m.a = v.size() > 3 ? v[3] : 1; \
  return m; \
})

inline ::std::string to_string(const robot_model_renderer::RenderingMode& value)
{
  switch (value)
  {
    case robot_model_renderer::RenderingMode::NORMAL:
      return "normal";
    case robot_model_renderer::RenderingMode::COLOR:
      return "color";
    case robot_model_renderer::RenderingMode::MASK:
      return "mask";
  }
  return "unknown";
}

DEFINE_CONVERTING_GET_PARAM(robot_model_renderer::RenderingMode, std::string, "", [](const std::string& v)
{
  const auto mode = cras::toLower(v); \
  if (mode.empty() || mode == "normal") \
    return robot_model_renderer::RenderingMode::NORMAL; \
  else if (mode == "color") \
    return robot_model_renderer::RenderingMode::COLOR; \
  else if (mode == "mask") \
    return robot_model_renderer::RenderingMode::MASK; \
  else \
    throw std::runtime_error( \
      cras::format("Invalid value %s for rendering mode. Allowed are: normal, color, mask", mode.c_str())); \
})

}

namespace robot_model_renderer
{

class RobotModelRendererNodelet : public cras::Nodelet
{
public:
  ~RobotModelRendererNodelet() override
  {
    this->requestStop();
  }

  void onInit() override
  {
    this->it = std::make_unique<image_transport::ImageTransport>(this->getNodeHandle());
    this->ogreQueue.reset(new ros::CallbackQueue);
    this->ogreThread = std::thread(&RobotModelRendererNodelet::ogreLoop, this);
  }

  void reset() override
  {
    this->unsubscribe();
    this->renderer->reset();
    this->renderer.reset();
    this->onInitialize();
  }

  void requestStop() override
  {
    cras::Nodelet::requestStop();
    this->unsubscribe();
    this->ogreThread.join();
  }

private:
  /**
   * \brief All OGRE code has to run in a separate thread. When we're launched as a nodelet, the subscription callbacks
   *        are normally distributed over many threads. However, OGRE doesn't like this. So we'll instead do almost
   *        nothing on the normal callback queue and we'll spin our own callback queue for everything OGRE-related.
   *        It is important to also do the OGRE initialization in this same thread.
   */
  void ogreLoop()
  {
    this->updateThreadName();

    this->onInitialize();

    while (this->ok())
    {
      this->ogreQueue->callAvailable(ros::WallDuration(0.1));
    }
  }

  void onInitialize()
  {
    const auto& publicParams = this->publicParams();
    const auto& params = this->privateParams();

    urdf::Model robotModel;

    while (this->ok())
    {
      try
      {
        const auto robotDescription = publicParams->getParam<std::string>(
          "robot_description", cras::nullopt, "", {.printMessages = false});

        if (robotDescription.empty())
        {
          CRAS_ERROR("Parameter robot_description is empty.");
        }
        else if (robotModel.initString(robotDescription))
        {
          // We have the model!
          break;
        }
        else
        {
          CRAS_ERROR("Failed to parse URDF model");
        }
      }
      catch (const cras::GetParamException& e)
      {
        CRAS_ERROR("Failed parsing robot_description: %s", e.what());
      }

      CRAS_WARN("Parameter robot_description will be read again in 1 second.");

      try
      {
        ros::WallDuration(1.0).sleep();
      }
      catch (const std::exception& e)
      {
      }
    }

    RosCameraRobotModelRendererConfig config;

    config.renderingMode = params->getParam("rendering_mode", config.renderingMode);

    if (config.renderingMode == RenderingMode::MASK)
      config.imageEncoding = sensor_msgs::image_encodings::MONO8;

    const auto imageEncoding = params->getParam("image_encoding", config.imageEncoding);
    if (!sensor_msgs::image_encodings::isColor(imageEncoding) && !sensor_msgs::image_encodings::isMono(imageEncoding))
    {
      CRAS_FATAL("The given image encoding is not a color or mono encoding.");
      this->requestStop();
      return;
    }

    config.imageEncoding = imageEncoding;
    config.backgroundColor = params->getParam("background_color", config.backgroundColor);
    config.nearClipDistance = params->getParam("near_clip", config.nearClipDistance, "m");
    config.farClipDistance = params->getParam("far_clip", config.farClipDistance, "m");
    config.doDistort = params->getParam("do_distort", config.doDistort);
    config.gpuDistortion = params->getParam("gpu_distortion", config.gpuDistortion);
    config.colorModeColor = params->getParam("color_mode_color", config.colorModeColor);
    config.drawOutline = params->getParam("draw_outline", config.drawOutline);
    config.outlineWidth = params->getParam("outline_width", config.outlineWidth);
    config.outlineColor = params->getParam("outline_color", config.outlineColor);
    config.outlineFromClosestColor = params->getParam("outline_from_closest_color", config.outlineFromClosestColor);
    config.invertColors = params->getParam("invert_colors", config.invertColors);
    config.invertAlpha = params->getParam("invert_alpha", config.invertAlpha);

    const auto visualVisible = params->getParam("visual", true);
    const auto collisionVisible = params->getParam("collision", false);

    // can contain either whole link names, or scoped names of their collisions
    // (i.e. "link::collision_1" or "link::my_collision")
    config.shapeFilter = std::make_shared<ShapeFilter>(visualVisible, collisionVisible);
    config.shapeFilter->setIgnoreShapes(params->getParam("ignored_shapes", std::set<std::string>{}));
    config.shapeFilter->setOnlyShapes(params->getParam("only_shapes", std::set<std::string>{}));

    this->renderer = std::make_unique<RosCameraRobotModelRenderer>(robotModel, this->getBufferPtr(), config);

    this->subscribe();
  }

  void subscribe()
  {
    this->maskPub = this->it->advertise("mask", 10);

    ros::SubscribeOptions caminfoSubOpts;
    caminfoSubOpts.init<sensor_msgs::CameraInfo>(
      "camera_info", 1, boost::bind(&RobotModelRendererNodelet::processCamInfoMessage, this, boost::placeholders::_1));
    caminfoSubOpts.callback_queue = this->ogreQueue.get();
    this->caminfoSub = this->getNodeHandle().subscribe(caminfoSubOpts);
  }

  void unsubscribe()
  {
    this->caminfoSub.shutdown();
    this->maskPub.shutdown();
  }

  void processCamInfoMessage(const sensor_msgs::CameraInfo::ConstPtr& msg)
  {
    const auto img = this->renderer->render(msg);
    if (img != nullptr)
      maskPub.publish(img);
    else
      CRAS_WARN_THROTTLE(1.0, "Robot model rendering failed");
  }

  std::unique_ptr<image_transport::ImageTransport> it;
  ros::CallbackQueuePtr ogreQueue;
  std::thread ogreThread;

  ros::Subscriber caminfoSub;
  image_transport::Publisher maskPub;

  std::unique_ptr<RosCameraRobotModelRenderer> renderer;
};

}

PLUGINLIB_EXPORT_CLASS(robot_model_renderer::RobotModelRendererNodelet, nodelet::Nodelet)

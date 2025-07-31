#include <memory>
#include <thread>

#include <urdf/model.h>

#include <cras_cpp_common/nodelet_utils.hpp>
#include <cras_cpp_common/param_utils.hpp>
#include <image_transport/image_transport.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <robot_model_renderer/ogre_helpers/render_system.h>
#include <robot_model_renderer/RosCameraRobotModelRenderer.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/ColorRGBA.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

namespace cras
{

DEFINE_CONVERTING_GET_PARAM(std_msgs::ColorRGBA, std::vector<float>, "", [](const std::vector<float>& v)
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

    const auto imageEncoding = params->getParam("image_encoding", config.imageEncoding);
    if (!sensor_msgs::image_encodings::isColor(imageEncoding) && !sensor_msgs::image_encodings::isMono(imageEncoding))
    {
      CRAS_FATAL("The given image encoding is not a color or mono encoding.");
      this->requestStop();
      return;
    }

    config.imageEncoding = imageEncoding;
    config.backgroundColor = params->getParam("background_color", config.backgroundColor, "[0-1]");
    config.visualVisible = params->getParam("visual", config.visualVisible);
    config.collisionVisible = params->getParam("collision", config.collisionVisible);
    config.nearClipDistance = params->getParam("near_clip", config.nearClipDistance, "m");
    config.farClipDistance = params->getParam("far_clip", config.farClipDistance, "m");
    config.doDistort = params->getParam("do_distort", config.doDistort);
    config.gpuDistortion = params->getParam("gpu_distortion", config.gpuDistortion);

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
    if (!this->ogreInited)
    {
      RenderSystem::get()->root()->getRenderSystem()->registerThread();
      this->ogreInited = true;
    }

    const auto img = this->renderer->render(msg);
    if (img != nullptr)
      maskPub.publish(img);
    else
      CRAS_WARN_THROTTLE(1.0, "Robot model rendering failed");
  }

  std::unique_ptr<image_transport::ImageTransport> it;
  ros::CallbackQueuePtr ogreQueue;
  std::thread ogreThread;
  bool ogreInited {false};

  ros::Subscriber caminfoSub;
  image_transport::Publisher maskPub;

  std::unique_ptr<RosCameraRobotModelRenderer> renderer;
};

}

PLUGINLIB_EXPORT_CLASS(robot_model_renderer::RobotModelRendererNodelet, nodelet::Nodelet)

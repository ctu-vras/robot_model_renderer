#include <memory>

#include <urdf/model.h>

#include <cras_cpp_common/node_utils.hpp>
#include <cras_cpp_common/param_utils.hpp>
#include <image_transport/image_transport.h>
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

class RobotModelRendererNode
{
public:
  RobotModelRendererNode(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, tf2_ros::Buffer* tf)
    : nh(nh), pnh(pnh), it(nh), tf(tf)
  {
    this->onInitialize();
  }

  virtual ~RobotModelRendererNode()
  {
    this->unsubscribe();
  }

  void reset()
  {
    this->unsubscribe();
    this->renderer->reset();
    this->renderer.reset();
    this->onInitialize();
  }

private:
  void onInitialize()
  {
    const auto publicParams = cras::nodeParams(this->nh);
    const auto params = cras::nodeParams(this->pnh);

    urdf::Model robotModel;

    while (ros::ok())
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
        CRAS_ERROR("Failed parsing robot_decription: %s", e.what());
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

    this->renderer = std::make_unique<RosCameraRobotModelRenderer>(robotModel, this->tf, config);

    this->subscribe();
  }

  void subscribe()
  {
    maskPub = it.advertise("mask", 10);
    caminfoSub = nh.subscribe("camera_info", 1, &RobotModelRendererNode::processCamInfoMessage, this);
  }

  void unsubscribe()
  {
    caminfoSub.shutdown();
    maskPub.shutdown();
  }

  void processCamInfoMessage(const sensor_msgs::CameraInfo::ConstPtr& msg)
  {
    const auto img = this->renderer->render(msg);
    if (img != nullptr)
      maskPub.publish(img);
    else
      CRAS_WARN_THROTTLE(1.0, "Robot model rendering failed");
  }

  ros::NodeHandle nh;
  ros::NodeHandle pnh;
  image_transport::ImageTransport it;
  tf2_ros::Buffer* tf;
  ros::Subscriber caminfoSub;
  image_transport::Publisher maskPub;

  std::unique_ptr<RosCameraRobotModelRenderer> renderer;
};

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_model_renderer_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  tf2_ros::Buffer tf;
  tf2_ros::TransformListener tf_listener(tf);

  robot_model_renderer::RobotModelRendererNode renderer(nh, pnh, &tf);
  ros::spin();

  return 0;
}

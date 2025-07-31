#include <memory>

#include <urdf/model.h>

#include <image_transport/image_transport.h>
#include <robot_model_renderer/RosCameraRobotModelRenderer.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

namespace robot_model_renderer
{

class RobotModelRendererNode
{
public:
  RobotModelRendererNode(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, tf2_ros::Buffer* tf)
    : nh_(nh), pnh_(pnh), it_(nh_), tf_(tf)
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
    this->renderer_->reset();
    this->renderer_.reset();
    this->onInitialize();
  }

private:
  void onInitialize()
  {
    std::string urdf_str;
    nh_.getParam("robot_description", urdf_str);
    urdf::Model descr;
    if (!descr.initString(urdf_str))
    {
      ROS_ERROR("Failed to parse URDF model");
      return;
    }

    RosCameraRobotModelRendererConfig config;

    const auto imageEncoding = pnh_.param("image_encoding", config.imageEncoding);
    if (!sensor_msgs::image_encodings::isColor(imageEncoding) && !sensor_msgs::image_encodings::isMono(imageEncoding))
    {
      ROS_FATAL("The given image encoding is not a color or mono encoding.");
      return;
    }

    config.imageEncoding = imageEncoding;
    config.visualVisible = pnh_.param("visual", config.visualVisible);
    config.collisionVisible = pnh_.param("collision", config.collisionVisible);
    config.nearClipDistance = pnh_.param("near_clip", config.nearClipDistance);
    config.farClipDistance = pnh_.param("far_clip", config.farClipDistance);
    config.doDistort = pnh_.param("do_distort", config.doDistort);
    config.gpuDistortion = pnh_.param("gpu_distortion", config.gpuDistortion);

    this->renderer_ = std::make_unique<RosCameraRobotModelRenderer>(descr, this->tf_, config);

    this->subscribe();
  }

  void subscribe()
  {
    mask_publisher_ = it_.advertise("mask", 10);
    caminfo_sub_ = nh_.subscribe("camera_info", 1, &RobotModelRendererNode::processCamInfoMessage, this);
  }

  void unsubscribe()
  {
    caminfo_sub_.shutdown();
    mask_publisher_.shutdown();
  }

  void processCamInfoMessage(const sensor_msgs::CameraInfo::ConstPtr& msg)
  {
    const auto img = this->renderer_->render(msg);
    if (img != nullptr)
      mask_publisher_.publish(img);
    else
      ROS_WARN_THROTTLE(1.0, "Robot model rendering failed");
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;
  tf2_ros::Buffer* tf_;
  ros::Subscriber caminfo_sub_;
  image_transport::Publisher mask_publisher_;

  std::unique_ptr<RosCameraRobotModelRenderer> renderer_;
};

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_model_renderer_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  tf2_ros::Buffer tf;
  tf2_ros::TransformListener tf_listener(tf);

  //create robot model
  robot_model_renderer::RobotModelRendererNode renderer(nh, pnh, &tf);
  ros::spin();

  return 0;
}

#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

class FramePublisher : public rclcpp::Node
{
public:
  FramePublisher()
  : Node("pose_tf2_frame_publisher")
  {
    // Declare and acquire `robotname` parameter
    robotname_ = this->declare_parameter<std::string>("robotname", "Wall-E");

    // Initialize the transform broadcaster
    tf_broadcaster_ =
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Subscribe to a /vrpn_mocap/robotname/pose topic and call handle_robot_pose
    // callback function on each message
    std::ostringstream stream;
    stream << "/vrpn_mocap/" << robotname_.c_str() << "/pose"; 
    std::string topic_name = stream.str();

    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        topic_name, qos,
        std::bind(&FramePublisher::handle_robot_pose, this, std::placeholders::_1));
  }

private:
  void handle_robot_pose(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg)
  {
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = robotname_.c_str();

    // Apply rotation since the y and z axis are swapped 
    // in Optitrack with respect to ROS
    t.transform.translation.x = msg->pose.position.x;
    t.transform.translation.y = msg->pose.position.z;
    t.transform.translation.z = msg->pose.position.y;

    tf2::Quaternion q;
    t.transform.rotation.x = msg->pose.orientation.x;
    t.transform.rotation.y = msg->pose.orientation.z;
    t.transform.rotation.z = msg->pose.orientation.y;
    t.transform.rotation.w = -msg->pose.orientation.w;

    // Send the transformation
    tf_broadcaster_->sendTransform(t);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string robotname_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());
  rclcpp::shutdown();
  return 0;
}
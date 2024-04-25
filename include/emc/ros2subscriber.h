#ifndef EMC_ROS2SUBSCRIBER_H_
#define EMC_ROS2SUBSCRIBER_H_

#include "rclcpp/rclcpp.hpp"

#include <string>
#include <memory>

namespace emc
{

template <typename T>
class Ros2Subscriber : public rclcpp::Node
{

public:

    Ros2Subscriber(std::string topic_name, std::string node_name, 
                    const rclcpp::QoS& qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable()) 
        : rclcpp::Node(node_name), qos_(qos)
    {
        sub_ = this->create_subscription<T>(topic_name, qos_, std::bind(&Ros2Subscriber::callback, this, std::placeholders::_1));
    };

    bool readMsg(T& msg)
    {
        if (!msg_)
            return false;
        msg = *msg_;
        msg_.reset();
        return true;
    };

private:
    typename rclcpp::Subscription<T>::SharedPtr sub_;

    typename T::SharedPtr msg_;

    rclcpp::QoS qos_;

    void callback(const typename T::SharedPtr msg){
        msg_ = msg;
    };
};

} // end namespace emc

#endif

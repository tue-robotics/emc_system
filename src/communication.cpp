#include "emc/communication.h"
#include <future> 
#include <functional>
#include <string>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace emc
{
    // Member variable declarations
    std::string laser_param_;
    std::string odom_param_;
    std::string pose_param_;        // Added missing variable
    std::string bumper_f_param_;
    std::string bumper_b_param_;
    std::string base_ref_param_;
    std::string open_door_param_;
    std::string speak_param_;
    std::string play_param_;
    std::string base_link_param_;   // Added missing variable

    class GetGlobalParam : public rclcpp::Node
    {
    public:
        GetGlobalParam() : Node("get_global_param")
        {
            parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "/global_parameter_server");
            parameters_client_->wait_for_service();

            // Fetch all required parameters
            auto parameters_future = parameters_client_->get_parameters(
                {"laser_", "odom_", "pose_", "bumper_f_", "bumper_b_", "base_ref_", "open_door_", "speak_", "play_", "base_link_"},
                std::bind(&GetGlobalParam::callbackGlobalParam, this, std::placeholders::_1));
        }

        void callbackGlobalParam(std::shared_future<std::vector<rclcpp::Parameter>> future)
        {
            auto result = future.get();
            for (const auto &param : result)
            {
                const std::string &param_name = param.get_name();
                const std::string &param_value = param.as_string();

                if (param_name == "laser_")
                    laser_param_ = param_value;
                else if (param_name == "odom_")
                    odom_param_ = param_value;
                else if (param_name == "pose_")
                    pose_param_ = param_value;
                else if (param_name == "bumper_f_")
                    bumper_f_param_ = param_value;
                else if (param_name == "bumper_b_")
                    bumper_b_param_ = param_value;
                else if (param_name == "base_ref_")
                    base_ref_param_ = param_value;
                else if (param_name == "open_door_")
                    open_door_param_ = param_value;
                else if (param_name == "speak_")
                    speak_param_ = param_value;
                else if (param_name == "play_")
                    play_param_ = param_value;
                else if (param_name == "base_link_")
                    base_link_param_ = param_value;

                // Log the retrieved parameter
                std::cout << "Retrieved parameter: " << param_name << " = " << param_value << std::endl;
            }

            std::cout << "All parameters retrieved successfully." << std::endl;
        }

    private:
        std::shared_ptr<rclcpp::AsyncParametersClient> parameters_client_;
    };


    Communication::Communication(std::string /*robot_name*/)
    {
        rclcpp::init(0, nullptr);

        std::cout << "Creating node for parameter retrieval..." << std::endl;
        auto node = std::make_shared<rclcpp::Node>("get_global_param");
        auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(node, "/global_parameter_server");

        std::cout << "Waiting for parameter server to be available..." << std::endl;
        if (!parameters_client->wait_for_service(std::chrono::seconds(5)))
        {
            throw std::runtime_error("Parameter server not available.");
        }
        std::cout << "Parameter server is available." << std::endl;

        std::cout << "Fetching parameters asynchronously..." << std::endl;
        parameters_client->get_parameters(
            {"laser_", "odom_", "pose_", "bumper_f_", "bumper_b_", "base_ref_", "open_door_", "speak_", "play_", "base_link_"},
            [this](std::shared_future<std::vector<rclcpp::Parameter>> future) {
                auto result = future.get();
                for (const auto &param : result)
                {
                    const std::string &param_name = param.get_name();
                    const std::string &param_value = param.as_string();

                    if (param_name == "laser_")
                        laser_param_ = param_value;
                    else if (param_name == "odom_")
                        odom_param_ = param_value;
                    else if (param_name == "pose_")
                        pose_param_ = param_value;
                    else if (param_name == "bumper_f_")
                        bumper_f_param_ = param_value;
                    else if (param_name == "bumper_b_")
                        bumper_b_param_ = param_value;
                    else if (param_name == "base_ref_")
                        base_ref_param_ = param_value;
                    else if (param_name == "open_door_")
                        open_door_param_ = param_value;
                    else if (param_name == "speak_")
                        speak_param_ = param_value;
                    else if (param_name == "play_")
                        play_param_ = param_value;
                    else if (param_name == "base_link_")
                        base_link_param_ = param_value;

                    // Log the retrieved parameter
                    std::cout << "Retrieved parameter: " << param_name << " = " << param_value << std::endl;
                }

                std::cout << "All parameters retrieved successfully." << std::endl;

                // Initialize subscribers and publishers after parameters are retrieved
                laser_node_ = std::make_shared<emc::Ros2Subscriber<sensor_msgs::msg::LaserScan>>(laser_param_, "emc_laser");
                laser_executor_ = new rclcpp::executors::SingleThreadedExecutor;
                laser_executor_->add_node(laser_node_);
                std::cout << "Laser subscriber created." << std::endl;

                odom_node_ = std::make_shared<emc::Ros2Subscriber<nav_msgs::msg::Odometry>>(odom_param_, "emc_odom");
                odom_executor_ = new rclcpp::executors::SingleThreadedExecutor;
                odom_executor_->add_node(odom_node_);
                std::cout << "Odom subscriber created." << std::endl;

                pub_node_ = new Ros2Publisher(base_ref_param_, open_door_param_, speak_param_, play_param_);
                std::cout << "Publishers created." << std::endl;
            });

        // Spin the node to process the callback
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        executor.spin_some(); // Process callbacks once
    }

    Communication::~Communication()
    {
    }

    void Communication::init()
    {
    }

    bool Communication::readLaserData(LaserData &scan)
    {
        laser_executor_->spin_once(std::chrono::nanoseconds(0)); // wait 0 nanoseconds for new messages. just empty the buffer.

        sensor_msgs::msg::LaserScan msg;
        if (!laser_node_->readMsg(msg))
            return false;

        scan.range_min = msg.range_min;
        scan.range_max = msg.range_max;
        scan.ranges = msg.ranges;
        scan.angle_min = msg.angle_min;
        scan.angle_max = msg.angle_max;
        scan.angle_increment = msg.angle_increment;
        scan.timestamp = rclcpp::Time(msg.header.stamp).seconds();
        return true;
    }

    bool Communication::readOdometryData(OdometryData &odom)
    {
        odom_executor_->spin_once(std::chrono::nanoseconds(0)); // wait 0 nanoseconds for new messages. just empty the buffer.

        nav_msgs::msg::Odometry msg;
        if (!odom_node_->readMsg(msg))
            return false;

        odom.x = msg.pose.pose.position.x;
        odom.y = msg.pose.pose.position.y;

        // Calculate yaw rotation from quaternion
        const geometry_msgs::msg::Quaternion &q = msg.pose.pose.orientation;
        odom.a = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));

        odom.timestamp = rclcpp::Time(msg.header.stamp).seconds();

        return true;
    }

    bool Communication::readFrontBumperData(BumperData & /*bumper*/)
    {
        return false;
        /*
        bumper_f_msg_.reset();

        //bumper_f_cb_queue_.callAvailable();

        if (!bumper_f_msg_)
            return false;

        bumper.contact = bumper_f_msg_->data;
        return true;
        */
    }

    bool Communication::readBackBumperData(BumperData & /*bumper*/)
    {
        return false;
        /*
        bumper_b_msg_.reset();

        //bumper_b_cb_queue_.callAvailable();

        if (!bumper_b_msg_)
            return false;

        bumper.contact = bumper_b_msg_->data;
        return true;
        */
    }

    void Communication::sendBaseVelocity(double vx, double vy, double va)
    {
        pub_node_->sendBaseVelocity(vx, vy, va);
    }

    void Communication::sendOpenDoorRequest()
    {
        pub_node_->sendOpenDoorRequest();
    }

    void Communication::sendMarker(visualization_msgs::msg::Marker marker)
    {
        pub_node_->sendMarker(marker);
    }

    void Communication::speak(const std::string &text)
    {
        pub_node_->speak(text);
    }

    void Communication::play(const std::string &file)
    {
        pub_node_->play(file);
    }

    void Communication::sendPoseEstimate(const geometry_msgs::msg::Transform &pose)
    {
        pub_node_->sendPoseEstimate(pose);
    }

    // publishers used to visualize information in the localization exercises (particle filter):

    void Communication::localization_viz_send_laser_scan(double angle_min, double angle_max, double angle_inc, int subsample, std::vector<float> prediction)
    {
        pub_node_->localization_viz_send_laser_scan(angle_min, angle_max, angle_inc, subsample, prediction);
    }

    void Communication::localization_viz_send_particles(int N, std::vector<std::vector<double>> particle_poses, double mapOrientation)
    {
        pub_node_->localization_viz_send_particles(N, particle_poses, mapOrientation);
    }

    void Communication::localization_viz_send_pose(std::vector<double> pose, double mapOrientation)
    {
        pub_node_->localization_viz_send_pose(pose, mapOrientation);
    }

} // end namespace emc
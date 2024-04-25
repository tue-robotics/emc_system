#ifndef EMC_ROS2PUBLISHER_H_
#define EMC_ROS2PUBLISHER_H_

#include "rclcpp/rclcpp.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include "tf2/LinearMath/Quaternion.h"

#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace emc
{

    /**
     * collection of all publishers used in the emc system
     */
    class Ros2Publisher : public rclcpp::Node
    {

    public:
        Ros2Publisher() : rclcpp::Node("emc_publishers")
        {
            std::string laser_param, odom_param, bumper_f_param, bumper_b_param, base_ref_param, open_door_param, speak_param, play_param;
            laser_param = "laser_scan";
            odom_param = "odom";
            bumper_f_param = "bumper_f";
            bumper_b_param = "bumper_b";
            base_ref_param = "cmd_vel";
            open_door_param = "open_door";
            speak_param = "speak";
            play_param = "play";

            pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(base_ref_param, 10);
            pub_open_door_ = this->create_publisher<std_msgs::msg::Empty>(open_door_param, 10);
            pub_speak_ = this->create_publisher<std_msgs::msg::String>(speak_param, 10);
            pub_play_ = this->create_publisher<std_msgs::msg::String>(play_param, 10);
            pub_marker_ = this->create_publisher<visualization_msgs::msg::Marker>("/marker", 10);

            // publishers used to visualize information in the localization exercises (particle filter):
            this->pub_laser_msg = this->create_publisher<sensor_msgs::msg::LaserScan>("/laser_match", 1);
            this->pub_particle = this->create_publisher<geometry_msgs::msg::PoseArray>("/particles", 1);
            this->pub_pose = this->create_publisher<geometry_msgs::msg::PoseArray>("/pose_estimate", 1);

            pub_tf2_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        };

        void sendBaseVelocity(double vx, double vy, double va)
        {
            geometry_msgs::msg::Twist ref;
            ref.linear.x = vx;
            ref.linear.y = vy;
            ref.angular.z = va;

            pub_cmd_vel_->publish(ref);
        };

        void sendOpenDoorRequest()
        {
            std_msgs::msg::Empty msg;
            pub_open_door_->publish(msg);
        };

        void sendMarker(visualization_msgs::msg::Marker marker)
        {
            pub_marker_->publish(marker);
        };

        void speak(const std::string &text)
        {
            std_msgs::msg::String str;
            str.data = text;
            pub_speak_->publish(str);
        };

        void play(const std::string &file)
        {
            std_msgs::msg::String str;
            str.data = file;
            pub_play_->publish(str);
        };

        // Postion data
        void sendPoseEstimate(const geometry_msgs::msg::Transform &pose)
        {
            // Publish tf transform
            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped.header.stamp = rclcpp::Clock{RCL_ROS_TIME}.now(); // rclcpp::Time::now();
            transformStamped.header.frame_id = "map";
            transformStamped.child_frame_id = robot_frame_name;
            transformStamped.transform = pose;
            pub_tf2_->sendTransform(transformStamped);
        };

        bool sendPath(std::vector<std::vector<double>> path, std::array<double, 3> color, double width, int id)
        {
            visualization_msgs::msg::Marker pathMarker;
            pathMarker.header.frame_id = "map";
            pathMarker.header.stamp = rclcpp::Clock{RCL_ROS_TIME}.now();
            pathMarker.ns = "path";
            pathMarker.id = id;
            pathMarker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            pathMarker.action = visualization_msgs::msg::Marker::ADD;
            pathMarker.color.a = 1.0;
            pathMarker.color.r = color[0];
            pathMarker.color.g = color[1];
            pathMarker.color.b = color[2];
            pathMarker.pose.orientation.w = 1.0;
            pathMarker.scale.x = width;
            for (std::vector<std::vector<double>>::iterator it = path.begin(); it != path.end(); ++it)
            {
                geometry_msgs::msg::Point point;
                if ((*it).size() < 2)
                {
                    RCLCPP_WARN_STREAM(this->get_logger(), "Point at index " << std::distance(path.begin(), it) << " has too few dimensions (expected at least 2, got " << (*it).size() << "), skipping.");
                    continue;
                }
                point.x = (*it)[0];
                point.y = (*it)[1];
                if ((*it).size() == 2)
                {
                    point.z = 0;
                }
                else
                {
                    point.z = (*it)[3];
                    if ((*it).size() > 3)
                    {
                        RCLCPP_WARN_STREAM(this->get_logger(), "point at index " << std::distance(path.begin(), it) << " has unused dimensions (expected 2 or 3, got " << (*it).size() << ").");
                    }
                }
                pathMarker.points.push_back(point);
            }
            if (pathMarker.points.size() < 2)
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Not enough valid points (expected at least 2, got " << pathMarker.points.size() << ").");
                return false;
            }
            sendMarker(pathMarker);
            return true;
        };

        // publishers used to visualize information in the localization exercises (particle filter):

        void localization_viz_send_laser_scan(double angle_min, double angle_max, double angle_inc, int subsample, std::vector<float> prediction)
        {
            sensor_msgs::msg::LaserScan msg{};

            msg.angle_min = angle_min;
            msg.angle_max = angle_max;
            msg.angle_increment = angle_inc * subsample;

            msg.range_min = 0.01;
            msg.range_max = 10;

            msg.header.frame_id = "internal/base_link";
            msg.header.stamp = this->now();

            msg.ranges = prediction;

            this->pub_laser_msg->publish(msg);
        };

        void localization_viz_send_particles(int N, std::vector<std::vector<double>> particle_poses, double mapOrientation)
        {
            geometry_msgs::msg::PoseArray msg{};

            msg.header.frame_id = "map";
            msg.header.stamp = this->now();
            tf2::Quaternion a;
            msg.poses.reserve(N);
            for (int i = 0; i < N; i++)
            {
                geometry_msgs::msg::Pose posemsg;
                auto pose_i = particle_poses[i];

                posemsg.position.x = std::cos(mapOrientation) * pose_i[0] - std::sin(mapOrientation) * pose_i[1];
                posemsg.position.y = std::sin(mapOrientation) * pose_i[0] + std::cos(mapOrientation) * pose_i[1];
                posemsg.position.z = 0;

                a.setRPY(0, 0, pose_i[2] - mapOrientation);
                posemsg.orientation.w = a.getW();
                posemsg.orientation.x = a.getX();
                posemsg.orientation.y = a.getY();
                posemsg.orientation.z = a.getZ();

                msg.poses.push_back(posemsg);
            }

            this->pub_particle->publish(msg);
        };


        void localization_viz_send_pose(std::vector<double> pose, double mapOrientation)
        {
            geometry_msgs::msg::PoseArray msg;

            msg.header.frame_id = "map";
            msg.header.stamp = this->now();
            tf2::Quaternion a;

            msg.poses.reserve(1);

            geometry_msgs::msg::Pose posemsg;
            posemsg.position.x = std::cos(mapOrientation) * pose[0] - std::sin(mapOrientation) * pose[1];
            posemsg.position.y = std::sin(mapOrientation) * pose[0] + std::cos(mapOrientation) * pose[1];
            posemsg.position.z = 0;

            a.setRPY(0, 0, pose[2] - mapOrientation);
            posemsg.orientation.w = a.getW();
            posemsg.orientation.x = a.getX();
            posemsg.orientation.y = a.getY();
            posemsg.orientation.z = a.getZ();

            msg.poses.push_back(posemsg);

            this->pub_pose->publish(msg);
        };

    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_open_door_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_speak_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_play_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;

        // pose publishing
        std::string robot_frame_name;
        std::unique_ptr<tf2_ros::TransformBroadcaster> pub_tf2_;

        // publishers used to visualize information in the localization exercises (particle filter):
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_laser_msg;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_particle;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_pose;
    };

} // end namespace emc

#endif

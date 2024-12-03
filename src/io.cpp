#include "emc/io.h"

#include "emc/communication.h"

#include "rclcpp/rclcpp.hpp" // for rclcpp::ok()

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <string>

namespace emc
{

    IO::IO(Communication *comm) : comm_(comm)
    {
        std::cout << "constructor of io" << std::endl;
        if (!comm_)
        {
            std::cout << "constructing comm_" << std::endl;
            comm_ = new Communication;
        }
    }

    IO::IO(std::string robot_name)
    {
        comm_ = new Communication(robot_name);
    }

    IO::~IO()
    {
        if (comm_)
            delete comm_;
    }

    bool IO::readLaserData(LaserData &scan)
    {
        return comm_->readLaserData(scan);
    }

    bool IO::readOdometryData(OdometryData &odom)
    {
        OdometryData new_odom;
        if (!comm_->readOdometryData(new_odom))
            return false;

        if (!odom_set_)
        {
            prev_odom_ = new_odom;
            odom_set_ = true;
            return false;
        }

        odom.timestamp = new_odom.timestamp; // TODO give dt?
        double dx = new_odom.x - prev_odom_.x;
        double dy = new_odom.y - prev_odom_.y;
        odom.x = cos(prev_odom_.a) * dx + sin(prev_odom_.a) * dy;
        odom.y = -sin(prev_odom_.a) * dx + cos(prev_odom_.a) * dy;
        odom.a = fmod(new_odom.a - prev_odom_.a + M_PI, 2 * M_PI) - M_PI;

        prev_odom_ = new_odom;
        return true;
    }

    bool IO::resetOdometry()
    {
        OdometryData new_odom;
        if (!comm_->readOdometryData(new_odom))
            return false;
        prev_odom_ = new_odom;
        odom_set_ = true;
        return true;
    }

    bool IO::readFrontBumperData(BumperData &bumper)
    {
        return comm_->readFrontBumperData(bumper);
    }

    bool IO::readBackBumperData(BumperData &bumper)
    {
        return comm_->readBackBumperData(bumper);
    }

    void IO::sendBaseReference(double vx, double vy, double va)
    {
        comm_->sendBaseVelocity(vx, vy, va);
    }

    void IO::sendOpendoorRequest()
    {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"
        system("aplay --device front:CARD=Device_1,DEV=0 ~/.emc/system/src/emc_system/sounds/doorbell.wav &");
#pragma GCC diagnostic pop

        comm_->sendOpenDoorRequest();
    }

    void IO::speak(const std::string &text)
    {
        comm_->speak(text);
    }

    void IO::play(const std::string &file)
    {
        comm_->play(file);
    }

    bool IO::ok()
    {
        return rclcpp::ok();
    }

    bool IO::sendPath(std::vector<std::vector<double>> path, std::array<double, 3> color, double width, int id)
    {
        return comm_->sendPath(path, color, width, id);
    }

    bool IO::sendPoseEstimate(double px, double py, double pz, double rx, double ry, double rz, double rw)
    {
        // apply map offset and send to comm_
        tf2::Transform pose;
        pose.setOrigin(tf2::Vector3(px, py, pz));
        pose.setRotation(tf2::Quaternion(rx, ry, rz, rw));

        comm_->sendPoseEstimate(tf2::toMsg(pose));
        return true;
    }

    bool IO::sendPoseEstimate(double px, double py, double pz, double roll, double pitch, double yaw)
    {
        // convert roll pitch yaw to quaternion
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        return this->sendPoseEstimate(px, py, pz, q.x(), q.y(), q.z(), q.w());
    }

    bool IO::sendPoseEstimate(double x, double y, double yaw)
    {
        return this->sendPoseEstimate(x, y, 0, 0, 0, yaw);
    }

    // publishers used to visualize information in the localization exercises (particle filter):

    void IO::localization_viz_send_laser_scan(double angle_min, double angle_max, double angle_inc, int subsample, std::vector<float> prediction)
    {
        comm_->localization_viz_send_laser_scan(angle_min, angle_max, angle_inc, subsample, prediction);
    }

    void IO::localization_viz_send_particles(int N, std::vector<std::vector<double>> particle_poses, double mapOrientation)
    {
        comm_->localization_viz_send_particles(N, particle_poses, mapOrientation);
    }

    void IO::localization_viz_send_pose(std::vector<double> pose, double mapOrientation)
    {
        comm_->localization_viz_send_pose(pose, mapOrientation);
    }

}

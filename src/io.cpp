#include "emc/io.h"

#include "emc/communication.h"

#include "rclcpp/rclcpp.hpp" // for rclcpp::ok()

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include <string>

namespace emc
{

IO::IO(Communication* comm) : comm_(comm)
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

bool IO::readLaserData(LaserData& scan)
{
    return comm_->readLaserData(scan);
}

bool IO::readOdometryData(OdometryData& odom)
{
    return comm_->readOdometryData(odom);
}  

bool IO::readFrontBumperData(BumperData& bumper)
{
    return comm_->readFrontBumperData(bumper);
}

bool IO::readBackBumperData(BumperData& bumper)
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

void IO::speak(const std::string& text)
{
    comm_->speak(text);
}

void IO::play(const std::string& file)
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
    //convert roll pitch yaw to quaternion
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return this->sendPoseEstimate(px, py, pz, q.x(), q.y(), q.z(), q.w());
}

bool IO::sendPoseEstimate(double x, double y, double yaw)
{
    return this->sendPoseEstimate(x, y, 0, 0, 0, yaw);
}

}

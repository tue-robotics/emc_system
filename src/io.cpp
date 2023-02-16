#include "emc/io.h"

#include "emc/communication.h"

#include <ros/init.h>  // for ros::ok()
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

#include <string>

namespace emc
{

IO::IO(Communication* comm) : comm_(comm)
{
    if (!comm_)
        comm_ = new Communication;
}

IO::IO(std::string robot_name) : comm_(new Communication(robot_name))
{
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

/*
bool IO::readControlEffort(ControlEffort& ce)
{
    return comm_->readControlEffort(ce);
}
*/

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
     
    comm_->sendOpendoorRequest();
}

void IO::speak(const std::string& text)
{
    //std::string cmd;
    //cmd = "sudo espeak " + text + " --stdout | sudo aplay --device \"default:CARD=Device\"";
    //cmd = "espeak " + text + " --stdout | aplay --device \"default:CARD=Device\" &";
    //system(cmd.c_str());

    comm_->speak(text);
}

void IO::play(const std::string& file)
{
    comm_->play(file);
}

bool IO::ok()
{
    return ros::ok();
}

void IO::sendPoseEstimate(double& px, double& py, double& pz, double& rx, double& ry, double& rz, double& rw)
{
    geometry_msgs::Transform pose;
    pose.translation.x = px;
    pose.translation.y = py;
    pose.translation.z = pz;
    pose.rotation.x = rx;
    pose.rotation.y = ry;
    pose.rotation.z = rz;
    pose.rotation.w = rw;

}

void IO::sendPoseEstimate(double& px, double& py, double& pz, double& rr, double& rp, double& ry)
{
    tf2::quaternion q;
    q.setRPY(rr, rp, ry);
    this->sendPoseEstimate(px, py, pz, q.x(), q.y(), q.z(), q.w())
}

void IO::sendPoseEstimate(geo::Pose3D pose)
{
    double px, py, pz, rx, ry, rz, rw;

}

}

#include "emc/io.h"

#include "emc/communication.h"

#include <ros/init.h>  // for ros::ok()
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/Marker.h>

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
    OdometryData new_odom;
    bool newdata = comm_->readOdometryData(new_odom);
    if (!newdata)
        return false;

    if (!odom_set)
    {
        ROS_WARN("Odom was not yet set. It is set now.");
        prev_odom = new_odom;
        odom_set = true;
        return false;
    }

    odom.timestamp = new_odom.timestamp; //TODO give dt?
    double dx = new_odom.x - prev_odom.x;
    double dy = new_odom.y - prev_odom.y;
    odom.x = cos(prev_odom.a) * dx + sin(prev_odom.a) * dy;
    odom.y = -sin(prev_odom.a) * dx + cos(prev_odom.a) * dy;
    odom.a = fmod(new_odom.a - prev_odom.a + M_PI, 2*M_PI) - M_PI;

    prev_odom = new_odom;
    return true;
}  

bool IO::resetOdometry()
{
    OdometryData new_odom;
    bool newdata = comm_->readOdometryData(new_odom);
    if (!newdata)
        return false;
    prev_odom = new_odom;
    odom_set = true;
    return true;
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

bool IO::sendPath(std::vector<std::vector<double>> path, std::array<double, 3> color, double width, int id)
{
    visualization_msgs::Marker pathMarker;
    pathMarker.header.frame_id = "map";
    pathMarker.header.stamp = ros::Time();
    pathMarker.ns = "path";
    pathMarker.id = id;
    pathMarker.type = visualization_msgs::Marker::LINE_STRIP;
    pathMarker.action = visualization_msgs::Marker::ADD;
    pathMarker.color.a = 1.0;
    pathMarker.color.r = color[0];
    pathMarker.color.g = color[1];
    pathMarker.color.b = color[2];
    pathMarker.pose.orientation.w = 1.0;
    pathMarker.scale.x = width;
    for (std::vector<std::vector<double>>::iterator it = path.begin(); it != path.end(); ++it)
    {
        geometry_msgs::Point point;
        if ((*it).size() < 2)
        {
            ROS_WARN_STREAM("Point at index " << std::distance(path.begin(), it) << " has too few dimensions (expected at least 2, got " << (*it).size() << "), skipping.");
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
                ROS_WARN_STREAM("point at index " << std::distance(path.begin(), it) << " has unused dimensions (expected 2 or 3, got " << (*it).size() << ").");
            }
        }
        pathMarker.points.push_back(point);
    }
    if (pathMarker.points.size() < 2)
    {
        ROS_ERROR_STREAM("Not enough valid points (expected at least 2, got " << pathMarker.points.size() << ").");
        return false;
    }
    comm_->sendMarker(pathMarker);
    return true;
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

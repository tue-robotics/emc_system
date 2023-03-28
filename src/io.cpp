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

bool IO::sendPath(std::vector<std::vector<double>> path, std::vector<double> color, int id)
{
    if (color.size() < 3)
    {
        color = {0.0, 0.0, 0.0};
        ROS_WARN_STREAM("invalid color");
    }
    MapConfig mapData;
    comm_->getMapConfig(mapData);
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
    pathMarker.scale.x = 0.01;
    for (std::vector<std::vector<double>>::iterator it = path.begin(); it != path.end(); ++it)
    {
        geometry_msgs::Point point;
        if ((*it).size() < 2)
        {
            ROS_WARN_STREAM("point at index " << std::distance(path.begin(), it) << " is invalid, skipping");;
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
                ROS_WARN_STREAM("point at index " << std::distance(path.begin(), it) << " has unused dimensions");
            }
        }

        if (mapData.mapInitialised)
        {
            tf2::Transform tfPoint;
            tfPoint.setOrigin(tf2::Vector3(point.x, point.y, point.z));
            tfPoint.setRotation(tf2::Quaternion(0, 0, 0, 1));
            tf2::Transform mapOffset;
            mapOffset.setOrigin(tf2::Vector3(mapData.mapOffsetX, mapData.mapOffsetY, 0));
            tf2::Quaternion q;
            q.setRPY(0, 0, mapData.mapOrientation);
            mapOffset.setRotation(q);

            tfPoint = mapOffset * tfPoint;
            point.x = tfPoint.getOrigin().x();
            point.y = tfPoint.getOrigin().y();
            point.z = tfPoint.getOrigin().z();
        }

        pathMarker.points.push_back(point);
    }
    if (pathMarker.points.size() < 2)
    {
        ROS_ERROR_STREAM("not enough valid points");
        return false;
    }
    comm_->sendMarker(pathMarker);
    return true;
}

void IO::sendPoseEstimate(double px, double py, double pz, double rx, double ry, double rz, double rw)
{
    // apply map offset and send to comm_
    tf2::Transform pose;
    MapConfig mapData;
    pose.setOrigin(tf2::Vector3(px, py, pz));
    pose.setRotation(tf2::Quaternion(rx, ry, rz, rw));

    if (comm_->getMapConfig(mapData))
    {
        tf2::Transform mapOffset;
        mapOffset.setOrigin(tf2::Vector3(mapData.mapOffsetX, mapData.mapOffsetY, 0));
        tf2::Quaternion q;
        q.setRPY(0, 0, mapData.mapOrientation);
        mapOffset.setRotation(q);

        pose = mapOffset * pose;
    }
    else
    {
        ROS_WARN_STREAM("No map data supplied");
    }

    comm_->sendPoseEstimate(tf2::toMsg(pose));

}

void IO::sendPoseEstimate(double px, double py, double pz, double roll, double pitch, double yaw)
{
    //convert roll pitch yaw to quaternion
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    this->sendPoseEstimate(px, py, pz, q.x(), q.y(), q.z(), q.w());
}

}

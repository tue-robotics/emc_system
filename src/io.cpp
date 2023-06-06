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

bool IO::sendPath(std::vector<std::vector<double>> path, std::array<double, 3> color, double width, int id)
{
    MapConfig mapData;
    if (!comm_->getMapConfig(mapData))
    {
        ROS_ERROR_STREAM("No map data supplied");
        return false;
    }
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
            ROS_WARN_STREAM("Point at index " << std::distance(path.begin(), it) << " has too few dimensions (expected 2, got " << (*it).size() << "), skipping.");
            continue;
        }
        point.x = (*it)[0];
        point.y = (*it)[1];
        point.z = 0;
        if ((*it).size() > 2)
        {
            ROS_WARN_STREAM("point at index " << std::distance(path.begin(), it) << " has unused dimensions (expected 2, got " << (*it).size() << ").");
        }

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

bool IO::sendPoints(std::vector<std::vector<double>> points, std::array<double, 3> color, double width, int id)
{
    MapConfig mapData;
    if (!comm_->getMapConfig(mapData))
    {
        ROS_ERROR_STREAM("No map data supplied");
        return false;
    }
    visualization_msgs::Marker PointMarker;
    PointMarker.header.frame_id = "map";
    PointMarker.header.stamp = ros::Time();
    PointMarker.ns = "points";
    PointMarker.id = id;
    PointMarker.type = visualization_msgs::Marker::SPHERE_LIST;
    PointMarker.action = visualization_msgs::Marker::ADD;
    PointMarker.color.a = 1.0;
    PointMarker.color.r = color[0];
    PointMarker.color.g = color[1];
    PointMarker.color.b = color[2];
    PointMarker.pose.orientation.w = 1.0;
    PointMarker.scale.x = width;
    PointMarker.scale.y = width;
    PointMarker.scale.z = width;
    for (std::vector<std::vector<double>>::iterator it = points.begin(); it != points.end(); ++it)
    {
        geometry_msgs::Point point;
        
        if ((*it).size() < 2)
        {
            ROS_WARN_STREAM("Point at index " << std::distance(points.begin(), it) << " has too few dimensions (expected 2, got " << (*it).size() << "), skipping.");
            continue;
        }
        point.x = (*it)[0];
        point.y = (*it)[1];
        point.z = 0;
        if ((*it).size() > 2)
        {
            ROS_WARN_STREAM("point at index " << std::distance(points.begin(), it) << " has unused dimensions (expected 2, got " << (*it).size() << ").");
        }

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
        
        PointMarker.points.push_back(point);
    }
    if (PointMarker.points.size() < 1)
    {
        ROS_ERROR_STREAM("No valid points");
        return false;
    }
    comm_->sendMarker(PointMarker);
    return true;
}

bool IO::sendPoseEstimate(double px, double py, double pz, double rx, double ry, double rz, double rw)
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
        ROS_ERROR_STREAM("No map data supplied");
        return false;
    }

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

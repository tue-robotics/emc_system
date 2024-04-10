#ifndef EMC_SYSTEM_IO_H_
#define EMC_SYSTEM_IO_H_

#include <array>
#include <vector>
#include <string>

#include "emc/data.h"
#include "emc/odom.h"
#include "emc/pose.h"
#include "emc/bumper.h"

namespace emc
{

class Communication;

class IO
{

public:

    IO(Communication* comm = nullptr);
    IO(std::string robot_name);

    ~IO();

    /**
     * @brief Receive new laser data if available
     * 
     * @param scan reference to a LaserData object to write the new data to
     * @return true if new laser data was available
     * @return false if not
     */
    bool readLaserData(LaserData& scan);

    /**
     * @brief Receive new pose data if available
     * 
     * @param pose reference to a PoseData object to write the new data to
     * @return true if new pose data was available
     * @return false if not
     */
    bool readPoseData(PoseData& pose);

    /**
     * @brief Receive new odometrydata if available
     * 
     * @param[out] odom reference to an OdometryData object to write the new data to. provides displacement since
     *  last time this function or resetOdometry() was called.
     * @return true if new data was available
     * @return false if not
     */
    bool readOdometryData(OdometryData& odom);

    /**
     * @brief Set the current position as (0,0,0)
     * 
     * @return true if new data was available to reset, false if not
    */
    bool resetOdometry();

    /**
     * @brief Receive new BumperData from the front bumper
     * 
     * @param bumper Bumberdata object to write the new data to, untouched if no data is available
     * @return true if new data was available
     * @return false if not
     */
    bool readFrontBumperData(BumperData& bumper);

    /**
     * @brief Receive new BumperData from the rear bumper
     * 
     * @param bumper Bumberdata object to write the new data to, untouched if no data is available
     * @return true if new data was available
     * @return false if not
     */
    bool readBackBumperData(BumperData& bumper);

    /**
     * @brief Send a command velocity to the robot. 
     * 
     * @param vx [m/s] Desired velocity in x direction (forward)
     * @param vy [m/s] Desired velocity in y direction (left)
     * @param va [rad/s] Desired rotational velocity. Turning to the left is positive
     */
    void sendBaseReference(double vx, double vy, double va);

    /**
     * @brief Broadcast a request to open nearby doors. Works in simulator only.
     * 
     */
    void sendOpendoorRequest();

    /**
     * @brief Check connection to the robot
     * 
     * @return true Connected to the robot
     * @return false Connection to the robot lost
     */
    bool ok();

    /**
     * @brief Make the robot speak the input text. This function is non-blocking.
     * 
     * @param text Sentence to be spoken by the robot.
     */
    void speak(const std::string& text);
    
    /**
     * @brief Play a sound file out loud. The file must be placed in the ~/MRC_audio folder and must be of .mp3 or .wav format.
     * 
     * @param file filename, including extension
     */
    void play(const std::string& file);

    /**
     * @brief Send a path to be drawn in rviz.
     * 
     * @param path The sequence of points, between which the path will be drawn. The sequence must be provided as a vector of points, where each point is formatted as {x,y,z} or {x,y}. Coordinates are relative to the centre of the map (same as simulator coordinates).
     * @param color The color in which the path will be drawn. Format is {r,g,b}, with each value between 0.0 and 1.0.
     * @param width [m] Line width.
     * @param id The id of this path. When drawing multiple paths, each sequence must have a unique id. Sending a new path with the same id will overwrite the previous path.
     * 
     * @return true Path is sent to rviz.
     * @return false Path does not have enough valid points.
     */
    bool sendPath(std::vector<std::vector<double>> path, std::array<double, 3> color = {0.0, 0.0, 0.0}, double width = 0.02, int id = 0);


    /**
     * @brief Send an estimate of the current robot pose to be shown in rviz, relative to the centre of the map (same as simulator coordinates).
     * 
     * @param x The x coordinate
     * @param y The y coordinate
     * @param yaw The yaw rotation
    */
    bool sendPoseEstimate(double x, double y, double yaw);

private:

    /**
     * @brief Send an estimate of the current robot pose to be shown in rviz. Rotation is provided as a quaternion.
     * 
     * @param px The x coordinate of the position.
     * @param py The y coordinate of the position.
     * @param pz The z coordinate of the position.
     * @param rx The x component of the rotation.
     * @param ry The y component of the rotation.
     * @param rz The z component of the rotation.
     * @param rw The w component of the rotation.
     */
    bool sendPoseEstimate(double px, double py, double pz, double rx, double ry, double rz, double rw); //use quaternion

    /**
     * @brief Send an estimate of the current robot pose to be shown in rviz. Rotation is provided as roll pitch and yaw.
     * 
     * @param px The x coordinate of the position.
     * @param py The y coordinate of the position.
     * @param pz The z coordinate of the position.
     * @param rr The roll component of the rotation.
     * @param rp The pitch component of the rotation.
     * @param ry The yaw component of the rotation.
     */
    bool sendPoseEstimate(double px, double py, double pz, double rr, double rp, double ry); //use roll pitch yaw

    // odometry memory
    OdometryData prev_odom_;
    bool odom_set_ = false; // wether odom has been set at least once.

    Communication* comm_;

};

} // end namespace emc

#endif

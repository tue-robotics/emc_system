#ifndef EMC_SYSTEM_IO_H_
#define EMC_SYSTEM_IO_H_

#include <vector>
#include <string>

#include "emc/data.h"
#include "emc/odom.h"
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
     * @brief Receive new odometrydata if available
     * 
     * @param odom reference to an OdometryData object to write the new data to
     * @return true if new data was available
     * @return false if not
     */
    bool readOdometryData(OdometryData& odom);

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

private:

    Communication* comm_;

};

} // end namespace emc

#endif

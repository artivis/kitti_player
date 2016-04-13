#ifndef _KITTI_PLAYER_SUPPORT_H_
#define _KITTI_PLAYER_SUPPORT_H_

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

#include <string>
#include <vector>

namespace PlayerSupport
{

// Read camera calibration values and return by argument
int getCamCalibration(std::string dir_root, std::string camera_name, double* K, std::vector<double> & D, double *R, double* P);

/** Get sensors static transforms from kitti calibration files.
 * If camera_name is informed, we look for T_XX and R_XX tokens
 */
int getStaticTransform(std::string calib_filename, geometry_msgs::TransformStamped *ros_msgTf, std_msgs::Header *header, std::string camera_name = "");

/// Get timestamp fom the file at the entry_number point
bool getTimestamp(std::string dir_timestamp, unsigned int entry_number, ros::Time &stamp);

// Get the timestamp string and convert to ros::Time
ros::Time parseTime(std::string timestamp);

int getGPS(std::string filename, sensor_msgs::NavSatFix *ros_msgGpsFix, std_msgs::Header *header);

int getIMU(std::string filename, sensor_msgs::Imu *ros_msgImu, std_msgs::Header *header);

int getOdomTf(std::string filename, tf2::Transform *tf2_tf);

}

#endif // _KITTI_PLAYER_SUPPORT_H_

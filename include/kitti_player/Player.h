#ifndef _KITTI_PLAYER_H_
#define _KITTI_PLAYER_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>

#include <rosgraph_msgs/Clock.h>

#include <boost/thread.hpp>
#if BOOST_VERSION >= 104100
  #include <boost/thread/future.hpp>
#endif // BOOST_VERSION >=  104100

#if !defined(_MSC_VER)
  #include <sys/select.h>
  #include <termios.h>
  #include <unistd.h>
#endif

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <string>

struct PlayerOptions
{
    std::string  path;
    float   frequency;      // publisher frequency. 1 > Kitti default 10Hz
    bool    all_data;       // publish everything
    bool    velodyne;       // publish velodyne point clouds /as PCL
    bool    gps;            // publish GPS sensor_msgs/NavSatFix    message
    bool    imu;            // publish IMU sensor_msgs/Imu message
    bool    odometry;       // publish GPS and IMU as nav_msgs/Odometry message
    bool    grayscale;      // publish
    bool    color;          // publish
    bool    viewer;         // enable CV viewer
    bool    timestamps;     // use KITTI timestamps;
    bool    statictf;       // publish static transforms on tf
    bool    odomtf;         // publish odometry on tf
    bool    start_paused;   // start the player paused
    bool    clock;          // publish timestamps on /clock topic
    std::string  frame_oxts;     // frame_id for frame attached to oxts
    std::string  frame_odom;     // frame_id for frame used to publish odometry messages
    std::string  frame_velodyne; // frame_id for frame attached to velodyne
    std::string  frame_image00;  // frame_id for frame attached to camera 00
    std::string  frame_image01;  // frame_id for frame attached to camera 01
    std::string  frame_image02;  // frame_id for frame attached to camera 02
    std::string  frame_image03;  // frame_id for frame attached to camera 03

    void printRequiredDirectoryTree();
};

class Player
{
public:
	Player(ros::NodeHandle& n, ros::NodeHandle& pn, PlayerOptions options);
	~Player();
	
  void run();

private:
  int checkAllDirectories();
  unsigned int countNumEntries();
  void processUserInput();

  int readCharFromStdin();
  void restoreTerminal();
  void setupTerminal();
  void publishStaticTf();
  ros::Time getTimestampAt(unsigned int entry);
  void clockThread();
  void tfThread();
  void publishColorDataAt(unsigned int entry);
  void publishGrayscaleDataAt(unsigned int entry);
  void publishVelodyneDataAt(unsigned int entry);
  void publishGpsDataAt(unsigned int entry);
  void publishImuDataAt(unsigned int entry);
  void publishOdometryDataAt(unsigned int entry);
  void updateOdomTfToDataAt(unsigned int entry);
  tf2::Transform getOdomTfAt(unsigned int entry);

private:
  ros::NodeHandle& n_;
  ros::NodeHandle& pn_;
  PlayerOptions options_;

  std::string dir_root_             ;
  std::string dir_image00_          ; std::string dir_timestamp_image00_;
  std::string dir_image01_          ; std::string dir_timestamp_image01_;
  std::string dir_image02_          ; std::string dir_timestamp_image02_;
  std::string dir_image03_          ; std::string dir_timestamp_image03_;
  std::string dir_oxts_             ; std::string dir_timestamp_oxts_;
  std::string dir_velodyne_points_  ; std::string dir_timestamp_velodyne_; //average of start&end (time of scan)

  unsigned int total_entries_;
  unsigned int entries_played_;

  ros::Time current_timestamp_;

  image_transport::CameraPublisher pub00_;
  image_transport::CameraPublisher pub01_;
  image_transport::CameraPublisher pub02_;
  image_transport::CameraPublisher pub03_;

  ros::Publisher map_pub_;
  ros::Publisher gps_pub_;
  ros::Publisher imu_pub_;
  ros::Publisher odom_pub_;

  boost::thread                   tf_thread_;
  boost::mutex                    tf_lock_;
  tf2_ros::TransformBroadcaster   tf_br_;
  geometry_msgs::TransformStamped tf_msg_;
  bool has_tf_msg_;

  tf2_ros::StaticTransformBroadcaster static_tf_br_;

  cv::Mat cv_image00_;
  cv::Mat cv_image01_;
  cv::Mat cv_image02_;
  cv::Mat cv_image03_;

  sensor_msgs::Image ros_msg00_;
  sensor_msgs::Image ros_msg01_;
  sensor_msgs::Image ros_msg02_;
  sensor_msgs::Image ros_msg03_;

  sensor_msgs::CameraInfo ros_cameraInfoMsg_camera00_;
  sensor_msgs::CameraInfo ros_cameraInfoMsg_camera01_;
  sensor_msgs::CameraInfo ros_cameraInfoMsg_camera02_;
  sensor_msgs::CameraInfo ros_cameraInfoMsg_camera03_;

  cv_bridge::CvImage cv_bridge_img_;

  boost::thread        clock_thread_;
  boost::mutex         clock_lock_;
  ros::Publisher       clock_pub_;
  rosgraph_msgs::Clock clock_msg_;
  bool has_clock_msg_;


  typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
  PointCloud::Ptr points_;

  bool paused_;
  bool step_;
  bool quit_;

  bool has_odom_origin_;
  tf2::Transform odom_origin_;

  bool terminal_modified_;
#if defined(_MSC_VER)
    HANDLE input_handle;
    DWORD stdin_set;
#else
    termios orig_flags_;
    fd_set  stdin_fdset_;
#endif
    int     maxfd_;

};

#endif // _KITTI_PLAYER_H_

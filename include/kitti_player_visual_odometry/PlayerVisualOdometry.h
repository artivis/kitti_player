#ifndef _KITTI_PLAYER_H_
#define _KITTI_PLAYER_H_


#include <kitti_player_visual_odometry/PlayerSupportVisualOdometry.h>



struct PlayerOptions
{
    std::string  path;
    std::string  sequence;
    float   frequency;        // publisher frequency. 1 > Kitti default 10Hz
    bool    all_data;         // publish everything
    bool    velodyne;         // publish velodyne point clouds /as PCL
    bool    grayscale;        // publish
    bool    color;            // publish
    bool    poses;            // publish
    bool    viewer;           // enable CV viewer
    bool    timestamps;       // use KITTI timestamps;
    bool    start_paused;   // start the player paused
    bool    clock;          // publish timestamps on /clock topic
    std::string  frame_velodyne; // frame_id for frame attached to velodyne
    std::string  frame_image00;  // frame_id for frame attached to camera 00
    std::string  frame_image01;  // frame_id for frame attached to camera 01
    std::string  frame_image02;  // frame_id for frame attached to camera 02
    std::string  frame_image03;  // frame_id for frame attached to camera 03
    std::string  bagpath;        // path to write the ros bag

    void printRequiredDirectoryTree();
};

class PlayerVisualOdometry
{
public:
    PlayerVisualOdometry(ros::NodeHandle& n, ros::NodeHandle& pn, PlayerOptions options);
    ~PlayerVisualOdometry();
	
  void publish();
  void writeBag();

private:
  int checkAllDirectories();
  unsigned int countNumEntries();
  void processUserInput();

  int readCharFromStdin();
  void restoreTerminal();
  void setupTerminal();

  ros::Time getTimestampAt(unsigned int entry);
  void clockThread();
  void loadColorDataAt(unsigned int entry);
  void loadGrayscaleDataAt(unsigned int entry);
  void loadVelodyneDataAt(unsigned int entry);
  void loadPosesDataAt(unsigned int entry);

private:
  ros::NodeHandle& n_;
  ros::NodeHandle& pn_;
  PlayerOptions options_;

  std::string dir_root_             ;
  std::string dir_sequence_         ;
  std::string dir_image00_          ;
  std::string dir_image01_          ;
  std::string dir_image02_          ;
  std::string dir_image03_          ;
  std::string dir_velodyne_points_  ;

  unsigned int total_entries_;
  unsigned int entries_played_;

  ros::Time current_timestamp_;

  image_transport::CameraPublisher pub00_;
  image_transport::CameraPublisher pub01_;
  image_transport::CameraPublisher pub02_;
  image_transport::CameraPublisher pub03_;

  ros::Publisher map_pub_;
  ros::Publisher pose_pub;

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

  geometry_msgs::PoseStamped ros_pose;


  sensor_msgs::PointCloud2 points_msg_;

  cv_bridge::CvImage cv_bridge_img_;

  boost::thread        clock_thread_;
  boost::mutex         clock_lock_;
  ros::Publisher       clock_pub_;
  rosgraph_msgs::Clock clock_msg_;

  bool have_pose; //Half of the sequences have pose
  std::vector<Eigen::Matrix3d> rotations;
  std::vector<geometry_msgs::Point> pts;
  std::vector<ros::Time> timestamps;

  bool has_clock_msg_;

  bool paused_;
  bool step_;
  bool quit_;



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

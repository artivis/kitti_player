#include <kitti_player_visual_odometry/Player_visual_odometry.h>


using namespace std;

namespace fs = boost::filesystem;

void PlayerOptions::printRequiredDirectoryTree()
{
    cout << "kitti_player needs a directory tree like the following:" << endl;
    cout << "└── dataset" << endl;
    cout << "    ├── sequences                " << endl;
    cout << "    │   ├── XX                   " << endl;
    cout << "    │   │   ├── image_0/         " << endl;
    cout << "    │   │   ├── image_1/         " << endl;
    cout << "    │   │   ├── image_2/         " << endl;
    cout << "    │   │   ├── image_3/         " << endl;
    cout << "    │   │   ├── velodyne/        " << endl;
    cout << "    │   │   ├── calib.txt        " << endl;
    cout << "    │   │   └── times.txt        " << endl;
    cout << "    │                            " << endl;
    cout << "    └── poses                    " << endl;
    cout << "        ├── XX.txt               " << endl;
}

Player_visual_odometry::Player_visual_odometry(ros::NodeHandle& n, ros::NodeHandle& pn, PlayerOptions options)
 : n_(n),
   pn_(pn),
   options_(options),
   has_clock_msg_(false),
   step_(false),
   quit_(false),
   have_pose(false),
   terminal_modified_(false)
{

    dir_root_             = fs::path(options_.path).string();
    dir_sequence_         = dir_root_+"sequences/"+options.sequence+"/";
    dir_image00_          = dir_sequence_+"image_0/";
    dir_image01_          = dir_sequence_+"image_1/";
    dir_image02_          = dir_sequence_+"image_2/";
    dir_image03_          = dir_sequence_+"image_3/";
    dir_velodyne_points_  = dir_sequence_+"velodyne/";


    ROS_INFO_STREAM ("Checking directories...");
    if(checkAllDirectories())
    {
        ROS_ERROR("Incorrect tree directory , use --help for details");
        quit_ = true;
        return;
    }
    ROS_INFO_STREAM (options_.path << "\t[OK]");

    total_entries_  = countNumEntries();
    entries_played_ = 0;

    // Initialize publishers
    image_transport::ImageTransport it(pn_);
    if(options_.all_data || options_.grayscale)
    {
        pub00_ = it.advertiseCamera("grayscale/left/image_rect", 1);
        pub01_ = it.advertiseCamera("grayscale/right/image_rect", 1);
    }
    if(options_.all_data || options_.color)
    {
        pub02_ = it.advertiseCamera("color/left/image_rect", 1);
        pub03_ = it.advertiseCamera("color/right/image_rect", 1);
    }
    if(options_.all_data || options_.velodyne)
        map_pub_ = pn_.advertise<sensor_msgs::PointCloud2> ("hdl64e", 1, true);


    if(options_.clock)
        clock_pub_ = n_.advertise<rosgraph_msgs::Clock>  ("clock", 1, true);

    if(options_.poses || options_.all_data){
        std::cout << __LINE__ <<endl;
        pose_pub = pn_.advertise<geometry_msgs::PoseStamped>  ("pose", 1, true);
        have_pose=PlayerSupport_visual_odometry::get_poses(dir_root_,options_.sequence,rotations,pts);
    }
    if(options_.timestamps){
        options_.timestamps=PlayerSupport_visual_odometry::getTimestamp(dir_sequence_,timestamps);
    }
    ROS_INFO_STREAM("Waiting 0.2 seconds after advertising topics...");
    ros::WallDuration(0.2).sleep();
    ROS_INFO_STREAM("Waiting 0.2 seconds after advertising topics...  done");

    if(options_.viewer)
    {
        ROS_INFO_STREAM("Opening CV viewer(s)");
        if(options_.color || options_.all_data)
        {
            ROS_DEBUG_STREAM("color||all " << options_.color << " " << options_.all_data);
            cv::namedWindow("CameraSimulator Color Viewer",CV_WINDOW_AUTOSIZE);
            string full_filename_image = (fs::path(dir_image02_) / fs::path(boost::str(boost::format("%006d") % 0 ) + ".png")).string();
            cv_image02_ = cv::imread(full_filename_image, CV_LOAD_IMAGE_UNCHANGED);
            cv::waitKey(5);
        }
        if(options_.grayscale || options_.all_data)
        {
            ROS_DEBUG_STREAM("grayscale||all " << options_.grayscale << " " << options_.all_data);
            cv::namedWindow("CameraSimulator Grayscale Viewer",CV_WINDOW_AUTOSIZE);
            string full_filename_image = (fs::path(dir_image00_) / fs::path(boost::str(boost::format("%006d") % 0 ) + ".png")).string();
            cv_image00_ = cv::imread(full_filename_image, CV_LOAD_IMAGE_UNCHANGED);
            cv::waitKey(5);
        }
        ROS_INFO_STREAM("Opening CV viewer(s)... OK");
    }

    // Load camera info
    if(options_.grayscale || options_.all_data)
    {
        ros_cameraInfoMsg_camera00_.header.frame_id = options_.frame_image00;
        ros_cameraInfoMsg_camera00_.height = 0;
        ros_cameraInfoMsg_camera00_.width  = 0;
        ros_cameraInfoMsg_camera01_.header.frame_id = options_.frame_image01;
        ros_cameraInfoMsg_camera01_.height = 0;
        ros_cameraInfoMsg_camera01_.width  = 0;
        if(!( PlayerSupport_visual_odometry::getCamCalibration(dir_sequence_,"0",ros_cameraInfoMsg_camera00_.K.data(),ros_cameraInfoMsg_camera00_.D,ros_cameraInfoMsg_camera00_.R.data(),ros_cameraInfoMsg_camera00_.P.data()) &&
              PlayerSupport_visual_odometry::getCamCalibration(dir_sequence_,"1",ros_cameraInfoMsg_camera01_.K.data(),ros_cameraInfoMsg_camera01_.D,ros_cameraInfoMsg_camera01_.R.data(),ros_cameraInfoMsg_camera01_.P.data())
            )
          )
        {
            ROS_ERROR_STREAM("Error reading CAMERA00/CAMERA01 calibration");
            quit_ = true;
            return;
        }
        //Assume same height/width for the camera pair
        string full_filename_image = (fs::path(dir_image00_) / fs::path(boost::str(boost::format("%006d") % 0 ) + ".png")).string();
        cv_image00_ = cv::imread(full_filename_image, CV_LOAD_IMAGE_UNCHANGED);
        cv::waitKey(5);
        ros_cameraInfoMsg_camera01_.height = ros_cameraInfoMsg_camera00_.height = cv_image00_.rows;// -1; TODO: CHECK -1?
        ros_cameraInfoMsg_camera01_.width  = ros_cameraInfoMsg_camera00_.width  = cv_image00_.cols;// -1;
    }
    if(options_.color || options_.all_data)
    {
        ros_cameraInfoMsg_camera02_.header.frame_id = options_.frame_image02;
        ros_cameraInfoMsg_camera02_.height = 0;
        ros_cameraInfoMsg_camera02_.width  = 0;
        ros_cameraInfoMsg_camera03_.header.frame_id = options_.frame_image03;
        ros_cameraInfoMsg_camera03_.height = 0;
        ros_cameraInfoMsg_camera03_.width  = 0;
        if(!(  PlayerSupport_visual_odometry::getCamCalibration(dir_sequence_,"2",ros_cameraInfoMsg_camera02_.K.data(),ros_cameraInfoMsg_camera02_.D,ros_cameraInfoMsg_camera02_.R.data(),ros_cameraInfoMsg_camera02_.P.data()) &&
               PlayerSupport_visual_odometry::getCamCalibration(dir_sequence_,"3",ros_cameraInfoMsg_camera03_.K.data(),ros_cameraInfoMsg_camera03_.D,ros_cameraInfoMsg_camera03_.R.data(),ros_cameraInfoMsg_camera03_.P.data())
            )
          )
        {
            ROS_ERROR_STREAM("Error reading CAMERA02/CAMERA03 calibration");
            quit_ = true;
            return;
        }
        //Assume same height/width for the camera pair
        string full_filename_image = (fs::path(dir_image02_) / fs::path(boost::str(boost::format("%006d") % 0 ) + ".png")).string();
        cv_image02_ = cv::imread(full_filename_image, CV_LOAD_IMAGE_UNCHANGED);
        cv::waitKey(5);
        ros_cameraInfoMsg_camera03_.height = ros_cameraInfoMsg_camera02_.height = cv_image02_.rows;// -1;TODO: CHECK, qui potrebbe essere -1
        ros_cameraInfoMsg_camera03_.width  = ros_cameraInfoMsg_camera02_.width  = cv_image02_.cols;// -1;
    }
}

Player_visual_odometry::~Player_visual_odometry()
{
    if(options_.viewer)
    {
        ROS_INFO_STREAM(" Closing CV viewer(s)");
        if(options_.color || options_.all_data)
            cv::destroyWindow("CameraSimulator Color Viewer");
        if(options_.grayscale || options_.all_data)
            cv::destroyWindow("CameraSimulator Grayscale Viewer");
        ROS_INFO_STREAM(" Closing CV viewer(s)... OK");
    }

    restoreTerminal();
}

int Player_visual_odometry::checkAllDirectories()
{
    // Return non-zero if directory tree is wrong
    if (
            (options_.all_data   && (   (!fs::exists(fs::path(dir_image00_))         || !fs::is_directory(fs::path(dir_image00_))) ||
                                        (!fs::exists(fs::path(dir_image01_))         || !fs::is_directory(fs::path(dir_image01_))) ||
                                        (!fs::exists(fs::path(dir_image02_))         || !fs::is_directory(fs::path(dir_image02_))) ||
                                        (!fs::exists(fs::path(dir_image03_))         || !fs::is_directory(fs::path(dir_image03_))) ||
                                        (!fs::exists(fs::path(dir_velodyne_points_)) || !fs::is_directory(fs::path(dir_velodyne_points_)))
                                    )
            )
            ||
            (options_.color      && (   (!fs::exists(fs::path(dir_image02_)) || !fs::is_directory(fs::path(dir_image02_))) ||
                                        (!fs::exists(fs::path(dir_image03_)) || !fs::is_directory(fs::path(dir_image03_)))
                                    )
            )
            ||
            (options_.grayscale  && (   (!fs::exists(fs::path(dir_image00_)) || !fs::is_directory(fs::path(dir_image00_))) ||
                                        (!fs::exists(fs::path(dir_image01_)) || !fs::is_directory(fs::path(dir_image01_)))
                                    )
            )
            ||

            (options_.velodyne   && (   (!fs::exists(fs::path(dir_velodyne_points_)) || !fs::is_directory(fs::path(dir_velodyne_points_)))))
            ||
            (options_.timestamps && (   (!fs::exists(fs::path(dir_sequence_))  || !fs::is_directory(fs::path(dir_sequence_)))
                                    )
            )
        )
    {
        return 1;
    }

    // If we get here we're OK
    return 0;
}

unsigned int Player_visual_odometry::countNumEntries()
{
    // sync datasets have the same number of elements on each folder, so
    // we just need to count one of them.
    if (options_.all_data)
    {
        return std::count_if(
            fs::directory_iterator(fs::path(dir_image02_)),
            fs::directory_iterator(),
            boost::bind( static_cast<bool(*)(const fs::path&)>(fs::is_regular_file), 
            boost::bind( &fs::directory_entry::path, _1 ) ) );
    }

    if (options_.color)
    {
        return std::count_if(
            fs::directory_iterator(fs::path(dir_image02_)),
            fs::directory_iterator(),
            boost::bind( static_cast<bool(*)(const fs::path&)>(fs::is_regular_file), 
            boost::bind( &fs::directory_entry::path, _1 ) ) );
    }

    if (options_.grayscale)
    {
        return std::count_if(
            fs::directory_iterator(fs::path(dir_image00_)),
            fs::directory_iterator(),
            boost::bind( static_cast<bool(*)(const fs::path&)>(fs::is_regular_file), 
            boost::bind( &fs::directory_entry::path, _1 ) ) );
    }


    if (options_.velodyne)
    {
        return std::count_if(
            fs::directory_iterator(fs::path(dir_velodyne_points_)),
            fs::directory_iterator(),
            boost::bind( static_cast<bool(*)(const fs::path&)>(fs::is_regular_file), 
            boost::bind( &fs::directory_entry::path, _1 ) ) );
    }

    // We should never get here.
    ROS_ERROR("An error occured when counting number of entries.");
    return 0;
}

void Player_visual_odometry::writeBag()
{
    rosbag::Bag bag;
    bag.open(options_.bagpath, rosbag::bagmode::Write);

    setupTerminal();
    boost::progress_display progress(total_entries_) ;
    while(entries_played_<total_entries_ && !quit_ && ros::ok())
    {
        processUserInput();

        if(!ros::ok() || quit_)  break;

        current_timestamp_ = getTimestampAt(entries_played_);

        // do it once


        if(options_.color || options_.all_data)
        {
            loadColorDataAt(entries_played_);
            bag.write(ros::names::resolve("~color/left/image_rect"), current_timestamp_, ros_msg02_);
            bag.write(ros::names::resolve("~color/left/camera_info"), current_timestamp_, ros_cameraInfoMsg_camera02_);
            bag.write(ros::names::resolve("~color/right/image_rect"), current_timestamp_, ros_msg03_);
            bag.write(ros::names::resolve("~color/right/camera_info"), current_timestamp_, ros_cameraInfoMsg_camera03_);
        }

        if(options_.grayscale || options_.all_data)
        {
            loadGrayscaleDataAt(entries_played_);
            bag.write(ros::names::resolve("~grayscale/left/image_rect"), current_timestamp_, ros_msg00_);
            bag.write(ros::names::resolve("~grayscale/left/camera_info"), current_timestamp_, ros_cameraInfoMsg_camera00_);
            bag.write(ros::names::resolve("~grayscale/right/image_rect"), current_timestamp_, ros_msg01_);
            bag.write(ros::names::resolve("~grayscale/right/camera_info"), current_timestamp_, ros_cameraInfoMsg_camera01_);
        }

        if(options_.velodyne || options_.all_data)
        {
            loadVelodyneDataAt(entries_played_);
            bag.write(ros::names::resolve("~hdl64e"), current_timestamp_, points_msg_);
        }

        if((options_.poses || options_.all_data) && have_pose)
        {
            loadPosesDataAt(entries_played_);
            bag.write(ros::names::resolve("~poses"), current_timestamp_, ros_pose);
        }


        ++progress;
        entries_played_++;
    }

    bag.close();
    restoreTerminal();
    quit_ = true;
}
    
void Player_visual_odometry::publish()
{
    paused_ = options_.start_paused;
    ros::WallRate loop_rate(options_.frequency);

    if(options_.clock)
        clock_thread_ = boost::thread( &Player_visual_odometry::clockThread, this );


    setupTerminal();
    boost::progress_display progress(total_entries_) ;

    while(entries_played_<total_entries_ && !quit_ && ros::ok())
    {
        processUserInput();

        if(!ros::ok() || quit_)  break;
        if(paused_ && !step_)  continue;

        // variable used to display time spent on the loop
        boost::timer t;

        if(options_.timestamps || options_.clock)
            current_timestamp_ = getTimestampAt(entries_played_);
        else
            current_timestamp_ = ros::Time::now();

        // do it once


        if(options_.color || options_.all_data)
        {
            loadColorDataAt(entries_played_);

            if(options_.viewer)
            {
                //display the left image only
                cv::imshow("CameraSimulator Color Viewer",cv_image02_);
                //give some time to draw images
                cv::waitKey(5);
            }

            pub02_.publish(ros_msg02_,ros_cameraInfoMsg_camera02_);
            pub03_.publish(ros_msg03_,ros_cameraInfoMsg_camera03_);

        }

        if(options_.grayscale || options_.all_data)
        {
            loadGrayscaleDataAt(entries_played_);

            if(options_.viewer)
            {
                //display the left image only
                cv::imshow("CameraSimulator Grayscale Viewer",cv_image00_);
                //give some time to draw images
                cv::waitKey(5);
            }

            pub00_.publish(ros_msg00_,ros_cameraInfoMsg_camera00_);
            pub01_.publish(ros_msg01_,ros_cameraInfoMsg_camera01_);

        }

        if(options_.velodyne || options_.all_data)
        {
            loadVelodyneDataAt(entries_played_);
            map_pub_.publish(points_msg_);

        }
        if((options_.poses || options_.all_data) && have_pose)
        {

            loadPosesDataAt(entries_played_);
            pose_pub.publish(ros_pose);

        }


        if(options_.clock)
        {
            // Update clock to be published in the clock thread
            clock_lock_.lock();
            clock_msg_.clock = current_timestamp_;
            clock_lock_.unlock();
            if(!has_clock_msg_) has_clock_msg_ = true;
        }

        ++progress;
        entries_played_++;
        ROS_DEBUG_STREAM("Loop took "<< t.elapsed() << " [s].");        
        loop_rate.sleep();
    }
    
    // clean up
    restoreTerminal();
    quit_ = true;
    if(options_.clock)
        clock_thread_.join();

    ROS_INFO("Done.");
}





void Player_visual_odometry::loadVelodyneDataAt(unsigned int entry)
{
    string full_filename_velodyne = (fs::path(dir_velodyne_points_) / fs::path(boost::str(boost::format("%006d") % entry ) + ".bin")).string();
    fstream input(full_filename_velodyne.c_str(), ios::in | ios::binary);
    if(!input.good())
    {
        ROS_ERROR_STREAM ( "Could not read file: " << full_filename_velodyne );
        quit_ = true;
        return;
    }

    pcl::PointCloud<pcl::PointXYZI> points;

    ROS_DEBUG_STREAM ("reading " << full_filename_velodyne);
    input.seekg(0, ios::beg);
    while(input.good() && !input.eof())
    {
        pcl::PointXYZI point;
        input.read((char *) &point.x, 3*sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));
        points.push_back(point);
    }
    input.close();

    // convert from pcl to sensor_msg 
    pcl::toROSMsg(points, points_msg_);
    // fill up header
    points_msg_.header.frame_id = options_.frame_velodyne;
    points_msg_.header.stamp = current_timestamp_;
}

void Player_visual_odometry::loadColorDataAt(unsigned int entry)
{
    string full_filename_image02 = (fs::path(dir_image02_) / fs::path(boost::str(boost::format("%006d") % entry ) + ".png")).string();
    string full_filename_image03 = (fs::path(dir_image03_) / fs::path(boost::str(boost::format("%006d") % entry ) + ".png")).string();
    ROS_DEBUG_STREAM ( full_filename_image02 << endl << full_filename_image03 << endl << endl);

    cv_image02_ = cv::imread(full_filename_image02, CV_LOAD_IMAGE_UNCHANGED);
    cv_image03_ = cv::imread(full_filename_image03, CV_LOAD_IMAGE_UNCHANGED);
    if ( (cv_image02_.data == NULL) || (cv_image03_.data == NULL) ){
        ROS_ERROR_STREAM("Error reading color images (02 & 03)");
        ROS_ERROR_STREAM(full_filename_image02 << endl << full_filename_image03);
        quit_ = true;
        return;
    }

    cv_bridge_img_.encoding = sensor_msgs::image_encodings::BGR8;

    ros_msg02_.header.stamp = ros_cameraInfoMsg_camera02_.header.stamp = cv_bridge_img_.header.stamp = current_timestamp_;
    cv_bridge_img_.header.frame_id = options_.frame_image02;
    cv_bridge_img_.image = cv_image02_;
    cv_bridge_img_.toImageMsg(ros_msg02_);

    ros_msg03_.header.stamp = ros_cameraInfoMsg_camera03_.header.stamp = cv_bridge_img_.header.stamp = current_timestamp_;
    cv_bridge_img_.header.frame_id = options_.frame_image03;
    cv_bridge_img_.image = cv_image03_;
    cv_bridge_img_.toImageMsg(ros_msg03_);
}

void Player_visual_odometry::loadGrayscaleDataAt(unsigned int entry)
{
    string full_filename_image00 = (fs::path(dir_image00_) / fs::path(boost::str(boost::format("%006d") % entry ) + ".png")).string();
    string full_filename_image01 = (fs::path(dir_image01_) / fs::path(boost::str(boost::format("%006d") % entry ) + ".png")).string();
    ROS_DEBUG_STREAM ( full_filename_image00 << endl << full_filename_image01 << endl << endl);

    cv_image00_ = cv::imread(full_filename_image00, CV_LOAD_IMAGE_UNCHANGED);
    cv_image01_ = cv::imread(full_filename_image01, CV_LOAD_IMAGE_UNCHANGED);
    if ( (cv_image00_.data == NULL) || (cv_image01_.data == NULL) ){
        ROS_ERROR_STREAM("Error reading color images (00 & 01)");
        ROS_ERROR_STREAM(full_filename_image00 << endl << full_filename_image01);
        quit_ = true;
        return;
    }

    cv_bridge_img_.encoding = sensor_msgs::image_encodings::MONO8;

    ros_msg00_.header.stamp = ros_cameraInfoMsg_camera00_.header.stamp = cv_bridge_img_.header.stamp = current_timestamp_;
    cv_bridge_img_.header.frame_id = options_.frame_image00;
    cv_bridge_img_.image = cv_image00_;
    cv_bridge_img_.toImageMsg(ros_msg00_);

    ros_msg01_.header.stamp = ros_cameraInfoMsg_camera01_.header.stamp = cv_bridge_img_.header.stamp = current_timestamp_;
    cv_bridge_img_.header.frame_id = options_.frame_image01;
    cv_bridge_img_.image = cv_image01_;
    cv_bridge_img_.toImageMsg(ros_msg01_);
}

void Player_visual_odometry::loadPosesDataAt(unsigned int entry){
    Eigen::Quaterniond q(rotations[entry]);
    geometry_msgs::Quaternion qu;
    qu.x=q.x();
    qu.y=q.y();
    qu.z=q.z();
    qu.w=q.w();

    ros_pose.header=cv_bridge_img_.header;
    ros_pose.pose.position=pts[entry];
    ros_pose.pose.orientation=qu;
}

ros::Time Player_visual_odometry::getTimestampAt(unsigned int entry)
{
    return timestamps[entry];
}

void Player_visual_odometry::clockThread()
{
    // Setup publishing rate
    ros::WallRate clock_rate = ros::WallRate(100.0);

    ROS_DEBUG_STREAM("Starting clock publish thread.");
    while(ros::ok() && !quit_)
    {
        if(has_clock_msg_)
        {
            clock_lock_.lock();
            clock_pub_.publish(clock_msg_);
            clock_lock_.unlock();
        }
        clock_rate.sleep();
    }
    ROS_DEBUG_STREAM("Finishing clock publish thread.");
}



void Player_visual_odometry::processUserInput()
{
    bool charsleftorpaused = true;
    step_ = false;  quit_ = false;
    while (charsleftorpaused && ros::ok())
    {
        switch (readCharFromStdin()){
        case ' ':
            paused_ = !paused_;
            break;
        case 's':
            step_ = true;
            break;
        case 'q':
            quit_ = true;
            break;
        case EOF:
            charsleftorpaused = false;
        }
    }
}

void Player_visual_odometry::setupTerminal() {
    if (terminal_modified_)
        return;

#if defined(_MSC_VER)
    input_handle = GetStdHandle(STD_INPUT_HANDLE);
    if (input_handle == INVALID_HANDLE_VALUE)
    {
        std::cout << "Failed to set up standard input handle." << std::endl;
        return;
    }
    if (! GetConsoleMode(input_handle, &stdin_set) )
    {
        std::cout << "Failed to save the console mode." << std::endl;
        return;
    }
    // don't actually need anything but the default, alternatively try this
    //DWORD event_mode = ENABLE_WINDOW_INPUT | ENABLE_MOUSE_INPUT;
    //if (! SetConsoleMode(input_handle, event_mode) )
    //{
    // std::cout << "Failed to set the console mode." << std::endl;
    // return;
    //}
    terminal_modified_ = true;
#else
    const int fd = fileno(stdin);
    termios flags;
    tcgetattr(fd, &orig_flags_);
    flags = orig_flags_;
    flags.c_lflag &= ~ICANON;      // set raw (unset canonical modes)
    flags.c_lflag &= ~ECHO;        // disable echo
    flags.c_cc[VMIN]  = 0;         // i.e. min 1 char for blocking, 0 chars for non-blocking
    flags.c_cc[VTIME] = 0;         // block if waiting for char
    tcsetattr(fd, TCSANOW, &flags);

    FD_ZERO(&stdin_fdset_);
    FD_SET(fd, &stdin_fdset_);
    maxfd_ = fd + 1;
    terminal_modified_ = true;
#endif
}

void Player_visual_odometry::restoreTerminal() {
    if (!terminal_modified_)
        return;

#if defined(_MSC_VER)
    SetConsoleMode(input_handle, stdin_set);
#else
    const int fd = fileno(stdin);
    tcsetattr(fd, TCSANOW, &orig_flags_);
#endif
    terminal_modified_ = false;
}

int Player_visual_odometry::readCharFromStdin() {
#ifdef __APPLE__
    fd_set testfd;
    FD_COPY(&stdin_fdset_, &testfd);
#elif !defined(_MSC_VER)
    fd_set testfd = stdin_fdset_;
#endif

#if defined(_MSC_VER)
    DWORD events = 0;
    INPUT_RECORD input_record[1];
    DWORD input_size = 1;
    BOOL b = GetNumberOfConsoleInputEvents(input_handle, &events);
    if (b && events > 0)
    {
        b = ReadConsoleInput(input_handle, input_record, input_size, &events);
        if (b)
        {
            for (unsigned int i = 0; i < events; ++i)
            {
                if (input_record[i].EventType & KEY_EVENT & input_record[i].Event.KeyEvent.bKeyDown)
                {
                    CHAR ch = input_record[i].Event.KeyEvent.uChar.AsciiChar;
                    return ch;
                }
            }
        }
    }
    return EOF;
#else
    timeval tv;
    tv.tv_sec  = 0;
    tv.tv_usec = 0;
    if (select(maxfd_, &testfd, NULL, NULL, &tv) <= 0)
        return EOF;
    return getc(stdin);
#endif
}

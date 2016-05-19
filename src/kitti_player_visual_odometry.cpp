#include <kitti_player_visual_odometry/PlayerVisualOdometry.h>



#include <boost/program_options.hpp>

using namespace std;

namespace po = boost::program_options;

int main(int argc, char **argv)
{
    PlayerOptions options;
    po::variables_map vm;

    po::options_description desc("Kitti_player, a player for KITTI raw datasets\nDatasets can be downloaded from: http://www.cvlibs.net/datasets/kitti/raw_data.php\n\nAllowed options",200);
    desc.add_options()
        ("help,h"                                                                                            ,  "help message")
        ("directory,d"   ,  po::value<string>(&options.path)->required()                                     ,  "*required* - path to the kitti dataset Directory")
        ("sequence  ,s"  ,  po::value<string>(&options.sequence)->required()                                 ,  "*required* - sequence number (starts at 00)")
        ("poses     ,p"  ,  po::value<bool>(&options.poses)          ->implicit_value(1) ->default_value(0)   ,  "replay poses")
        ("frequency,f"   ,  po::value<float>(&options.frequency)     ->default_value(10.0)                   ,  "set replay Frequency")
        ("all,a"         ,  po::value<bool> (&options.all_data)      ->implicit_value(1) ->default_value(0)  ,  "replay All data")
        ("velodyne,v"    ,  po::value<bool> (&options.velodyne)      ->implicit_value(1) ->default_value(0)  ,  "replay Velodyne data")
        ("grayscale,g"   ,  po::value<bool> (&options.grayscale)     ->implicit_value(1) ->default_value(0)  ,  "replay Stereo Grayscale images")
        ("color,c"       ,  po::value<bool> (&options.color)         ->implicit_value(1) ->default_value(0)  ,  "replay Stereo Color images")
        ("viewer,V"        ,  po::value<bool> (&options.viewer)        ->implicit_value(1) ->default_value(0)  ,  "enable image viewer")
        ("timestamps,T"  ,  po::value<bool> (&options.timestamps)    ->implicit_value(1) ->default_value(0)  ,  "use KITTI timestamps")
        ("pause,P"         ,  po::value<bool> (&options.start_paused)  ->implicit_value(1) ->default_value(0)  ,  "start the player paused")
        ("clock,C"         ,  po::value<bool> (&options.clock)         ->implicit_value(1) ->default_value(0)  ,  "publish timestamps on /clock")
        ("frame_velodyne",  po::value<string>(&options.frame_velodyne)->default_value("velodyne")            ,  "name of frame attached to velodyne")
        ("frame_image00" ,  po::value<string>(&options.frame_image00)->default_value("image00")              ,  "name of frame attached to camera 00")
        ("frame_image01" ,  po::value<string>(&options.frame_image01)->default_value("image01")              ,  "name of frame attached to camera 01")
        ("frame_image02" ,  po::value<string>(&options.frame_image02)->default_value("image02")              ,  "name of frame attached to camera 02")
        ("frame_image03" ,  po::value<string>(&options.frame_image03)->default_value("image03")              ,  "name of frame attached to camera 03")
        ("bag,b"         ,  po::value<string>(&options.bagpath)->default_value("")                           ,  "if informed th player will write the dataset as a rosbag on the informed path")
    ;

    try // parse options
    {
        po::parsed_options parsed = po::command_line_parser(argc, argv).options(desc).allow_unregistered().run();
        po::store(parsed, vm);
        po::notify(vm);

        // vector<string> to_pass_further = po::collect_unrecognized(parsed.options, po::include_positional);

        // Can't handle __ros (ROS parameters ... )
        //        if (to_pass_further.size()>0)
        //        {
        //            ROS_WARN_STREAM("Unknown Options Detected, shutting down node\n");
        //            cerr << desc << endl;
        //            return 1;
        //        }
    }
    catch(...)
    {
        cerr << desc << endl;
        options.printRequiredDirectoryTree();
        ROS_WARN_STREAM("Parse error, shutting down node\n");
        return 1;
    }

    if (vm.count("help")) {
        cout << desc << endl;
        options.printRequiredDirectoryTree();
        return 1;
    }

    if (!(options.all_data || options.color || options.grayscale || options.velodyne ))
    {
        ROS_WARN_STREAM("Job finished without playing the dataset. No 'publishing' parameters provided");
        return 1;
    }

    ros::init(argc, argv, "kitti_player");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");

    PlayerVisualOdometry player(n, pn, options);

    if (!options.bagpath.empty())
        player.writeBag();
    else
        player.publish();

	return 0;
}

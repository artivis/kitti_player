/*
 * Copyright (c) 2016, Ellon Paiva Mendes
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cstdio>
#include <string>
#include <iostream>
#include <fstream>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h> 
#include "tf2_ros/static_transform_broadcaster.h"

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

using namespace std;
using namespace boost;

/// Reads a calibration file and fills the TransformedStamped with the transformation
bool calib_file_to_tf_stamped(string filename, geometry_msgs::TransformStamped & msg)
{
  bool found_T = false, found_R = false;

  ifstream file(filename);
  string line;
  if (file.is_open())
  {
    while ( getline (file,line) )
    {
      // Split the line into :'s 
      typedef vector< string > split_vector_type;
      split_vector_type SplitVec;
      split( SplitVec, line, is_any_of(":"), token_compress_on );

      if(!SplitVec.empty() && SplitVec[0] == "T")
      {
        // Fill up translation part
        string value_string = trim_copy(SplitVec[1]);
        SplitVec.clear();
        split( SplitVec, value_string, is_any_of(" "), token_compress_on );         
        msg.transform.translation.x = lexical_cast<double>(SplitVec[0]);
        msg.transform.translation.y = lexical_cast<double>(SplitVec[1]);
        msg.transform.translation.z = lexical_cast<double>(SplitVec[2]);
        found_T = true;
      }
      else if(!SplitVec.empty() && SplitVec[0] == "R")
      {
        // Fill up rotation part
        string value_string = trim_copy(SplitVec[1]);
        SplitVec.clear();
        split( SplitVec, value_string, is_any_of(" "), token_compress_on );
        tf2::Matrix3x3 mat(lexical_cast<double>(SplitVec[0]),lexical_cast<double>(SplitVec[1]),lexical_cast<double>(SplitVec[2]),
                           lexical_cast<double>(SplitVec[3]),lexical_cast<double>(SplitVec[4]),lexical_cast<double>(SplitVec[5]),
                           lexical_cast<double>(SplitVec[6]),lexical_cast<double>(SplitVec[7]),lexical_cast<double>(SplitVec[8]));
        tf2::Quaternion quat;
        mat.getRotation(quat);
        msg.transform.rotation.x = quat.x();
        msg.transform.rotation.y = quat.y();
        msg.transform.rotation.z = quat.z();
        msg.transform.rotation.w = quat.w();
        found_R = true;
      }
    }
    file.close();
  }

  // Return success if we found both T and R in the file
  if(found_T && found_R)
      return false;

  // Error
  return true;
}

int main(int argc, char ** argv)
{
  //Initialize ROS
  ros::init(argc, argv,"kitti_static_transform_publisher", ros::init_options::AnonymousName);
  tf2_ros::StaticTransformBroadcaster broadcaster;

  if(argc == 4)
  {
    geometry_msgs::TransformStamped msg;

    if(calib_file_to_tf_stamped(argv[1],msg))
    {
      ROS_ERROR("error parsing calib file");
      return -1;
    }

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = argv[2];
    msg.child_frame_id = argv[3];

    broadcaster.sendTransform(msg);
    ROS_INFO("Spinning until killed publishing %s to %s", msg.header.frame_id.c_str(), msg.child_frame_id.c_str());
    ros::spin();

    return 0;
  }
  else
  {
    printf("A command line utility for manually sending a transform.\n");
    //printf("It will periodicaly republish the given transform. \n");
    printf("Usage: kitti_static_transform_publisher calib_file.txt frame_id child_frame_id \n");
    printf("\nThis transform is the transform of the coordinate frame from frame_id into the coordinate frame \n");
    printf("of the child_frame_id.  \n");
    ROS_ERROR("kitti_static_transform_publisher exited due to not having the right number of arguments");
    return -1;
  }
}

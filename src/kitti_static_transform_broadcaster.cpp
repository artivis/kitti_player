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
#include <tf2/LinearMath/Transform.h> 
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2_ros/static_transform_broadcaster.h>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

using namespace std;
using namespace boost;

/// Fill up translation part of msg
void parse_translation(string value_string, geometry_msgs::TransformStamped & msg)
{
  typedef vector< string > split_vector_type;
  split_vector_type SplitVec;

  trim(value_string);
  split( SplitVec, value_string, is_any_of(" "), token_compress_on );         

  msg.transform.translation.x = lexical_cast<double>(SplitVec[0]);
  msg.transform.translation.y = lexical_cast<double>(SplitVec[1]);
  msg.transform.translation.z = lexical_cast<double>(SplitVec[2]);
}

/// Fill up rotation part of msg
void parse_rotation(string value_string, geometry_msgs::TransformStamped & msg)
{
  typedef vector< string > split_vector_type;
  split_vector_type SplitVec;

  trim(value_string);
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
}

bool cam_calib_file_to_tf_stamped(string filename, vector<geometry_msgs::TransformStamped> & msgs)
{
  bool found_T_01 = false, found_R_01 = false;
  bool found_T_02 = false, found_R_02 = false;
  bool found_T_03 = false, found_R_03 = false;

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

      if(!SplitVec.empty() && SplitVec[0] == "T_01")
      {
        parse_translation(SplitVec[1],msgs[0]);
        found_T_01 = true;
      }
      else if(!SplitVec.empty() && SplitVec[0] == "R_01")
      {
        parse_rotation(SplitVec[1],msgs[0]);
        found_R_01 = true;
      }
      else if(!SplitVec.empty() && SplitVec[0] == "T_02")
      {
        parse_translation(SplitVec[1],msgs[1]);
        found_T_02 = true;
      }
      else if(!SplitVec.empty() && SplitVec[0] == "R_02")
      {
        parse_rotation(SplitVec[1],msgs[1]);
        found_R_02 = true;
      }
      else if(!SplitVec.empty() && SplitVec[0] == "T_03")
      {
        parse_translation(SplitVec[1],msgs[2]);
        found_T_03 = true;
      }
      else if(!SplitVec.empty() && SplitVec[0] == "R_03")
      {
        parse_rotation(SplitVec[1],msgs[2]);
        found_R_03 = true;
      }
    }
    file.close();
  }

  // Return success if we found all T and R in the file
  if(found_T_01 && found_R_01 &&
     found_T_02 && found_R_02 &&
     found_T_03 && found_R_03)
      return false;

  // Error
  return true;
}

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
        parse_translation(SplitVec[1],msg);
        found_T = true;
      }
      else if(!SplitVec.empty() && SplitVec[0] == "R")
      {
        parse_rotation(SplitVec[1],msg);
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
    msg.child_frame_id = argv[2];
    msg.header.frame_id = argv[3];

    broadcaster.sendTransform(msg);
    ROS_INFO("Spinning until killed publishing %s to %s", msg.child_frame_id.c_str(), msg.header.frame_id.c_str());
    ros::spin();

    return 0;
  }
  else if(argc == 6)
  {
    vector<geometry_msgs::TransformStamped> msgs(3);

    if(cam_calib_file_to_tf_stamped(argv[1],msgs))
    {
      ROS_ERROR("error parsing calib file");
      return -1;
    }

    msgs[0].child_frame_id = argv[3];
    msgs[1].child_frame_id = argv[4];
    msgs[2].child_frame_id = argv[5];

    std::for_each(msgs.begin(),msgs.end(),[&] (geometry_msgs::TransformStamped & msg) 
    {
      // Invert transform messages. This is necessary because the cam
      // transforms defines image_00 in the frame of image_{01,02,03}, and
      // this is not suitable for TF since it means image_00 will have
      // multiple parents in the tf tree. Inverting makes image_{01,02,03}
      // childs of image_00
      tf2::Transform tf;
      tf2::convert(msg.transform,tf);
      tf2::convert(tf.inverse(),msg.transform);

      // Fill up header
      msg.header.frame_id = argv[2];
      msg.header.stamp = ros::Time::now();

      broadcaster.sendTransform(msg);
    } );
    
    ROS_INFO("Spinning until killed. Publishing: %s to %s; %s to %s; and %s to %s",
      msgs[0].child_frame_id.c_str(), msgs[0].header.frame_id.c_str(),
      msgs[1].child_frame_id.c_str(), msgs[1].header.frame_id.c_str(),
      msgs[2].child_frame_id.c_str(), msgs[2].header.frame_id.c_str());
    ros::spin();

    return 0;    
  }
  else
  {
    printf("A command line utility for manually sending a transform.\n");
    //printf("It will periodicaly republish the given transform. \n");
    printf("Usage: kitti_static_transform_publisher calib_X_to_Y.txt frame_id_X frame_id_Y \n");
    printf("OR \n");
    printf("Usage: kitti_static_transform_publisher calib_cam_to_cam.txt frame_id_image_00 frame_id_image_01 frame_id_image_02 frame_id_image_03 \n");
    printf("\nThis transform is the transform of the coordinate frame child_frame_id into the coordinate frame frame_id.\n");
    ROS_ERROR("kitti_static_transform_publisher exited due to not having the right number of arguments");
    return -1;
  }
}

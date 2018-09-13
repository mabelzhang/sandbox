#ifndef _CAMERA_INFO_H_
#define _CAMERA_INFO_H_

// Mabel Zhang
// 12 Sep 2018
//
//
//

#include <iostream>  // std::cout
#include <fstream>   // std::ifstream
#include <sstream>

#include <boost/filesystem.hpp>

// ROS
#include <ros/package.h>

#include <Eigen/Core>

#include <util/io_util.h>  // join_paths(), dirname(), basename(), splitext()


// P: Return value. 3 x 4 projection matrix.
//       [fx'  0  cx' Tx]
//   P = [ 0  fy' cy' Ty]
//       [ 0   0   1   0]
//   http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CameraInfo.html
bool load_intrinsics (Eigen::MatrixXf & P)
{
  ////////
  // Load file with paths to camera configuration, written by
  //   scene_generation.py
 
  // Get path of this package
  std::string pkg_path = ros::package::getPath ("depth_scene_rendering");

  // Load the file containing the path to camera intrinsics matrix
  std::string intrinsics_cfg_path = "";
  join_paths (pkg_path, "config/intrinsics_path.txt", intrinsics_cfg_path);
  std::ifstream intrinsics_cfg_f (intrinsics_cfg_path.c_str ());

  // Read the file to get the path
  std::string intrinsics_path = "";
  std::getline (intrinsics_cfg_f, intrinsics_path);
  intrinsics_cfg_f.close ();


  /////
  // Load camera intrinsics matrix, written by scene_generation.py

  if (! boost::filesystem::exists (intrinsics_path))
  {
    fprintf (stderr, "%sERROR: Camera intrinsics file does not exist: "
      "%s%s\n", FAIL, intrinsics_path.c_str (), ENDC);
    return false;
  }

  // Read camera intrinsics matrix
  P = Eigen::MatrixXf::Zero (3, 4);
  std::ifstream intrinsics_f (intrinsics_path.c_str ());
  std::string row = "";
  // Each line in the file is a row in the matrix, delimited by space
  int row_i = 0;
  while (std::getline (intrinsics_f, row))
  {
    std::istringstream iss (row);
    // Read three columns of 3 x 3 matrix
    float c1, c2, c3, c4;
    if (! (iss >> c1 >> c2 >> c3 >> c4))
    {
      printf ("%sERROR: error reading row %d of camera intrinsics matrix in %s. Stopping.%s\n", FAIL, row_i, intrinsics_path.c_str (), ENDC);
      break;
    }

    P.row (row_i) = Eigen::Vector4f (c1, c2, c3, c4);
    row_i ++;
  }

  std::cout << "Loaded camera intrinsics: " << std::endl << P << std::endl;

  return true;
}

#endif

// Mabel Zhang
// 4 Sep 2018
//
// Convert .pcd files from BlenSor (outputted by scene_generation.py, which
//   calls scan_kinect.py), to 2D images with raw depth values, using OpenCV and
//   intrinsic camera matrix from BlenSor.
// This can't be done from within Blender, `.` Blender doesn't know about
//   OpenCV libraries. Must do as post-processing from terminal.
//
// Usage:
// $ rosrun depth_scene_rendering postprocess_scenes 
//

#include <iostream>     // std::cout
#include <fstream>      // std::ifstream
#include <sstream>
#include <cfloat>

#include <boost/filesystem.hpp>

// ROS
#include <ros/package.h>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// Eigen (PCL underlying matrices)
#include <Eigen/Core>

// OpenCV
#include <opencv2/opencv.hpp>
// Ref cv::sfm header files https://docs.opencv.org/3.1.0/d3/df9/sfm_8hpp.html
//#include <opencv2/sfm/projection.hpp>

// Custom
#include <util/ansi_colors.h>
#include <util/io_util.h>  // join_paths(), dirname(), basename(), splitext()
#include <util/pcd_util.h>  // load_cloud_file()


int main (int argc, char ** argv)
{
  // Get path of this package
  std::string pkg_path = ros::package::getPath ("depth_scene_rendering");


  // Works, but don't need it. Raw depths already saved in .pcd!
  /*
  ////////
  // Load camera intrinsics matrix, written by scene_generation.py

  // Load the file containing the path to camera intrinsics matrix
  std::string intrinsics_cfg_path = "";
  join_paths (pkg_path, "config/intrinsics_path.txt", intrinsics_cfg_path);
  std::ifstream intrinsics_cfg_f (intrinsics_cfg_path.c_str ());
  // Read the file
  std::string intrinsics_path = "";
  std::getline (intrinsics_cfg_f, intrinsics_path);

  // Read camera intrinsics matrix
  Eigen::Matrix3f intrinsics = Eigen::Matrix3f ();
  std::ifstream intrinsics_f (intrinsics_path.c_str ());
  std::string row = "";
  // Each line in the file is a row in the matrix, delimited by space
  int row_i = 0;
  while (std::getline (intrinsics_f, row))
  {
    std::istringstream iss (row);
    // Read three columns of 3 x 3 matrix
    float c1, c2, c3;
    if (! (iss >> c1 >> c2 >> c3))
    {
      printf ("ERROR: error reading row %d of camera intrinsics matrix in %s. Stopping.\n", row_i, intrinsics_path.c_str ());
      break;
    }

    intrinsics.row (row_i) = Eigen::Vector3f (c1, c2, c3);
    row_i ++;
  }

  std::cout << "Loaded camera intrinsics: " << std::endl << intrinsics << std::endl;
  */


  // Text file with list of .pcd scene names, written by scene_generation.py
  std::string scene_list_path = "";
  //join_paths (pkg_path, "config/scenes_noisy.txt", scene_list_path);
  join_paths (pkg_path, "config/scenes_test.txt", scene_list_path);
  std::ifstream scene_list_f (scene_list_path.c_str ());

  // Camera is never more than 2 m from table. PNG integer color values will be
  //   obtained by scaling the raw floating point depth by this. This
  //   corresponds to RGB values of 255.
  //float MAX_DEPTH = 2.0;
  // TODO: TEMPORARILY set to larger depth to test on BlenSor default cube scene
  // TODO: Use BlenSor Kinect camera's max/min depth instead... using that, we can
  //   recover the raw depth from their PGM, I think. Try it.
  float MIN_DEPTH = 0.7;
  float MAX_DEPTH = 6.0;


  // Read text file line by line
  std::string scene_path = "";
  while (std::getline (scene_list_f, scene_path))
  {
    // Instantiate cloud
    pcl::PointCloud <pcl::PointXYZ>::Ptr cloud_ptr =
      pcl::PointCloud <pcl::PointXYZ>::Ptr (
        new pcl::PointCloud <pcl::PointXYZ> ());

    if (! boost::filesystem::exists (scene_path))
    {
      fprintf (stderr, "%sERROR: Scene file does not exist, "
        "skipping this one: %s%s\n", FAIL, scene_path.c_str (), ENDC);
      continue;
    }
 
    // Load scene cloud
    load_cloud_file (scene_path, cloud_ptr);
    printf ("Cloud size: %ld points\n", cloud_ptr->size ());
    printf ("Organized? %s\n", cloud_ptr->isOrganized () ? "true" : "false");


    //Eigen::MatrixXf raw_depth = Eigen::MatrixXf (cloud_ptr -> height,
    //  cloud_ptr -> width);
    cv::Mat raw_depth = cv::Mat::zeros (cloud_ptr -> height, cloud_ptr -> width,
      CV_32F);

    // Convert pcd to 2D matrix with raw depth values
    //   2D coordinates (u, v) are already in the pcl::PointCloud, since it's
    //   organized. Only need to fetch z directly from each point.
    // Ref .pcd file format and pcl::PointCloud are row-major:
    //   http://docs.pointclouds.org/1.5.1/classpcl_1_1_p_c_d_reader.html
    //   http://docs.pointclouds.org/trunk/classpcl_1_1_point_cloud.html
    for (int r = 0; r < cloud_ptr -> height; r ++)
    {
      for (int c = 0; c < cloud_ptr -> width; c ++)
      {
        //raw_depth (r, c) = cloud_ptr -> at (c, r).z;
        // TODO: z is all negative right now, need to find out why. FOr now, just take absolute var, to debug png outputted
        raw_depth.at <float> (r, c) = cloud_ptr -> at (c, r).z * -1.0;
      }
    }

    // Write raw depth values to file
    // TODO: This is no good, image files are all of integer format!!! That is
    //   why none of the PNG or PGM files had raw floating point depth, now I
    //   see... duh!! To make values on the same scale for all integer images,
    //   a max depth needs to be defined, and all images need to be scaled by
    //   that depth.
    //   But preferably the scaling is done at the CNN input step. I still
    //   prefer storing raw depth map. The question is how do I visualize it to
    //   make sure it looks right. I should scale it the same way as I would
    //   scale it at the CNN input, and inspect it to make sure it makes sense.
    std::vector <std::string> exts;
    splitext (scene_path, exts);
    std::string depth_path = exts [0] + "_view0.png";

    // Scale raw depth by max depth among all scenes, to get integer RGB value
    //   to save as images for manual inspection
    cv::Mat depth_img_channel = (raw_depth - MIN_DEPTH) / (MAX_DEPTH - MIN_DEPTH) * 255.0;
    float min_scaled = FLT_MAX, max_scaled = FLT_MIN;
    for (int r = 0; r < depth_img_channel.rows; r ++)
    {
      for (int c = 0; c < depth_img_channel.cols; c ++)
      {
        if (depth_img_channel.at <float> (r, c) < min_scaled)
          min_scaled = depth_img_channel.at <float> (r, c);
        if (depth_img_channel.at <float> (r, c) > max_scaled)
          max_scaled = depth_img_channel.at <float> (r, c);
      }
    }
    fprintf (stderr, "min_scaled %f, max_scaled %f\n", min_scaled, max_scaled);
    depth_img_channel.convertTo (depth_img_channel, CV_8UC1);

    // Duplicate the channel to 3 channels, to fit image formats for saving
    cv::Mat depth_img = cv::Mat::zeros (depth_img_channel.rows,
      depth_img_channel.cols, CV_8UC3);
    // Ref: https://stackoverflow.com/questions/3614163/convert-single-channle-image-to-3-channel-image-c-opencv
    cv::cvtColor (depth_img_channel, depth_img, CV_GRAY2BGR);

    // Ref API https://docs.opencv.org/2.4/modules/highgui/doc/reading_and_writing_images_and_video.html#imwrite
    cv::imwrite (depth_path, depth_img);
    printf ("%sWritten depth image to %s%s\n", OKCYAN, depth_path.c_str (),
      ENDC);

  }

  scene_list_f.close ();


  return 0;
}


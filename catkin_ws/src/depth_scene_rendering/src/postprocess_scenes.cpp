// Mabel Zhang
// 4 Sep 2018
//
// Convert .pcd files from BlenSor (outputted by scene_generation.py, which
//   calls scan_kinect.py), to 2D images, by scaling by constant min and max
//   depth values saved from BlenSor. Scaling by constants enable us to later
//   recover the raw depth values from the images, which have integer values.
// This is not possible with BlenSor's PGM image output, because the PGMs are
//   created by scaling the raw depth by the max depth in each individual
//   image, not a constant across all images. This means 1. the image values
//   are not comparable across images, and 2. raw depths cannot be recovered.
//
// Blender camera points toward -z, so point cloud depths are multiplied by -1.
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

// Custom
#include <util/ansi_colors.h>
#include <util/io_util.h>  // join_paths(), dirname(), basename(), splitext()
#include <util/pcd_util.h>  // load_cloud_file()
#include <depth_scene_rendering/depth_to_image.h>  // RawDepthConversion


// Parameters:
//   depth: returned by cv::Mat::depth ()
void inspect_image (cv::Mat & mat, double & min, double & max)
{
  min = DBL_MAX, max = DBL_MIN;

  int depth = mat.depth ();
  for (int r = 0; r < mat.rows; r ++)
  {
    for (int c = 0; c < mat.cols; c ++)
    {
      // Ref: https://stackoverflow.com/questions/26153143/check-if-an-opencv-matrix-is-of-floating-point-component-type
      if (depth == CV_32F)
      {
        if (mat.at <float> (r, c) < min)
          min = mat.at <float> (r, c);
        if (mat.at <float> (r, c) > max)
          max = mat.at <float> (r, c);
      }
      if (depth == CV_64F)
      {
        if (mat.at <double> (r, c) < min)
          min = mat.at <double> (r, c);
        if (mat.at <double> (r, c) > max)
          max = mat.at <double> (r, c);
      }
      else if (depth == CV_8U)
      {
        if (mat.at <char> (r, c) < min)
          min = mat.at <char> (r, c);
        if (mat.at <char> (r, c) > max)
          max = mat.at <char> (r, c);
      }
      else if (depth == CV_16U)
      {
        if (mat.at <unsigned int> (r, c) < min)
          min = mat.at <unsigned int> (r, c);
        if (mat.at <unsigned int> (r, c) > max)
          max = mat.at <unsigned int> (r, c);
      }
      else if (depth == CV_8S || depth == CV_16S || depth == CV_32S)
      {
        if (mat.at <int> (r, c) < min)
          min = mat.at <int> (r, c);
        if (mat.at <int> (r, c) > max)
          max = mat.at <int> (r, c);
      }
    }
  }
  fprintf (stderr, "min %g, max %g\n", min, max);
}


int main (int argc, char ** argv)
{
  // Get path of this package
  std::string pkg_path = ros::package::getPath ("depth_scene_rendering");

  RawDepthConversion converter = RawDepthConversion ();
  if (converter.getError ())
  {
    printf ("%sRawDepthConversion errored. Terminating program.%s\n",
      OKCYAN, ENDC);
    return 0;
  }


  /////
  // Convert raw depths in pcd to integers to save as images

  // Text file with list of .pcd scene names, written by scene_generation.py
  std::string scene_list_path = "";
  //join_paths (pkg_path, "config/scenes_noisy.txt", scene_list_path);
  join_paths (pkg_path, "config/scenes_test.txt", scene_list_path);
  std::ifstream scene_list_f (scene_list_path.c_str ());

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
    // NOTE: Blender camera's z-axis points behind camera. So all z's are
    //   negative. Multiply by -1 to get positive depth.
    flip_z (cloud_ptr);
    printf ("Cloud size: %ld points\n", cloud_ptr->size ());
    printf ("Organized? %s\n", cloud_ptr->isOrganized () ? "true" : "false");


    // Instantiate image with raw depths
    cv::Mat raw_depth = cv::Mat::zeros (cloud_ptr -> height, cloud_ptr -> width,
      CV_32F);

    float min_depth = FLT_MAX, max_depth = FLT_MIN;
    float min_neg_depth = FLT_MAX, max_neg_depth = FLT_MIN;

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
        raw_depth.at <float> (r, c) = cloud_ptr -> at (c, r).z;

        // DEBUG
        if (cloud_ptr -> at (c, r).z > max_depth)
          max_depth = cloud_ptr -> at (c, r).z;
        if (cloud_ptr -> at (c, r).z < min_depth)
          min_depth = cloud_ptr -> at (c, r).z;

        // DEBUG
        if (- cloud_ptr -> at (c, r).z > max_neg_depth)
          max_neg_depth = - cloud_ptr -> at (c, r).z;
        if (- cloud_ptr -> at (c, r).z < min_neg_depth)
          min_neg_depth = - cloud_ptr -> at (c, r).z;
      }
    }
    fprintf (stderr, "Cloud inspection (raw): min %g, max %g\n",
      min_depth, max_depth);

    double min_raw = 0.0, max_raw = 0.0;
    fprintf (stderr, "raw_depth inspection: ");
    inspect_image (raw_depth, min_raw, max_raw);

    // Path to output image
    std::vector <std::string> exts;
    splitext (scene_path, exts);
    std::string depth_path = exts [0] + "_view0.png";

    // Instantiate image with integers
    // Scale raw depth by max depth among all scenes, to get integer RGB value
    //   to save as images for manual inspection
    cv::Mat depth_img_channel = cv::Mat::zeros (raw_depth.rows, raw_depth.cols,
      CV_8UC1);
    converter.convert_depths_to_ints (raw_depth, depth_img_channel);

    // DEBUG. Inspect scaled image
    double min_scaled = 0.0, max_scaled = 0.0;
    fprintf (stderr, "depth_img_channel inspection: ");
    inspect_image (depth_img_channel, min_scaled, max_scaled);

    // Duplicate the channel to 3 channels, to fit image formats for saving
    cv::Mat depth_img = cv::Mat::zeros (depth_img_channel.rows,
      depth_img_channel.cols, CV_8UC3);
    // Ref: https://stackoverflow.com/questions/3614163/convert-single-channle-image-to-3-channel-image-c-opencv
    cv::cvtColor (depth_img_channel, depth_img, CV_GRAY2BGR);

    // Write converted integers image to file
    // API https://docs.opencv.org/2.4/modules/highgui/doc/reading_and_writing_images_and_video.html#imwrite
    cv::imwrite (depth_path, depth_img);
    printf ("%sWritten depth image to %s%s\n", OKCYAN, depth_path.c_str (),
      ENDC);


    // Sanity check: Make sure can recover RAW depths from images.
    // Convert image back to raw depths, see if depths match orig .pcd
    cv::Mat depth_converted = cv::Mat::zeros (depth_img_channel.rows,
      depth_img_channel.cols, CV_32F);
    converter.convert_ints_to_depths (depth_img_channel, depth_converted);

    double min_cvt = 0.0, max_cvt = 0.0;
    fprintf (stderr, "depth_converted inspection: ");
    inspect_image (depth_converted, min_cvt, max_cvt);

    // If all elts equal, all elts will == true, nonzero.
    // API https://docs.opencv.org/2.4/modules/core/doc/basic_structures.html#matrix-expressions
    cv::Mat match = (raw_depth - depth_converted < 1e-6);
    // NaN does not equal itself. If a != a, then it is nan.
    int nNans = cv::countNonZero (raw_depth != raw_depth);
    // Number of elements that match = num_matches + num_nans (nans never match)
    int nMatches = cv::countNonZero (match) + nNans;

    printf ("%d (should be %d) recovered depth values matched original (%d NaNs).\n",
      nMatches, depth_converted.rows * depth_converted.cols, nNans);
    if (nMatches == depth_converted.rows * depth_converted.cols)
      printf ("Converted depths == raw depths. "
        "Raw depths correctly recovered from converted image.\n");
    else
      printf ("Converted depths != raw depths! Find out why.\n");

    // DEBUG: Compare the two matrices
    if (nMatches != depth_converted.rows * depth_converted.cols)
    {
      for (int r = 0; r < depth_converted.rows; r ++)
      {
        for (int c = 0; c < depth_converted.cols; c ++)
        {
          if (! match.at <char> (r, c) && ! isnan (raw_depth.at <float> (r, c)))
            printf ("Mismatch: %f to %f\n", raw_depth.at <float> (r, c),
              depth_converted.at <float> (r, c));
        }
      }
    }
  }

  scene_list_f.close ();


  return 0;
}


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



class RawDepthConversion
{

private:

  float MIN_DEPTH, MAX_DEPTH;

  // Caller to constructor should call this to see if there was error loading
  //   camera configuration. If so, program should terminate.
  bool error_;


public:

  RawDepthConversion ()
  {
    error_ = false;

    // Get path of this package
    std::string pkg_path = ros::package::getPath ("depth_scene_rendering");


    ////////
    // Load file with paths to camera configuration, written by
    //   scene_generation.py
 
    // Load the file containing the path to camera intrinsics matrix
    std::string intrinsics_cfg_path = "";
    join_paths (pkg_path, "config/intrinsics_path.txt", intrinsics_cfg_path);
    std::ifstream intrinsics_cfg_f (intrinsics_cfg_path.c_str ());
    // Read the file
    std::string intrinsics_path = "";
    std::getline (intrinsics_cfg_f, intrinsics_path);
    intrinsics_cfg_f.close ();
 

    /////
    // Load camera intrinsics matrix, written by scene_generation.py

    // Works, but don't need it. Raw depths already saved in .pcd!
    /*
    if (! boost::filesystem::exists (intrinsics_path))
    {
      fprintf (stderr, "%sERROR: Camera intrinsics file does not exist: "
        "%s%s\n", FAIL, intrinsics_path.c_str (), ENDC);
      return 0;
    }
 
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
        printf ("%sERROR: error reading row %d of camera intrinsics matrix in %s. Stopping.%s\n", FAIL, row_i, intrinsics_path.c_str (), ENDC);
        break;
      }
 
      intrinsics.row (row_i) = Eigen::Vector3f (c1, c2, c3);
      row_i ++;
    }
 
    std::cout << "Loaded camera intrinsics: " << std::endl << intrinsics << std::endl;
    */
 
 
    /////
    // Read BlenSor Kinect min/max depth range that generated the .pcd scenes.
    //   File was written by scene_generation.py.
    // Significance:
    // This will be used to scale raw depth values to integer range [0, 255] to
    //   save as image files. Rescaling all images by this same range is
    //   important - it creates an absolute scale so that the raw depth can
    //   always be recovered from the images.
    //
    //   BlenSor's PGM output does not allow recovering the raw depths, `.`
    //   each image is rescaled to the image's own max depth, so the scale is
    //   relative within each image, values across images are not comparable.
 
    std::string depth_range_path;
    dirname (intrinsics_path, depth_range_path);
    join_paths (depth_range_path, "cam_depth_range.txt", depth_range_path);
 
    if (! boost::filesystem::exists (depth_range_path))
    {
      fprintf (stderr, "%sERROR: Camera depth range config file does not exist: "
        "%s%s\n", FAIL, depth_range_path.c_str (), ENDC);
      error_ = true;
      return;
    }
 
    printf ("Reading camera depth range from %s\n", depth_range_path.c_str ());
    std::ifstream depth_range_f (depth_range_path.c_str ());
 
    std::string row = "";
    float depth_range [2] = {0.0f, 0.0f};
    int range_row_i = 0;
    // Read each row of file
    while (std::getline (depth_range_f, row))
    {
      //printf ("%s\n", row.c_str ());
      std::istringstream iss (row);
 
      if (! (iss >> depth_range [range_row_i]))
      {
        printf ("%sERROR: error reading row %d of camera depth range in %s. "
          "Stopping.%s\n", FAIL, range_row_i, depth_range_path.c_str (), ENDC);
        break;
      }
 
      range_row_i ++;
    }
 
    MIN_DEPTH = depth_range [0];
    MAX_DEPTH = depth_range [1];
    printf ("%sGot Kinect range from configuration file: min %f, max %f.%s "
      "Will scale raw depths in .pcd files by this range to get integer image RGBs.\n",
      OKCYAN, MIN_DEPTH, MAX_DEPTH, ENDC);
 
    depth_range_f.close ();
  }

  bool getError ()
  {
    return error_;
  }


  // Convert raw floating point depth values to integers in range [0, 255], so
  //   can save as image format file.
  // Parameters:
  //   raw_depth: Input. 2D image of raw floating point depths.
  //   img: Output. Same size as input. Integer values
  void convert_depths_to_ints (cv::Mat & raw_depth, cv::Mat & img)
  {
    img = (raw_depth - MIN_DEPTH) / (MAX_DEPTH - MIN_DEPTH) * 255.0;
  }
  
  // Reverse operation of convert_depths_to_ints().
  // NOTE: If change formula in convert_depths_to_ints(), update this fn
  //   accordingly, so that this fn still recovers *raw depths*!!
  // Recover the 3D raw depths that the 2D image was converted from, using the
  //   Kinect min/max ranges saved in configuration file.
  // Parameters:
  //   img: Input. Integer values.
  //   raw_depth: Output. Raw floating point depth values.
  void convert_ints_to_depths (cv::Mat & img, cv::Mat raw_depth)
  {
    raw_depth = img / 255.0 * (MAX_DEPTH - MIN_DEPTH) + MIN_DEPTH;
  }

};


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
      else if (depth == CV_8U || depth == CV_16U)
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
    printf ("Cloud size: %ld points\n", cloud_ptr->size ());
    printf ("Organized? %s\n", cloud_ptr->isOrganized () ? "true" : "false");


    // Instantiate image with raw depths
    cv::Mat raw_depth = cv::Mat::zeros (cloud_ptr -> height, cloud_ptr -> width,
      CV_32F);

    float min_depth = FLT_MAX, max_depth = FLT_MIN;

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
        // TODO: z is all negative right now, need to find out why. For now,
        //   just take absolute var, to debug png outputted
        raw_depth.at <float> (r, c) = cloud_ptr -> at (c, r).z * -1.0;

        if (- cloud_ptr -> at (c, r).z > max_depth)
          max_depth = - cloud_ptr -> at (c, r).z;
        if (- cloud_ptr -> at (c, r).z < min_depth)
          min_depth = - cloud_ptr -> at (c, r).z;
      }
    }
    fprintf (stderr, "(TODO: NEGATIVE) Cloud inspection: min %g, max %g\n", min_depth, max_depth);

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

    /*
    // Convert type to ints
    depth_img_channel.convertTo (depth_img_channel, CV_8UC1);
    */

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

    // TODO: TEMPORARY, * -1 to reverse the temp compensation of negatve depth values in BlenSor scan
    depth_converted *= -1;

    // API https://docs.opencv.org/2.4/modules/core/doc/basic_structures.html#matrix-expressions
    if (cv::countNonZero (raw_depth == depth_converted) == 0)
      printf ("Converted depths == raw depths. "
        "Raw depths recovered from converted image.\n");
    else
      printf ("Converted depths != raw depths! Find out why.\n");
  }

  scene_list_f.close ();


  return 0;
}


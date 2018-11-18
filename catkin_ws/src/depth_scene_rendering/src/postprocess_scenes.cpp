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
#include <opencv2/core/eigen.hpp>

// Custom
#include <util/ansi_colors.h>
#include <util/io_util.h>  // join_paths(), dirname(), basename(), splitext()
#include <util/pcd_util.h>  // load_cloud_file()
#include <util/cv_util.h>  // project_3d_pts_to_2d()

// Local
#include <depth_scene_rendering/depth_to_image.h>  // RawDepthScaling, crop_image_center()
#include <depth_scene_rendering/camera_info.h>  // load_intrinsics(), load_nx4_matrix()
#include <depth_scene_rendering/postprocess_scenes.h>  // calc_object_pose_in_img()
#include <depth_scene_rendering/scene_yaml.h>


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
      else if (depth == CV_8U || depth == CV_8S)
      {
        if (mat.at <char> (r, c) < min)
          min = mat.at <char> (r, c);
        if (mat.at <char> (r, c) > max)
          max = mat.at <char> (r, c);
      }
      else if (depth == CV_16U)
      {
        if (mat.at <unsigned short> (r, c) < min)
          min = mat.at <unsigned short> (r, c);
        if (mat.at <unsigned short> (r, c) > max)
          max = mat.at <unsigned short> (r, c);
      }
      else if (depth == CV_16S)
      {
        if (mat.at <short> (r, c) < min)
          min = mat.at <short> (r, c);
        if (mat.at <short> (r, c) > max)
          max = mat.at <short> (r, c);
      }
      else if (depth == CV_32S)
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


// Project a point cloud to 2D depth image, using camera intrinsics matrix.
// Parameters:
//   scaler: Constants and methods to scale raw depth to integers, for images.
//     Constants scale all images by the same range, so scaled values across
//     all images are comparable.
//   P: Camera projection matrix
//   depth_path: Path to write image to
//   depth_img: Return value
void convert_pcd_to_image (RawDepthScaling & scaler,
  pcl::PointCloud <pcl::PointXYZ>::Ptr & cloud_ptr, Eigen::MatrixXf & P,
  const std::string & depth_path, cv::Mat & depth_img, bool save_img=true)
{
  bool DEBUG_CONVERSION = true;

  // Instantiate image with raw depths
  cv::Mat raw_depth = cv::Mat::zeros (cloud_ptr -> height, cloud_ptr -> width,
    CV_32F);

  float min_depth = FLT_MAX, max_depth = FLT_MIN;
  float min_neg_depth = FLT_MAX, max_neg_depth = FLT_MIN;


  // Project with intrinsics matrix. This makes object centered correctly.

  // 3 x n
  Eigen::MatrixXf pts = cloud_ptr -> getMatrixXfMap ().block (0, 0, 3,
    cloud_ptr -> size ());
  Eigen::MatrixXi uv;
  project_3d_pts_to_2d_homo (pts, P, uv);

  int n_nans = 0;

  for (int c = 0; c < uv.cols (); c ++)
  {
    // Empty point, no depth recorded
    if (isnan (pts (2, c)))
    {
      n_nans += 1;
      continue;
    }

    // (< 480, < 640): depth
    //fprintf (stderr, "(%d, %d): %g\n", uv (1, c), uv (0, c), pts (2, c));

    // cv::Mat indexes with (v, u). Eigen::MatrixXf indexes with (r, c)
    raw_depth.at <float> (uv (1, c), uv (0, c)) = pts (2, c);
  }


  double min_raw = 0.0, max_raw = 0.0;
  fprintf (stderr, "raw_depth inspection: ");
  inspect_image (raw_depth, min_raw, max_raw);

  // Instantiate image with integers
  // Scale raw depth by max depth among all scenes, to get integer RGB value
  //   to save as images for manual inspection
  cv::Mat depth_img_channel;
  scaler.scale_depths_to_ints (raw_depth, depth_img_channel);

  // DEBUG. Inspect scaled image
  double min_scaled = 0.0, max_scaled = 0.0;
  fprintf (stderr, "depth_img_channel inspection: ");
  inspect_image (depth_img_channel, min_scaled, max_scaled);

  // Return value
  // Duplicate the channel to 3 channels, to fit image formats for saving
  depth_img = cv::Mat::zeros (depth_img_channel.rows,
    depth_img_channel.cols, CV_8UC3);
  // Ref: https://stackoverflow.com/questions/3614163/convert-single-channle-image-to-3-channel-image-c-opencv
  cv::cvtColor (depth_img_channel, depth_img, CV_GRAY2BGR);

  // Write converted integers image to file
  // API https://docs.opencv.org/2.4/modules/highgui/doc/reading_and_writing_images_and_video.html#imwrite
  if (save_img)
  {
    cv::imwrite (depth_path, depth_img);
    fprintf (stderr, "%sWritten depth image to %s%s\n", OKCYAN,
      depth_path.c_str (), ENDC);
  }


  // Sanity check: Make sure can recover RAW depths from images.
  // Convert image back to raw depths, see if depths match orig .pcd
  if (DEBUG_CONVERSION)
  {
    fprintf (stderr, "Sanity check: checking if raw depths can be recovered from the integers.\n");

    cv::Mat depth_recovered = cv::Mat::zeros (depth_img_channel.rows,
      depth_img_channel.cols, CV_32F);
    scaler.scale_ints_to_depths (depth_img_channel, depth_recovered);
   
    double min_cvt = 0.0, max_cvt = 0.0;
    fprintf (stderr, "depth_recovered inspection: ");
    inspect_image (depth_recovered, min_cvt, max_cvt);
   
    // If all elts equal, all elts will == true, nonzero.
    // API https://docs.opencv.org/2.4/modules/core/doc/basic_structures.html#matrix-expressions
    cv::Mat match = (raw_depth - depth_recovered < 1e-6);
    // NaN does not equal itself. If a != a, then it is nan.
    int nNans = cv::countNonZero (raw_depth != raw_depth);
    // Number of elements that match = num_matches + num_nans (nans never match)
    int nMatches = cv::countNonZero (match) + nNans;
   
    fprintf (stderr, "%d (should be %d) recovered depth values matched original (%d NaNs).\n",
      nMatches, depth_recovered.rows * depth_recovered.cols, nNans);
    if (nMatches == depth_recovered.rows * depth_recovered.cols)
      fprintf (stderr, "Converted depths == raw depths. "
        "Raw depths correctly recovered from converted image.\n");
    else
      fprintf (stderr, "Converted depths != raw depths! Find out why.\n");
   
    // DEBUG: Compare the two matrices
    if (nMatches != depth_recovered.rows * depth_recovered.cols)
    {
      for (int r = 0; r < depth_recovered.rows; r ++)
      {
        for (int c = 0; c < depth_recovered.cols; c ++)
        {
          if (! match.at <char> (r, c) && ! isnan (raw_depth.at <float> (r, c)))
            fprintf (stderr, "Mismatch: %f to %f\n", raw_depth.at <float> (r, c),
              depth_recovered.at <float> (r, c));
        }
      }
    }
  }
}


int main (int argc, char ** argv)
{
  bool DISPLAY_IMAGES = false;

  // Dry run only counts how many point clouds are all NaNs, and prints them,
  //   so they can be manually removed from the scenes YAML file.
  bool DRY_RUN = false;

  // Parse cmd line args
  for (int i = 0; i < argc; i++)
  {
    if (! strcmp (argv [i], "--display"))
      DISPLAY_IMAGES = true;
    else if (! strcmp (argv [i], "--dry-run"))
      DRY_RUN = true;
  }

  // Set to false to save running time
  bool SAVE_UNCROP = false;


  // Get path of this package
  std::string pkg_path = ros::package::getPath ("depth_scene_rendering");

  RawDepthScaling scaler = RawDepthScaling ();
  if (scaler.getError ())
  {
    fprintf (stderr, "%sRawDepthScaling errored. Terminating program.%s\n",
      OKCYAN, ENDC);
    return 0;
  }


  /////
  // Convert raw depths in pcd to integers to save as images

  size_t start_time = time (NULL);

  // Text file with list of .pcd scene names, written by scene_generation.py
  std::string scene_list_path = "";
  join_paths (pkg_path, "config/scenes_noisy.yaml", scene_list_path);

  // Load 3 x 4 camera intrinsics matrix
  Eigen::MatrixXf P;
  load_intrinsics (P);

  // scenes.yaml file
  ScenesYaml scene_list_yaml = ScenesYaml (scene_list_path);
  std::vector <std::string> scene_paths;
  std::vector <std::string> empty_scenes;
  std::vector <std::string> empty_scene_objs;
  int n_imgs_saved = 0;
  // For each object
  for (int o_i = 0; o_i < scene_list_yaml.get_n_objects (); o_i++)
  {
    scene_list_yaml.get_scenes (o_i, scene_paths);

    // For each rendered scene of this object
    for (std::vector <std::string>::iterator s_it = scene_paths.begin ();
      s_it != scene_paths.end (); s_it++)
    {
      std::string scene_path = *s_it;
 
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
      // Account for Blender's camera frame in computer graphics convention, which
      //   has z flipped to -z (pointing behind camera), y flipped to -y. It is
      //   180 degrees off wrt x-axis from robotic perception convention.
      flip_yz (cloud_ptr);
      fprintf (stderr, "Cloud size: %ld points\n", cloud_ptr->size ());
      fprintf (stderr, "Organized? %s\n",
        cloud_ptr->isOrganized () ? "true" : "false");

      // Check how many NaNs are there. Don't actually remove them, `.` then
      //   the "density of the cloud will be lost". Call the dry run fn.
      //   Need to keep all the points `.` image width*height structure must
      //   be preserved in cloud!
      //   Ref: http://docs.pointclouds.org/trunk/group__filters.html#gac463283a9e9c18a66d3d29b28a575064
      std::vector <int> notNaNs_idx;
      pcl::removeNaNFromPointCloud (*cloud_ptr, notNaNs_idx);
      // Sanity check
      if (notNaNs_idx.size () == 0)
      {
        fprintf (stderr, "%sERROR: All points in cloud are NaNs. Rendering does not have object in frame. Skipping this cloud. You might want to remove this scene from YAML scene list.%s\n", FAIL, ENDC);
        empty_scenes.push_back (scene_path);
        empty_scene_objs.push_back (scene_list_yaml.get_object_name (o_i));
        continue;
      }
      if (DRY_RUN)
        continue;

      std::vector <std::string> exts;
      splitext (scene_path, exts);
      // Path to output image
      std::string depth_path = exts [0] + ".png";
 
      cv::Mat depth_img;
      convert_pcd_to_image (scaler, cloud_ptr, P, depth_path, depth_img,
        SAVE_UNCROP);
 
 
      // Find object center in image pixels
      Eigen::VectorXf p_obj_2d;
      // flip
      calc_object_pose_in_img (scene_path, P, p_obj_2d, depth_img.rows,
        depth_img.cols, true);
 
      // Crop image, without changing image center, so that intrinsics matrix
      //   still works with the raw depths!
      // NOTE after cropping, camera intrinsics / projection matrix will no
      //   longer work, `.` center of cropped image is different! Crop must be
      //   AFTER done using camera projection matrix.
      // TODO: Figure out a size that works for all objects.
      cv::Mat cropped;
      crop_image (depth_img, cropped, p_obj_2d[0], p_obj_2d[1],
        RawDepthScaling::CROP_W, RawDepthScaling::CROP_H, false);

      // Scale image
      // Use INTER_NEAREST to use actual values from image, no invented
      //   values from interpolation.
      cv::Mat scaled;
      cv::resize (cropped, scaled, cv::Size (RawDepthScaling::SCALE_H,
        RawDepthScaling::SCALE_W), 0, 0, cv::INTER_NEAREST);
 
      // Write converted integers image to file
      // API https://docs.opencv.org/2.4/modules/highgui/doc/reading_and_writing_images_and_video.html#imwrite
      std::string crop_path = exts [0] + "crop.png";
      cv::imwrite (crop_path, scaled);
      fprintf (stderr, "%sWritten cropped and scaled depth image to %s%s\n", OKCYAN,
        crop_path.c_str (), ENDC);
      n_imgs_saved += 1;
 
      if (DISPLAY_IMAGES)
      {
        cv::imshow ("Converted depth image", depth_img);
        cv::waitKey (0);
       
        cv::imshow ("Cropped and scaled", scaled);
        cv::waitKey (0);
      }
    }
  }

  fprintf (stderr, "Elapsed time %ld s\n", time (NULL) - start_time);

  fprintf (stderr, "%s%ld point cloud scenes had all points NaNs, you might want to delete these from YAML file, and delete the files altogether:%s\n",
    WARN, empty_scenes.size (), ENDC);
  for (int s_i = 0; s_i < empty_scenes.size (); s_i ++)
  {
    fprintf (stderr, "%s: %s\n", empty_scene_objs [s_i].c_str (),
      empty_scenes [s_i].c_str ());
  }

  fprintf (stderr, "%d cropped images saved\n", n_imgs_saved);

  return 0;
}


// Mabel Zhang
// 5 Sep 2018
//
// Load .pcd scene files outputted from BlenSor, by depth_scene_rendering
//   scene_generation.py.
// Given a contact point, test whether the point is in front of, or behind,
//   depth camera's reach (point cloud). If the point is behind at least one
//   point in the point cloud, it is considered occluded (behind). Otherwise,
//   (ray is in free space), it is unoccluded.
//
// Usage:
//   $ rosrun tactile_graspit_collection occlusion_test
//

// C++
#include <fstream>
#include <string>
#include <stdio.h>
#include <time.h>

// ROS
#include <ros/ros.h>
#include <ros/package.h>

// Eigen
#include <Eigen/Core>

// OpenCV
#include <opencv2/opencv.hpp>

// Custom packages
#include <util/io_util.h>  // join_paths ()
#include <util/pcd_util.h>
#include <util/pcl_raytrace_util.h>  // RayTracer
#include <util/ansi_colors.h>
#include <depth_scene_rendering/camera_info.h>  // load_intrinsics ()
#include <depth_scene_rendering/depth_to_image.h>  // RawDepthConversion


// Separate contact points into visible and occluded channels.
class OcclusionSeparation
{
private:

  RawDepthConversion converter_;


public:

  OcclusionSeparation ()
  {
  }

  // Caller should call this after calling constructor, to make sure camera
  //   configuration were loaded correctly.
  bool getError ()
  {
    return converter_.getError ();
  }

  // Separate visible and occluded contact points into two heatmaps.
  //   If ray goes through free space, it is in front of point cloud, or
  //     visible (green ray visualized in RViz by RayTracer).
  //   If ray goes through or hits the camera point cloud, it is occluded by
  //     the poiint cloud (red ray).
  //   Green = in front (visible)
  //   Red = behind (occluded, goes through a pt in point cloud).
  // Parameters:
  //   pts: 3 x n. Put points on columns, as opposed to rows, so that indexing
  //     is faster, `.` Eigen is column-major by default.
  //   visible, occluded: Indices of points, indicating whether the
  //     point is in front of or occluded by the point cloud.
  //   P: 3 x 4 camera projection / intrinsics matrix
  //          [fx'  0  cx' Tx]
  //      P = [ 0  fy' cy' Ty]
  //          [ 0   0   1   0]
  //      http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CameraInfo.html
  //   visible_uv, occluded_uv: Return values. Image coordinates (u, v) of
  //     3D points in pts parameter, projected onto image plane by P.
  void separate_and_project (Eigen::MatrixXf & pts,
    std::vector <bool> occluded,
    //std::vector <int> & visible, std::vector <int> & occluded,
    Eigen::MatrixXf & P,
    //Eigen::MatrixXf & visible_uv, Eigen::MatrixXf & occluded_uv)
    int height, int width,
    cv::Mat & visible_img, cv::Mat & occluded_img)
  {
    // Pseudocode, this function simply does this, but is a lot of code because
    //   there isn't slice-indexing with Eigen:
    // img_coords_occluded = P * pts [occluded]
    // img_coords_visible = P * pts [not occluded]


    // Make matrices homogeneous
    // Concatenate a row of 1s to bottom of matrix, using comma operator
    //   Ref: https://stackoverflow.com/questions/21496157/eigen-how-to-concatenate-matrix-along-a-specific-dimension
    // 4 x n
    Eigen::MatrixXf pts4 (4, pts.cols ());
    pts4 << pts, Eigen::MatrixXf::Ones (1, pts.cols ());

    // Project points into image plane, to find (u, v) image coords of the
    //   3D points.
    //   [u v w]' = P * [X Y Z 1]'
    //   x = u / w
    //   y = v / w
    //   http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CameraInfo.html
    // 3 x n = (3 x 4) * (4 x n)
    std::cerr << "pts:" << std::endl << pts4 << std::endl;
    Eigen::MatrixXf uv3 = P * pts4;
    std::cerr << "u, v, w:" << std::endl << uv3 << std::endl;
    // Divide by w, to get [u/w y/w 1]' in each column.
    // Replicate row to 3 rows, to be same dimension as divider
    uv3 = uv3.cwiseQuotient (uv3.row (2).replicate (uv3.rows (), 1));
    std::cerr << "u/w, y/w, 1:" << std::endl << uv3 << std::endl;
    // 2 x n. Eliminate 3rd row
    // NOTE: can't do uv3 = uv3.topRows (2), first column all 0s. Need to
    //   reallocate a new matrix.
    Eigen::MatrixXi uv = uv3.topRows (2).array ().round ().cast <int> ();
    std::cerr << "Final image coordinates:" << std::endl << uv << std::endl;


    /*
    // Separate visible and occluded points

    // Instantiate matrices of visible and occluded points
    //   Init to 3 x n, as opposed to n x 3, so that each column is a point,
    //     indexing gets faster `.` Eigen is column-major by default.
    // 2 x n_visible
    Eigen::MatrixXf visible_uv = Eigen::MatrixXf::Zero (2, visible.size ());
    // 2 x n_occluded
    Eigen::MatrixXf occluded_uv = Eigen::MatrixXf::Zero (2, occluded.size ());

    // Populate matrices of points, separating visible and occluded points
    // No slice indexing in Eigen default, other than in dev, so will use
    //   for-loop
    for (int i = 0; i < visible.size (); i ++)
    {
      visible_uv.col (i) = uv.col (visible.at (i));
    }
    for (int i = 0; i < occluded.size (); i ++)
    {
      occluded_uv.col (i) = uv.col (occluded.at (i));
    }
    */


/*
  }

  // Parameters:
  //   visible_uv, occluded_uv: Inputs. Returned from separate_and_project(),
  //     2D image coordinates (u, v) of visible and occluded points projected
  //     into the image plane.
  //   visible_img, occluded_img: Return values. 2D images of mostly 0s, with 1s
  //     at visible and occluded points.
  void create_heatmap (
    Eigen::MatrixXf & visible_uv, Eigen::MatrixXf & occluded_uv,
    int height, int width,
    cv::Mat & visible_img, cv::Mat & occluded_img)
  {
*/

    // Instantiate empty heat maps
    cv::Mat visible_f = cv::Mat::zeros (height, width, CV_32F);
    cv::Mat occluded_f = cv::Mat::zeros (height, width, CV_32F);

    // Set image coordinates corresponding to 3D points to raw depths
    for (int i = 0; i < occluded.size (); i ++)
    {
      // I(v, u) = depth z
      // Multiply by -1, to account for Blender camera facing -z.
      if (occluded.at (i) == false)
        visible_f.at <float> (uv (1, i), uv (0, i)) = -1.0 * pts (2, i); 
      else
        occluded_f.at <float> (uv (1, i), uv (0, i)) = -1.0 * pts (2, i); 

      fprintf (stderr, "%f (scaled to %d)\n", -1.0 * pts (2, i),
        converter_.convert_depth_to_int (-1.0 * pts (2, i)));
    }


    // Instantiate integer channels
    cv::Mat visible_ch = cv::Mat::zeros (height, width, CV_8UC1);
    cv::Mat occluded_ch = cv::Mat::zeros (height, width, CV_8UC1);

    // Scale raw depths by camera max depth, so scaled values are consistent
    //   across all images in the dataset.
    converter_.convert_depths_to_ints (visible_f, visible_ch);
    converter_.convert_depths_to_ints (occluded_f, occluded_ch);

    // Duplicate the channel to 3 channels, to fit image formats for saving
    // Ref: https://stackoverflow.com/questions/3614163/convert-single-channle-image-to-3-channel-image-c-opencv
    visible_img = cv::Mat::zeros (height, width, CV_8UC3);
    cv::cvtColor (visible_ch, visible_img, CV_GRAY2BGR);
    occluded_img = cv::Mat::zeros (height, width, CV_8UC3);
    cv::cvtColor (occluded_ch, occluded_img, CV_GRAY2BGR);
  }

};


int main (int argc, char ** argv)
{
  ros::init (argc, argv, "occlusion_test");
  ros::NodeHandle nh;

  // Random seed
  srand (time (NULL));

  OcclusionSeparation separator = OcclusionSeparation ();
  if (separator.getError ())
  {
    fprintf (stderr, "%sRawDepthConversion errored. Terminating program.%s\n",
      OKCYAN, ENDC);
    return 0;
  }


  // Get path of package
  std::string pkg_path = ros::package::getPath ("depth_scene_rendering");

  // Text file with list of .pcd scene names
  std::string noisy_scene_list_path = "";
  join_paths (pkg_path, "config/scenes_noisy.txt", noisy_scene_list_path);
  std::ifstream noisy_scene_list_f (noisy_scene_list_path.c_str ());

  // Octree resolution, in meters
  // 0.005 too large, some endpoints judged as occluded should be in front.
  float octree_res = 0.002;

  // Noise of randomly generated endpoint, in meters
  float noise_res = 0.01;

  // Read text file line by line. Each line is the path to a .pcd scene file
  std::string scene_path = "";
  // Ex. /media/master/Data_Ubuntu/courses/research/graspingRepo/train/visuotactile_grasping/2018-09-08-04-33-14_noisy00000.pcd
  while (std::getline (noisy_scene_list_f, scene_path))
  {
    // Instantiate cloud
    pcl::PointCloud <pcl::PointXYZ>::Ptr cloud_ptr =
      pcl::PointCloud <pcl::PointXYZ>::Ptr (
        new pcl::PointCloud <pcl::PointXYZ> ());
 
    // Load scene cloud
    load_cloud_file (scene_path, cloud_ptr);
    fprintf (stderr, "Cloud size: %ld points\n", cloud_ptr->size ());
    fprintf (stderr, "Organized? %s\n",
      cloud_ptr->isOrganized () ? "true" : "false");

    // Make octree to hold point cloud, for raytrace test
    // Ref: http://pointclouds.org/documentation/tutorials/octree.php
    RayTracer raytracer = RayTracer (cloud_ptr, octree_res, true, &nh);


    fprintf (stderr, "Testing ray-tracing...\n");
    // Origin of ray is always from camera center, 0 0 0.
    Eigen::Vector3f origin (0, 0, 0);


    // 1 m along z of camera frame, i.e. straight out of and normal to image
    //   plane.
    // Blender camera faces -z. So will shoot to -z.
    //Eigen::Vector3f endpoint (0, 0, -1);

    // Generate a number between 1 and 10
    int nPts = rand () % 10 + 1;
    fprintf (stderr, "Generating %d random points\n", nPts);

    int nVoxels = raytracer.get_n_voxels ();
    // Init to 3 x n, as opposed to n x 3, so that each point is on a column.
    //   Makes indexing run faster. Eigen is column-major by default.
    Eigen::MatrixXf endpoints = Eigen::MatrixXf::Zero (3, nPts);
    for (int i = 0; i < nPts; i ++)
    {
      // Generate a number bound by (0, 2 * size of number of voxels - 1), so
      //   there is 50% chance of picking a point in the point cloud.
      int idx = rand () % nVoxels;

      pcl::PointXYZ pt;
      raytracer.get_voxel (idx, pt);
      Eigen::Vector3f noisy_pt = Eigen::Vector3f (pt.x, pt.y, pt.z);

      // Add noise
      //std::cerr << "voxel: " << noisy_pt.transpose () << std::endl;
      // Random() returns float in range [-1, 1]
      Eigen::Vector3f noise = Eigen::Vector3f::Random () * noise_res;
      //std::cerr << "noise: " << noise.transpose () << std::endl;
      endpoints.col (i) = noisy_pt + noise;
      //std::cerr << "noisy_pt: " << endpoints.col (i).transpose () << std::endl;
    }

    // Do ray-trace occlusion test for each endpoint
    // Indices of occluded and visible endpoints
    //std::vector <int> occluded;
    //std::vector <int> visible;
    std::vector <bool> occluded;
    for (int i = 0; i < nPts; i ++)
    {
      //std::cerr << "Ray through " << endpoints.col (i).transpose () << std::endl;

      // Ray trace
      // Occluded = red arrow drawn in RViz, unoccluded = green
      // Must test endpoints one by one, not an n x 3 matrix, `.` octree
      //   getIntersectedVoxelCenters() only takes one ray at a time.
      bool curr_occluded = raytracer.raytrace_occlusion_test (origin,
        endpoints.col (i));
      occluded.push_back (curr_occluded);
      fprintf (stderr, "Occluded? %s\n", curr_occluded ? "true" : "false");

      /*
      if (curr_occluded)
        occluded.push_back (curr_occluded);
      else
        visible.push_back (curr_occluded);
      */

      // Debug
      //char enter;
      //std::cerr << "Press any character, then press enter: ";
      //std::cin >> enter;
    }


    // Load 3 x 4 camera intrinsics matrix
    Eigen::MatrixXf P;
    load_intrinsics (P);

    // Separate endpoints into visible and occluded, in pixel coordinates
    Eigen::MatrixXf visible_uv, occluded_uv;
    //separator.separate_and_project (endpoints, visible, occluded, P,
    //  visible_uv, occluded_uv);

    // Create heatmaps of visible and occluded points, in image plane.
    // In order to save images as images, esp convenient for debugging, images
    //   must be integers, 3 channels.
    cv::Mat visible_img, occluded_img;
    //separator.create_heatmap (visible_uv, occluded_uv, cloud_ptr -> height,
    //  cloud_ptr -> width, visible_img, occluded_img);

    separator.separate_and_project (endpoints, occluded, P,
      cloud_ptr -> height, cloud_ptr -> width, visible_img, occluded_img);

    // Display visible_img, occluded_img, to debug
    //cv::namedWindow ("Visible contacts", cv::WINDOW_AUTOSIZE);
    cv::Mat dst;
    cv::normalize (visible_img, dst, 0, 1, cv::NORM_MINMAX);
    cv::imshow ("Visible contacts", visible_img);
    //cv::namedWindow ("Occluded contacts", cv::WINDOW_AUTOSIZE);
    //cv::imshow ("Occluded contacts", occluded_img);
    // Press in the open window to close it
    cv::waitKey (0);

    // Save visible and occluded images
    std::vector <std::string> exts;
    splitext (scene_path, exts);
    std::string visible_path = exts [0];
    visible_path += "_vis.png";
    cv::imwrite (visible_path, visible_img);
    fprintf (stderr, "%sWritten visible heatmap to %s%s\n", OKCYAN,
      visible_path.c_str (), ENDC);

    std::string occluded_path = exts [0];
    occluded_path += "_occ.png";
    cv::imwrite (occluded_path, occluded_img);
    fprintf (stderr, "%sWritten occluded heatmap to %s%s\n", OKCYAN,
      occluded_path.c_str (), ENDC);
  }

  return 0;
}


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
#include <util/filter.h>  // blob_filter ()
#include <depth_scene_rendering/camera_info.h>  // load_intrinsics ()
#include <depth_scene_rendering/depth_to_image.h>  // RawDepthScaling, crop_image()
#include <util/cv_util.h>  // project_3d_to_2d()


// Separate contact points into visible and occluded channels.
class OcclusionSeparation
{
private:

  RawDepthScaling scaler_;


public:

  OcclusionSeparation ()
  {
  }

  // Caller should call this after calling constructor, to make sure camera
  //   configuration were loaded correctly.
  bool getError ()
  {
    return scaler_.getError ();
  }

  // Separate visible and occluded contact points into two heatmaps. Find image
  //   coordinates of the 3D points by camera intrinsics matrix P.
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
    std::vector <bool> occluded, Eigen::MatrixXf & P, int height, int width,
    cv::Mat & visible_img, cv::Mat & occluded_img)
  {
    // Pseudocode, this function simply does this, but is a lot of code because
    //   there isn't slice-indexing with Eigen:
    // img_coords_occluded = P * pts [occluded]
    // img_coords_visible = P * pts [not occluded]


    // Project points into image plane, to find (u, v) image coords of the
    //   3D points.
    //   [u v w]' = P * [X Y Z 1]'
    //          x = u / w
    //          y = v / w
    // Extract intrinsics matrix K from projection matrix P.
    // Multiplication result is (u, v, depth)
    // 2 x n
    Eigen::MatrixXi uv;
    project_3d_to_2d_homo (pts, P, uv);
    std::cerr << "Final image coordinates:" << std::endl << uv << std::endl;


    // Instantiate empty heat maps
    cv::Mat visible_f = cv::Mat::zeros (height, width, CV_32F);
    cv::Mat occluded_f = cv::Mat::zeros (height, width, CV_32F);
    int n_vis = 0, n_occ = 0;

    // Set image coordinates corresponding to 3D points to raw depths
    for (int i = 0; i < occluded.size (); i ++)
    {
      // I(v, u) = depth z
      if (occluded.at (i) == false)
      {
        n_vis += 1;
        visible_f.at <float> (uv (1, i), uv (0, i)) = pts (2, i); 
      }
      else
      {
        n_occ += 1;
        occluded_f.at <float> (uv (1, i), uv (0, i)) = pts (2, i); 
      }

      fprintf (stderr, "%f (scaled to %d)\n", pts (2, i),
        scaler_.scale_depth_to_int (pts (2, i)));
    }
    fprintf (stderr, "%d points visible, %d points occluded\n", n_vis, n_occ);


    // Instantiate integer channels
    cv::Mat visible_ch = cv::Mat::zeros (height, width, CV_8UC1);
    cv::Mat occluded_ch = cv::Mat::zeros (height, width, CV_8UC1);

    // Scale raw depths by camera max depth, so scaled values are consistent
    //   across all images in the dataset.
    scaler_.scale_depths_to_ints (visible_f, visible_ch);
    scaler_.scale_depths_to_ints (occluded_f, occluded_ch);

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

  bool DEBUG_RAYTRACE = false;
  bool VIS_RAYTRACE = false;

  // Random seed
  srand (time (NULL));

  OcclusionSeparation separator = OcclusionSeparation ();
  if (separator.getError ())
  {
    fprintf (stderr, "%sRawDepthScaling errored. Terminating program.%s\n",
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
    // Multiply by -1, to account for Blender camera facing -z.
    flip_z (cloud_ptr);
    fprintf (stderr, "Cloud size: %ld points\n", cloud_ptr->size ());
    fprintf (stderr, "Organized? %s\n",
      cloud_ptr->isOrganized () ? "true" : "false");

    // Make octree to hold point cloud, for raytrace test
    // Ref: http://pointclouds.org/documentation/tutorials/octree.php
    RayTracer raytracer = RayTracer (cloud_ptr, octree_res, VIS_RAYTRACE, &nh);


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

    if (DEBUG_RAYTRACE)
    {
      std::cerr << "endpoints: " << std::endl;
      std::cerr << endpoints << std::endl;
    }

    // Do ray-trace occlusion test for each endpoint
    std::vector <bool> occluded;
    for (int i = 0; i < nPts; i ++)
    {
      if (DEBUG_RAYTRACE)
        std::cerr << "Ray through " << endpoints.col (i).transpose () << std::endl;

      // Ray trace
      // Occluded = red arrow drawn in RViz, unoccluded = green
      // Must test endpoints one by one, not an n x 3 matrix, `.` octree
      //   getIntersectedVoxelCenters() only takes one ray at a time.
      bool curr_occluded = raytracer.raytrace_occlusion_test (origin,
        endpoints.col (i));
      occluded.push_back (curr_occluded);
      if (DEBUG_RAYTRACE)
        fprintf (stderr, "Occluded? %s\n", curr_occluded ? "true" : "false");

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

    // Create heatmaps of visible and occluded points, in image plane.
    // In order to save images as images, esp convenient for debugging, images
    //   must be integers, 3 channels.
    cv::Mat visible_img, occluded_img;

    separator.separate_and_project (endpoints, occluded, P,
      cloud_ptr -> height, cloud_ptr -> width, visible_img, occluded_img);

    // Save visible and occluded channels
    std::vector <std::string> exts;
    splitext (scene_path, exts);

    // Optional. Individual non-zero point. Saving because easier to test
    //   different parameters of blob, than to regenerate contact points and
    //   raytracing.
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


    // Crop the heat maps
    cv::Mat vis_crop, occ_crop;
    crop_image (visible_img, vis_crop, RawDepthScaling::CROP_W,
      RawDepthScaling::CROP_H);
    crop_image (occluded_img, occ_crop, RawDepthScaling::CROP_W,
      RawDepthScaling::CROP_H);


    // Blob the visible and occluded images, to create heatmaps. Save to file
    // In my visualize_dataset.py on adv_synth of dexnet, used BLOB_EXPAND=2,
    //   BLOB_GAUSS=0.5, for 32 x 32 images. Python gaussian function doesn't
    //   have size, only sigma.
    cv::Mat visible_blob, occluded_blob;
    // Use 31 for uncropped 640x480. 15 or smaller for cropped 100x100.
    int BLOB_EXPAND = 11;
    int GAUSS_SZ = 11;
    // Pass in 0 to let OpenCV calculating sigma from size
    float GAUSS_SIGMA = 0;

    std::string vis_blob_path = exts [0];
    vis_blob_path += "_vis_blob.png";
    //blob_filter (visible_img, visible_blob, BLOB_EXPAND, GAUSS_SZ, GAUSS_SIGMA);
    // Operate on the cropped img
    blob_filter (vis_crop, visible_blob, BLOB_EXPAND, GAUSS_SZ, GAUSS_SIGMA);
    cv::imwrite (vis_blob_path, visible_blob);
    fprintf (stderr, "%sWritten visible blobbed heatmap to %s%s\n", OKCYAN,
      vis_blob_path.c_str (), ENDC);

    std::string occ_blob_path = exts [0];
    occ_blob_path += "_occ_blob.png";
    //blob_filter (occluded_img, occluded_blob, BLOB_EXPAND, GAUSS_SZ, 
    // Operate on the cropped img
    blob_filter (occ_crop, occluded_blob, BLOB_EXPAND, GAUSS_SZ, 
      GAUSS_SIGMA);
    cv::imwrite (occ_blob_path, occluded_blob);
    fprintf (stderr, "%sWritten occluded blobbed heatmap to %s%s\n", OKCYAN,
      occ_blob_path.c_str (), ENDC);

    // Debug blob_filter()
    // Convert cv::Mat to std::vector
    // https://gist.github.com/mryssng/f43c9ae4cae13b204855e108a004c73a
    std::vector <float> vis_blob_vec;
    if (visible_blob.isContinuous())
    {
      vis_blob_vec.assign((uchar*)visible_blob.datastart, (uchar*)visible_blob.dataend);
    }
    else
    {
      for (int i = 0; i < visible_blob.rows; ++i)
      {
        vis_blob_vec.insert(vis_blob_vec.end(), visible_blob.ptr<uchar>(i), visible_blob.ptr<uchar>(i)+visible_blob.cols);
      }
    }

    // Debug blob_filter()
    // Find unique elts
    // https://stackoverflow.com/questions/1041620/whats-the-most-efficient-way-to-erase-duplicates-and-sort-a-vector
    //sort (vis_blob_vec.begin (), vis_blob_vec.end ());
    //vis_blob_vec.erase (unique (vis_blob_vec.begin (), vis_blob_vec.end ()),
    //  vis_blob_vec.end ());
    //fprintf (stderr, "%ld unique values in visible image:\n",
    //  vis_blob_vec.size ());
    //for (int i = 0; i < vis_blob_vec.size (); i ++)
    //{
    //  std::cerr << vis_blob_vec.at (i) << std::endl;
    //}


    // Display heatmaps to debug
    //cv::namedWindow ("Visible contacts", cv::WINDOW_AUTOSIZE);
    cv::Mat dst;
    //cv::normalize (visible_img, dst, 0, 1, cv::NORM_MINMAX);
    //cv::imshow ("Visible contacts", visible_img);
    //cv::normalize (visible_blob, dst, 0, 1, cv::NORM_MINMAX);
    // TODO: Actually the images written to file do look blurred! These displayed versions have sharp edges for some reason. Use inspect_channels.py to inspect with matplotlib, better visualization.
    cv::imshow ("Visible contacts", visible_blob);
    cv::waitKey (0);

    //cv::namedWindow ("Occluded contacts", cv::WINDOW_AUTOSIZE);
    //cv::normalize (occluded_img, dst, 0, 1, cv::NORM_MINMAX);
    //cv::imshow ("Occluded contacts", occluded_img);
    //cv::normalize (occluded_blob, dst, 0, 1, cv::NORM_MINMAX);
    cv::imshow ("Occluded contacts", occluded_blob);
    // Press in the open window to close it
    cv::waitKey (0);
  }

  return 0;
}


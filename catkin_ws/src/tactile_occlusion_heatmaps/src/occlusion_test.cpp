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
//   $ rosrun tactile_occlusion_heatmaps occlusion_test [--display] [--vis]
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
#include <util/cv_util.h>  // project_3d_pts_to_2d()
#include <util/eigen_util.h>  // load_csv_to_Eigen(), append_homogeneous_row()

// Local
#include <depth_scene_rendering/camera_info.h>  // load_intrinsics ()
#include <depth_scene_rendering/depth_to_image.h>  // RawDepthScaling, crop_image_center()
#include <depth_scene_rendering/postprocess_scenes.h>  // calc_object_pose_in_img(), calc_object_pose_wrt_cam()
#include <depth_scene_rendering/scene_yaml.h>  // ScenesYaml
#include <tactile_graspit_collection/contacts_io.h>
#include <tactile_graspit_collection/config_paths.h>  // PathConfigYaml


// Separate contact points into visible and occluded channels.
class OcclusionSeparation
{
private:

  RawDepthScaling scaler_;

  const bool DEBUG_PROJECT = false;


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
  //   pts: 3 x n. In camera frame. Put points on columns, as opposed to rows,
  //     so that indexing is faster, `.` Eigen is column-major by default.
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
    project_3d_pts_to_2d_homo (pts, P, uv);
    // Flip x and y. postprocess_scenes.h project_3d_pose_to_2d() does this,
    //   but cv_util.h project_3d_pts_to_2d() doesn't. TODO do something about
    //   this. Make it cleaner and less ad-hoc.
    uv.row (0) = width - uv.row (0).array ();
    uv.row (1) = height - uv.row (1).array ();
    if (DEBUG_PROJECT)
      std::cerr << "Final image coordinates:" << std::endl << uv << std::endl;


    // Instantiate empty heat maps
    cv::Mat visible_f = cv::Mat::zeros (height, width, CV_32F);
    cv::Mat occluded_f = cv::Mat::zeros (height, width, CV_32F);
    int n_vis = 0, n_occ = 0;

    // Set image coordinates, corresponding to 3D points, to raw depths
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

      if (DEBUG_PROJECT)
        fprintf (stderr, "Raw depth in meters: %f (scaled to %d integer)\n",
          pts (2, i), scaler_.scale_depth_to_int (pts (2, i)));
    }
    fprintf (stderr, "%sVisible: %d points. Occluded: %d points%s\n", OKCYAN,
      n_vis, n_occ, ENDC);
    std::cerr << "Contact points in 2D pixels:" << std::endl;
    std::cerr << uv << std::endl;


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


// Generate random points in the cloud, then add noise that may possibly put
//   the point off the cloud.
void generate_random_endpoints (int max_pts, RayTracer & raytracer,
  Eigen::MatrixXf & endpoints)
{
  // Noise of randomly generated endpoint, in meters
  float noise_res = 0.01;

  // Generate a number between 1 and 10
  int nPts = rand () % max_pts + 1;
  fprintf (stderr, "Generating %d random points\n", nPts);

  int nVoxels = raytracer.get_n_voxels ();
  // Init to 3 x n, as opposed to n x 3, so that each point is on a column.
  //   Makes indexing run faster. Eigen is column-major by default.
  endpoints = Eigen::MatrixXf::Zero (3, nPts);
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
}



int main (int argc, char ** argv)
{
  ros::init (argc, argv, "occlusion_test");
  ros::NodeHandle nh;

  bool DEBUG_RAYTRACE = false;
  bool VIS_RAYTRACE = false;

  bool GEN_RAND_PTS = false;

  bool DISPLAY_IMAGES = false;


  // Parse cmd line args
  for (int i = 0; i < argc; i++)
  {
    if (! strcmp (argv [i], "--display"))
      DISPLAY_IMAGES = true;
    else if (! strcmp (argv [i], "--vis"))
      VIS_RAYTRACE = true;
  }


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
  std::string scene_pkg_path = ros::package::getPath ("depth_scene_rendering");

  // Text file with list of .pcd scene names
  std::string scene_list_path = "";
  join_paths (scene_pkg_path, "config/scenes_noisy.yaml", scene_list_path);


  // Read config file for paths
  std::string tac_pkg_path = ros::package::getPath (
    "tactile_occlusion_heatmaps");
  std::string config_path;
  join_paths (tac_pkg_path, "config/paths.yaml", config_path);

  // Get contacts directory
  PathConfigYaml config = PathConfigYaml (config_path);
  std::string contacts_dir;
  config.get_contacts_path (contacts_dir);


  // Octree resolution, in meters
  // 0.005 too large, some endpoints judged as occluded should be in front.
  float octree_res = 0.002;

  // Load 3 x 4 camera intrinsics matrix, to project 3D to 2D
  Eigen::MatrixXf P;
  load_intrinsics (P);

  // scenes.txt file
  // Read text file line by line. Each line is the path to a .pcd scene file
  //std::ifstream scene_list_f (scene_list_path.c_str ());
  //std::string scene_path = "";
  // Ex. /media/master/Data_Ubuntu/courses/research/graspingRepo/train/visuotactile_grasping/2018-09-08-04-33-14_noisy00000.pcd
  //while (std::getline (scene_list_f, scene_path))

  // scenes.yaml file
  //   Tells what object is in each scene file!
  //   Outer loop around object, load grasps using contacts_io.h, and
  //     load the .pcd files for different views of each object.
  ScenesYaml scene_list_yaml = ScenesYaml (scene_list_path);
  std::vector <std::string> scene_paths;
  // For each object
  for (int o_i = 0; o_i < scene_list_yaml.get_n_objects (); o_i++)
  {
    // Load contact points for this object
    Eigen::MatrixXf contacts_O;
    Eigen::MatrixXf contacts_O4;
    Eigen::MatrixXf contacts_meta;
    int n_grasps = 0;
    if (! GEN_RAND_PTS)
    {
      std::string obj_cont_path;
      join_paths (contacts_dir, scene_list_yaml.get_object_name (o_i) + ".csv",
        obj_cont_path);
     
      fprintf (stderr, "%sLoading object contacts %s%s\n", OKCYAN,
        obj_cont_path.c_str (), ENDC);
      // n x 3. T^o, in object frame
      Eigen::MatrixXf contacts_O = load_csv_to_Eigen <Eigen::MatrixXf> (
        obj_cont_path);
      // n x 4. Must use temp var, A = A.transpose() gets runtime error
      Eigen::MatrixXf temp;
      append_homogeneous_col (contacts_O, temp);
      // 4 x n
      contacts_O4 = temp.transpose ();


      // Load contacts per individual grasp.  _meta.csv tells how many elements
      //   to index the contacts_O matrix for each individual grasp, i.e.
      std::string obj_meta_path;
      join_paths (contacts_dir,
        scene_list_yaml.get_object_name (o_i) + "_meta.csv", obj_meta_path);

      // 1 x nGrasps array
      fprintf (stderr, "%sLoading object contacts meta %s%s\n", OKCYAN,
        obj_meta_path.c_str (), ENDC);
      contacts_meta = load_csv_to_Eigen <Eigen::MatrixXf> (
        obj_meta_path);

      // For each grasp, render all the scenes. At the end have
      //   nGrasps x nScenes training samples
      n_grasps = contacts_meta.cols ();
    }
    // Else just generate one set of random points per scene. At the end have
    //   1 x nScenes training samples.
    else
      n_grasps = 1;


    // For each grasp of this object
    int curr_contact_start_idx = 0;
    for (int g_i = 0; g_i < n_grasps; g_i++)
    {
      fprintf (stderr, "%sGrasp [%d] out of %d%s\n", OKCYAN, g_i, n_grasps,
        ENDC);

      Eigen::MatrixXf curr_grasp_contacts;
      int n_contacts = 0;
      if (! GEN_RAND_PTS)
      {
        // Number of contacts in this grasp
        n_contacts = contacts_meta (g_i);

        // Index contacts for the current grasps.
        // Syntax block(i, j, rows, cols).
        // 4 x nContacts. 4 x 0 if no contacts
        // [:, grasp_start : grasp_start + n_contacts_at_this_grasp]
        // Coordinates are in object frame. Same points in object frame for
        //   all camera scenes. Only coordinates in camera frame are different
        //   across scenes.
        curr_grasp_contacts = contacts_O4.block (0,
          curr_contact_start_idx, contacts_O4.rows (), n_contacts);
      }

      std::cerr << "curr_contact_start_idx: " << curr_contact_start_idx
        << std::endl;
      std::cerr << "n_contacts: " << n_contacts << std::endl;
      std::cerr << "contacts: " << std::endl;
      std::cerr << curr_grasp_contacts << std::endl;


      // Get scenes of current object, rendered at different camera angles
      scene_list_yaml.get_scenes (o_i, scene_paths);
     
      // For each rendered scene of this object
      for (std::vector <std::string>::iterator s_it = scene_paths.begin ();
        s_it != scene_paths.end (); s_it++)
      {
        fprintf (stderr, "%sScene %ld out of %ld%s\n", OKCYAN,
          s_it - scene_paths.begin () + 1, scene_paths.size (), ENDC);

        std::string scene_path = *s_it;
     
        // Instantiate cloud
        pcl::PointCloud <pcl::PointXYZ>::Ptr cloud_ptr =
          pcl::PointCloud <pcl::PointXYZ>::Ptr (
            new pcl::PointCloud <pcl::PointXYZ> ());
       
        // Load scene cloud
        load_cloud_file (scene_path, cloud_ptr);
        // Multiply y and z by -1, to account for Blender camera facing -z.
        flip_yz (cloud_ptr);
        //fprintf (stderr, "Cloud size: %ld points\n", cloud_ptr->size ());
        //fprintf (stderr, "Organized? %s\n",
        //  cloud_ptr->isOrganized () ? "true" : "false");
       
        // Make octree to hold point cloud, for raytrace test
        // Ref: http://pointclouds.org/documentation/tutorials/octree.php
        RayTracer raytracer = RayTracer (cloud_ptr, octree_res, VIS_RAYTRACE,
          &nh);
       
       
        fprintf (stderr, "Ray-tracing...\n");
        // Origin of ray is always from camera center, 0 0 0.
        Eigen::Vector3f origin (0, 0, 0);
       
       
        // 1 m along z of camera frame, i.e. straight out of and normal to image
        //   plane.
        // Blender camera faces -z. So will shoot to -z.
        //Eigen::Vector3f endpoint (0, 0, -1);
       
        // 3 x n, in camera frame
        Eigen::MatrixXf endpoints;
     
        // Generate random endpoints to raytrace through
        // These are in camera frame, `.` points are generated by raytracer, which
        //   comes from pcd point cloud, captured in camera frame.
        if (GEN_RAND_PTS)
        {
          generate_random_endpoints (10, raytracer, endpoints);
        }
        // Transform grasp contacts that are in object frame to be in camera
        //   frame
        else
        {
          // Get object pose wrt camera frame. This is the inverse of extrinsics
          //   matrix, which is saved as camera pose wrt object frame.
          // 4 x 4
          Eigen::MatrixXf T_c_o;
          calc_object_pose_wrt_cam (scene_path, P, T_c_o,
            cloud_ptr -> height, cloud_ptr -> width);
//TODO: Do I need to flip x and y here? The 2D version would flip them here.
// This doesn't work, because T_c_o translation is just 0 0 1! Flipping 0 gives 0
// Actually the problem might be that, object frame != world frame. Object frame
//   has z pointing sideways, while world has z pointing up. I thought I saved
//   the camera extrinsics wrt object frame in scene_generation.py though.
//   0 0 1 is only position, look at the 3 x 3 R part of T_c_o, is it correct?
//   Can convert to Quaternion and check.
// Maybe that is why when camera points down, the matrix is not identity!!!???
//   because the matrix of camera orientation wrt object orientation is simply
//   not identity!!!! So my "correction" of pi wrt y-axis in scene_generation.py
//   might have been spurious!!!
// > Problem is that Blender rotates OBJ files by 90 degs wrt x by default!!!
//   So T^W_o was identity, when in fact it isn't!!! That threw off my camera
//   extrinsics matrix wrt object frame, obtained in Blender. So the camera
//   extrinsics applied to the contact coordinates in GraspIt, to transform
//   them from object frame to camera frame, was incorrect.
//   Now changed the loading to Y Forward, Z up, in scene_generation.py, and
//   reran everything. now correct!!!
  //T_c_o (0, 3) = -T_c_o (0, 3);
  //T_c_o (1, 3) = -T_c_o (1, 3);

          // No contacts in a grasp means the grasp is terrible, didn't make any
          //   contact with object. For now, keep them, because simply output
          //   poor grasp quality. TODO Decide whether to keep for final run
          Eigen::MatrixXf contacts_C;
          if (n_contacts == 0)
            // 4 x 0
            contacts_C = curr_grasp_contacts;
          else
          { 
            // Transform contacts from object frame to camera frame, using
            //   camera extrinsics matrix wrt object frame, T^c_o.
            // T^c = T^c_o * T^o
            //     = (T^o_c)^-1 * T_o
            // 4 x n = (4 x 4) * (4 x n)
            contacts_C = T_c_o * curr_grasp_contacts;
          }
          endpoints = contacts_C.topRows (3);

          std::cerr << "Object center in camera frame: " << std::endl;
          Eigen::Vector4f origin;
          origin << 0, 0, 0, 1;
          std::cerr << T_c_o * origin << std::endl;
        }

        if (DEBUG_RAYTRACE)
        {
          std::cerr << "endpoints in camera frame: " << std::endl;
          std::cerr << endpoints << std::endl;
        }

     
        // Do ray-trace occlusion test for each endpoint
        std::vector <bool> occluded;
        for (int p_i = 0; p_i < endpoints.cols (); p_i++)
        {
          if (DEBUG_RAYTRACE)
            std::cerr << "Ray through " << endpoints.col (p_i).transpose () << std::endl;

          // Ray trace
          // Occluded = red arrow drawn in RViz, unoccluded = green
          // Must test endpoints one by one, not an n x 3 matrix, `.` octree
          //   getIntersectedVoxelCenters() only takes one ray at a time.
          bool curr_occluded = raytracer.raytrace_occlusion_test (origin,
            endpoints.col (p_i));
          occluded.push_back (curr_occluded);
          if (DEBUG_RAYTRACE)
            fprintf (stderr, "Occluded? %s\n", curr_occluded ? "true" : "false");
       
          // Debug
          //char enter;
          //std::cerr << "Press any character, then press enter: ";
          //std::cin >> enter;
        }
       
       
        // Separate endpoints into visible and occluded, in pixel coordinates
        Eigen::MatrixXf visible_uv, occluded_uv;
       
        // Create heatmaps of visible and occluded points, in image plane.
        // In order to save images as images, esp convenient for debugging, images
        //   must be integers, 3 channels.
        cv::Mat visible_img, occluded_img;
        separator.separate_and_project (endpoints, occluded, P,
          cloud_ptr -> height, cloud_ptr -> width, visible_img, occluded_img);
       
       
        // Find object center in image pixels, using camera extrinsics
        Eigen::VectorXf p_obj_2d;
        calc_object_pose_in_img (scene_path, P, p_obj_2d, visible_img.rows,
          visible_img.cols, true);
       
        // Crop the heatmaps
        // NOTE after cropping, camera intrinsics / projection matrix will no
        //   longer work, `.` center of cropped image is different! Crop must be
        //   AFTER done using camera projection matrix.
        cv::Mat vis_crop, occ_crop;
        crop_image (visible_img, vis_crop, p_obj_2d[0], p_obj_2d[1],
          RawDepthScaling::CROP_W, RawDepthScaling::CROP_H, false);
        crop_image (occluded_img, occ_crop, p_obj_2d[0], p_obj_2d[1],
          RawDepthScaling::CROP_W, RawDepthScaling::CROP_H, false);
       
        // TODO: Resize to 32 x 32, before adding blob. Then decrease blob size
        //   to suit 32 x 32 images.
       
        // Save visible and occluded channels
        std::vector <std::string> exts;
        splitext (scene_path, exts);
       
        // Optional. Individual non-zero point. Saving because easier to test
        //   different parameters of blob, than to regenerate contact points and
        //   raytracing.
        /*
        std::string visible_path = exts [0];
        visible_path += "_g" + std::to_string (g_i) + "_vis.png";
        cv::imwrite (visible_path, vis_crop);  //visible_img);
        fprintf (stderr, "%sWritten visible heatmap to %s%s\n", OKCYAN,
          visible_path.c_str (), ENDC);
       
        std::string occluded_path = exts [0];
        occluded_path += "_g" + std::to_string (g_i) + "_occ.png";
        cv::imwrite (occluded_path, occ_crop);  //occluded_img);
        fprintf (stderr, "%sWritten occluded heatmap to %s%s\n", OKCYAN,
          occluded_path.c_str (), ENDC);
        */

       
        // Blob the visible and occluded images, to create heatmaps. Save to
        //   file.
        // In my visualize_dataset.py on adv_synth of dexnet, used
        //   BLOB_EXPAND=2, BLOB_GAUSS=0.5, for 32 x 32 images. Python gaussian
        //   function doesn't have size, only sigma.
        cv::Mat visible_blob, occluded_blob;
        // Use 31 for uncropped 640x480. 15 or smaller for cropped 100x100.
        int BLOB_EXPAND = 9;
        int GAUSS_SZ = 9;
        // Pass in 0 to let OpenCV calculating sigma from size
        float GAUSS_SIGMA = 0;
       
        std::string vis_blob_path = exts [0];
        vis_blob_path += "_g" + std::to_string (g_i) + "_vis_blob.png";
        //blob_filter (visible_img, visible_blob, BLOB_EXPAND, GAUSS_SZ, GAUSS_SIGMA);
        // Operate on the cropped img
        blob_filter (vis_crop, visible_blob, BLOB_EXPAND, GAUSS_SZ, GAUSS_SIGMA);
        cv::imwrite (vis_blob_path, visible_blob);
        fprintf (stderr, "%sWritten visible blobbed heatmap to %s%s\n", OKCYAN,
          vis_blob_path.c_str (), ENDC);
       
        std::string occ_blob_path = exts [0];
        occ_blob_path += "_g" + std::to_string (g_i) + "_occ_blob.png";
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


        if (DISPLAY_IMAGES)
        {
          // Display heatmaps to debug
          //cv::namedWindow ("Visible contacts", cv::WINDOW_AUTOSIZE);
          cv::Mat dst;
          //cv::normalize (visible_img, dst, 0, 1, cv::NORM_MINMAX);
          //cv::imshow ("Visible contacts", visible_img);
          //cv::normalize (visible_blob, dst, 0, 1, cv::NORM_MINMAX);
          // These displayed versions have sharp edges for some reason. Actual image
          //   does show Gaussian blurred. Use inspect_channels.py to inspect, and
          //   visualize_heatmaps.py to visualize heatmap overlay on depth image.
          std::cerr << "Displaying visible heatmap\n";
          cv::imshow ("Visible contacts", visible_blob);
          cv::waitKey (0);
        
          //cv::namedWindow ("Occluded contacts", cv::WINDOW_AUTOSIZE);
          //cv::normalize (occluded_img, dst, 0, 1, cv::NORM_MINMAX);
          //cv::imshow ("Occluded contacts", occluded_img);
          //cv::normalize (occluded_blob, dst, 0, 1, cv::NORM_MINMAX);
          std::cerr << "Displaying occluded heatmap\n";
          cv::imshow ("Occluded contacts", occluded_blob);
          // Press in the open window to close it
          cv::waitKey (0);
        }
      }

      curr_contact_start_idx += n_contacts;
    }
  }

  return 0;
}


#ifndef _DEPTH_TO_IMAGE_H_
#define _DEPTH_TO_IMAGE_H_

// Mabel Zhang
// 10 Sep 2018
//
// Refactored from ../../src/postprocess_scenes.cpp
//

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


  int convert_depth_to_int (float depth)
  {
    return (int) (round ((depth - MIN_DEPTH) / (MAX_DEPTH - MIN_DEPTH) * 255.0));
  }

  float convert_int_to_depth (int gray)
  {
    return gray / 255.0 * (MAX_DEPTH - MIN_DEPTH) + MIN_DEPTH;
  }

  // Convert raw floating point depth values to integers in range [0, 255], so
  //   can save as image format file.
  // Parameters:
  //   raw_depth: CV_32F. Input. 2D image of raw floating point depths.
  //   img: CV_8UC1. Output. Same size as input. Integer values
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

#endif

#!/usr/bin/env python

# Mabel Zhang
# 18 Sep 2018
#
# Python counterpart of ../../depth_scene_rendering/include/depth_scene_rendering/depth_to_image.h
#

# Python
import os

# ROS
import rospkg

# Custom
from util.ansi_colors import ansi_colors


class RawDepthScaling:

  MIN_DEPTH = -1
  MAX_DEPTH = -1


  # Equivalent to C++ version RawDepthScaling ctor in ../../depth_scene_rendering/include/depth_scene_rendering/depth_to_image.h
  def load_depth_range (self):

    # Read BlenSor Kinect min#max depth range that generated the .pcd scenes.
    #   File was written by scene_generation.py.
    # Significance:
    # This will be used to scale raw depth values to integer range [0, 255] to
    #   save as image files. Rescaling all images by this same range is
    #   important - it creates an absolute scale so that the raw depth can
    #   always be recovered from the images.
    #
    #   BlenSor's PGM output does not allow recovering the raw depths, `.`
    #   each image is rescaled to the image's own max depth, so the scale is
    #   relative within each image, values across images are not comparable.
 
    pkg_path = rospkg.RosPack ().get_path ('depth_scene_rendering')
    intrinsics_cfg_path = os.path.join (pkg_path, "config/intrinsics_path.txt")
    with open (intrinsics_cfg_path, 'rb') as intrinsics_cfg_f:
      intrinsics_path = intrinsics_cfg_f.read ()
 
    depth_range_path = os.path.join (os.path.dirname (intrinsics_path), 
      'cam_depth_range.txt')
    if not os.path.exists (depth_range_path):
      print ('%sERROR: Camera depth range config file does not exist: %s%s' %
        ansi_colors.FAIL, depth_range_path, ansi_colors.ENDC)
      return False, None
 
 
    print ('Reading camera depth range from %s' % depth_range_path)
    depth_range = []
    with open (depth_range_path, 'rb') as depth_range_f:
      for row in depth_range_f:

        try:
          depth_range.append (float (row.strip ()))
        except:
          print ('%sERROR: error reading row %d of camera depth range in %s. Stopping.%s' % (
            ansi_colors.FAIL, range_row_i, depth_range_path, ansi_colors.ENDC))
          return False, None
 
    print ('%sGot Kinect range from configuration file: min %f, max %f.%s Will recover raw depths from scaled depths in .png files by this range.' % (
      ansi_colors.OKCYAN, depth_range [0], depth_range [1], ansi_colors.ENDC))

    self.MIN_DEPTH = depth_range [0]
    self.MAX_DEPTH = depth_range [1]
 
    return True, depth_range


  # Equivalent to C++ version in ../../depth_scene_rendering/include/depth_scene_rendering/depth_to_image.h
  def scale_ints_to_depths (self, gray):

    return gray / 255.0 * (self.MAX_DEPTH - self.MIN_DEPTH) + self.MIN_DEPTH


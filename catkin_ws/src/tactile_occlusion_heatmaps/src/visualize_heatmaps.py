#!/usr/bin/env python

# Mabel Zhang
# 18 Sep 2018
#
# Visualize depth image with tactile overlay.
#
# Usage:
#   $ rosrun tactile_occlusion_heatmaps visualize_heatmaps.py
#

# Python
import os

# ROS
import rospkg

# Mabel: For running on GPU cluster remotely, which does not have python-tk
import socket
if socket.gethostname () != 'snaefellsjokull':
  # Ref: https://stackoverflow.com/questions/4930524/how-can-i-set-the-backend-in-matplotlib-in-python
  import matplotlib
  matplotlib.use ('Agg')
import matplotlib.pyplot as plt

# Custom packages
from util.ansi_colors import ansi_colors
from util.image_util import np_from_depth, show_image, matshow_image

# Local
from config_paths import get_heatmap_raw_fmt, get_heatmap_blob_fmt, get_vis_path
from depth_to_image import RawDepthScaling



def main ():

  pkg_path = rospkg.RosPack ().get_path ('depth_scene_rendering')
  scene_list_path = os.path.join (pkg_path, "config/scenes_noisy.txt")
  scene_list_f = open (scene_list_path, 'rb')

  vis_fmt, occ_fmt = get_heatmap_blob_fmt ()
  #vis_fmt, occ_fmt = get_heatmap_raw_fmt ()

  scaler = RawDepthScaling ()
  success, depth_range = scaler.load_depth_range ()
  if not success:
    return
  MIN_DEPTH = depth_range [0]
  MAX_DEPTH = depth_range [1]

  vis_path = get_vis_path ()


  for scene_path in scene_list_f:

    print ('Loading triplet files for %s' % scene_path)

    depth_im = np_from_depth (os.path.splitext (scene_path) [0] + 'crop.png')
    vis_im = np_from_depth (vis_fmt % os.path.splitext (scene_path) [0])
    occ_im = np_from_depth (occ_fmt % os.path.splitext (scene_path) [0])

    # Calculate raw depths from the integers in image
    depth_im = scaler.scale_ints_to_depths (depth_im)
    vis_im = scaler.scale_ints_to_depths (vis_im)
    occ_im = scaler.scale_ints_to_depths (occ_im)

    fig = plt.figure (figsize=(15,6))

    ax = plt.subplot (1,3,1)
    # gray_r
    depth_obj = plt.imshow (depth_im [:, :, 0], cmap=plt.cm.jet)
      #clim=[MIN_DEPTH, MAX_DEPTH])
    plt.title ('Raw Depth')
    plt.colorbar (depth_obj, fraction=0.046, pad=0.01)

    ax = plt.subplot (1,3,2)
    plt.imshow (depth_im [:, :, 0], cmap=plt.cm.jet, alpha=0.4)
    vis_obj = plt.imshow (vis_im [:, :, 0], cmap=plt.cm.jet, alpha=0.6)
      #clim=[MIN_DEPTH, MAX_DEPTH])
    plt.title ('Visible')
    plt.colorbar (vis_obj, fraction=0.046, pad=0.01)

    ax = plt.subplot (1,3,3)
    plt.imshow (depth_im [:, :, 0], cmap=plt.cm.jet, alpha=0.4)
    occ_obj = plt.imshow (occ_im [:, :, 0], cmap=plt.cm.jet, alpha=0.6)
      #clim=[MIN_DEPTH, MAX_DEPTH])
    plt.title ('Occluded')
    # Flush colorbar with image
    fig.colorbar (occ_obj, fraction=0.046, pad=0.01)


    fig.tight_layout ()

    ax = plt.gca ()
    ax.set_aspect (1)
    dest = os.path.join (vis_path,
      os.path.splitext (os.path.basename (scene_path)) [0] + '.png')
    fig.savefig (dest)

    # To save individual image cleanly
    #extent = ax.get_window_extent ().transformed (fig.dpi_scale_trans.inverted ())
    #fig.savefig (dest, bbox_inches=extent)

    print ('%sWritten entire plot to %s%s' % (ansi_colors.OKCYAN, dest,
      ansi_colors.ENDC))


    plt.show ()




if __name__ == '__main__':
  main ()


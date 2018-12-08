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
import csv
import argparse

import numpy as np

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
from util.ansi_colors import ansi_colors as ansi
from util.image_util import np_from_depth, show_image, matshow_image
from util.matplotlib_util import black_background, black_colorbar
from grasp_collection.config_paths import get_contacts_path
from depth_scene_rendering.config_read_yaml import ConfigReadYAML

# Local
from tactile_occlusion_heatmaps.config_paths import \
  get_heatmap_blob_fmt, \
  get_renders_data_path, get_heatmaps_data_path, \
  get_vis_path, get_vis_depth_fmt, get_vis_vis_fmt, get_vis_occ_fmt, \
  get_vis_heatmap_fmt
from depth_to_image import RawDepthScaling



def main ():

  arg_parser = argparse.ArgumentParser ()

  # Variable number of args http://stackoverflow.com/questions/13219910/argparse-get-undefined-number-of-arguments
  arg_parser.add_argument ('--display', action="store_true",
    help='Specify for debugging one by one. Displays matplotlib plots. This will block program flow and require user interaction to close window to move on to next data sample.')
  arg_parser.add_argument ('--no-save', action='store_true',
    help='Specify to NOT save any visualized images.')
  arg_parser.add_argument ('--uinput', action='store_true',
    help='Specify to use keyboard interaction, useful for debugging.')
  arg_parser.add_argument ('--scale-heatmaps', action='store_true',
    help='Specify the flag if it was specified to occlusion_test.cpp to generate the heatmaps. Rescale heatmaps back to min/max depth values.')
  arg_parser.add_argument ('--black-bg', action='store_true',
    help='Generate plots with black background.')

  args = arg_parser.parse_args ()

  SCALE_HEATMAPS = args.scale_heatmaps
  BLACK_BG = args.black_bg


  DISPLAY_IMAGES = args.display
  UINPUT = args.uinput

  SAVE_IMG = not args.no_save
  # Number of grasps per object to save images
  N_TO_SAVE = 1

  # User adjust parameter. Save just a single axis, for paper or presentation
  # Indices in the 3 subplots
  DEPTH_IDX = 1
  VIS_IDX = 2
  OCC_IDX = 3
  # NOTE: Make sure these names match with the subplot indices!
  SUBPLOT_NAMES = [get_vis_depth_fmt (), get_vis_vis_fmt (), get_vis_occ_fmt ()]
  #SAVE_SUBPLOTS = [DEPTH_IDX]
  SAVE_SUBPLOTS = [OCC_IDX]

  pkg_path = rospkg.RosPack ().get_path ('depth_scene_rendering')
  scene_list_path = os.path.join (pkg_path, "config/scenes_noisy.yaml")
  scene_list_f = open (scene_list_path, 'rb')

  renders_dir = get_renders_data_path ()
  heatmaps_dir = get_heatmaps_data_path ()

  heatmap_fmts = get_heatmap_blob_fmt ()
  vis_fmt = heatmap_fmts [0]
  occ_fmt = heatmap_fmts [1]

  scaler = RawDepthScaling ()
  success, depth_range = scaler.load_depth_range ()
  if not success:
    return
  MIN_DEPTH = depth_range [0]
  MAX_DEPTH = depth_range [1]

  vis_dir = get_vis_path ()
  contacts_dir = get_contacts_path ()


  # scenes.yaml
  objs = ConfigReadYAML.read_scene_paths ()
  # String
  obj_names = objs [0]
  # List of list of strings, paths to .pcd scene files
  scene_paths = objs [2]

  # For each object
  terminate = False
  for o_i in range (len (obj_names)):
  #for o_i in [7]:

    obj_name = obj_names [o_i]

    # Contacts meta file, number of elements in the list is number of grasps
    obj_meta_path = os.path.join (contacts_dir, obj_name + '_meta.csv')
    with open (obj_meta_path, 'rb') as obj_meta_f:
      obj_meta_reader = csv.reader (obj_meta_f)
      # Only 1 row in file. List of strings separated by comma
      for row in obj_meta_reader:
        n_grasps = len (row)

    skip_obj = False
    # For each scene for this object
    for s_i in range (len (scene_paths [o_i])):
     
      # For each grasp for this object
      for g_i in range (n_grasps):

        scene_path = scene_paths [o_i] [s_i]
        scene_base = os.path.basename (scene_path)
     
        print ('Object [%d], Scene [%d], Grasp [%d], Loading triplet files for %s' % (
          o_i, s_i, g_i, scene_path))

        depth_name = os.path.join (renders_dir,
          os.path.splitext (scene_base) [0] + 'crop.png')
        vis_name = os.path.join (heatmaps_dir,
          vis_fmt % (os.path.splitext (scene_base) [0], g_i))
        occ_name = os.path.join (heatmaps_dir,
          occ_fmt % (os.path.splitext (scene_base) [0], g_i))

        depth_im = np_from_depth (depth_name)
        vis_im = np_from_depth (vis_name)
        occ_im = np_from_depth (occ_name)
 
        if depth_im is None or vis_im is None or occ_im is None:
          print ('%sERROR: One or more of (depth, vis_tac, occ_tac) images does not exist. Did you forget to run rosrun depth_scene_generation postprocess_scenes? Run it to generate the cropped images. Terminating...%s' % (
            ansi.FAIL, ansi.ENDC))
          return

        # Calculate raw depths from the integers in image
        depth_im = scaler.scale_ints_to_depths (depth_im)
        if SCALE_HEATMAPS:
          vis_im = scaler.scale_ints_to_depths (vis_im)
          occ_im = scaler.scale_ints_to_depths (occ_im)
        else:
          vis_im = vis_im.astype (np.float32) / 255.0
          occ_im = occ_im.astype (np.float32) / 255.0
     
        fig = plt.figure (figsize=(15,6))
     
        ax = plt.subplot (1,3,1)
        # gray_r
        depth_obj = plt.imshow (depth_im [:, :, 0], cmap=plt.cm.jet)
          #clim=[MIN_DEPTH, MAX_DEPTH])
        tt1 = plt.title ('Raw Depth')
        cb1 = plt.colorbar (depth_obj, fraction=0.046, pad=0.01)
        if BLACK_BG:
          black_background (title_hdl=tt1)
          black_colorbar (cb1)
     
        ax = plt.subplot (1,3,2)
        # Plot a white background first, so that black background plots produce
        #   the same heatmap appearances as white background ones.
        # Ref plot white image https://stackoverflow.com/questions/28234416/plotting-a-white-grayscale-image-in-python-matplotlib
        plt.imshow (np.ones ((depth_im.shape[0], depth_im.shape[1])),
          cmap='gray', vmin=0, vmax=1, alpha=1.0)
        plt.imshow (depth_im [:, :, 0], cmap=plt.cm.jet, alpha=0.4)
        vis_obj = plt.imshow (vis_im [:, :, 0], cmap=plt.cm.jet, alpha=0.7)
          #clim=[MIN_DEPTH, MAX_DEPTH])
        tt2 = plt.title ('Visible')
        cb2 = plt.colorbar (vis_obj, fraction=0.046, pad=0.01)
        if BLACK_BG:
          black_background (title_hdl=tt2)
          black_colorbar (cb2)
     
        ax = plt.subplot (1,3,3)
        plt.imshow (np.ones ((depth_im.shape[0], depth_im.shape[1])),
          cmap='gray', vmin=0, vmax=1, alpha=1.0)
        plt.imshow (depth_im [:, :, 0], cmap=plt.cm.jet, alpha=0.4)
        occ_obj = plt.imshow (occ_im [:, :, 0], cmap=plt.cm.jet, alpha=0.7)
          #clim=[MIN_DEPTH, MAX_DEPTH])
        tt3 = plt.title ('Occluded')
        # Flush colorbar with image
        cb3 = plt.colorbar (occ_obj, fraction=0.046, pad=0.01)
        if BLACK_BG:
          black_background (title_hdl=tt3)
          black_colorbar (cb3)
     
     
        fig.tight_layout ()

        if SAVE_IMG:
          dest = os.path.join (vis_dir,
            get_vis_heatmap_fmt () % (
              os.path.splitext (os.path.basename (scene_base)) [0], g_i))
     
          if BLACK_BG:
            plt.savefig (dest, bbox_inches='tight',
              facecolor=fig.get_facecolor (), edgecolor='none', transparent=True)
          else:
            fig.savefig (dest)
          print ('%sWritten entire plot to %s%s' % (ansi.OKCYAN, dest,
            ansi.ENDC))
     
        # Save an individual axis
        for subplot_i in SAVE_SUBPLOTS:

          # For depth image, only need to save 1st one, `.` all the same
          if subplot_i == DEPTH_IDX and g_i != 0:
            continue

          ax = plt.subplot (1,3,subplot_i)
          ax.set_aspect (1)
          plt.axis ('off')
 
          # To save individual axis cleanly
          depth_dest = os.path.join (vis_dir, SUBPLOT_NAMES [subplot_i-1] % (
            os.path.splitext (os.path.basename (scene_base)) [0]))
          extent = ax.get_window_extent ().transformed (fig.dpi_scale_trans.inverted ())
          fig.savefig (depth_dest, bbox_inches=extent)
          print ('%sWritten depth image axis to %s%s' % (ansi.OKCYAN, depth_dest,
            ansi.ENDC))

        if DISPLAY_IMAGES:
          plt.show ()

        plt.close (fig)

        if g_i >= N_TO_SAVE - 1:
          break

        if UINPUT:
          uinput = raw_input ('Press s to skip to next scene, o to skip to next objet, q to quit, or anything else to go to next grasp in this scene: ')
          if uinput.lower () == 's':
            break
          elif uinput.lower () == 'o':
            skip_obj = True
            break
          elif uinput.lower () == 'q':
            terminate = True
            break

      # Break out of s_i
      if skip_obj or terminate:
        break

    # Break out of o_i
    if terminate:
      break



if __name__ == '__main__':
  main ()


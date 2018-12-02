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
from grasp_collection.config_paths import get_contacts_path
from depth_scene_rendering.config_read_yaml import ConfigReadYAML

# Local
from tactile_occlusion_heatmaps.config_paths import \
  get_heatmap_blob_fmt, \
  get_renders_data_path, get_heatmaps_data_path, \
  get_vis_path, get_vis_heatmap_fmt
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

  args = arg_parser.parse_args ()


  DISPLAY_IMAGES = args.display
  SAVE_IMG = not args.no_save
  UINPUT = args.uinput

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
  objs = ConfigReadYAML.read_object_names ()
  # String
  obj_names = objs [0]
  # List of list of strings, paths to .pcd scene files
  scene_paths = objs [2]

  # For each object
  terminate = False
  for o_i in range (len (obj_names)):
  # TODO: TEMPORARY debugging object 7 only
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
        # TODO 2018 11 26: This is a bug. Tactile heatmaps shouldn't be scaled the same way as depth images!!! They should be in range 0 to 1, not the depth ranges!
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
        if SAVE_IMG:
          dest = os.path.join (vis_dir,
            get_vis_heatmap_fmt () % (
              os.path.splitext (os.path.basename (scene_base)) [0], g_i))
          fig.savefig (dest)
     
          # To save individual axis cleanly
          #extent = ax.get_window_extent ().transformed (fig.dpi_scale_trans.inverted ())
          #fig.savefig (dest, bbox_inches=extent)
       
          print ('%sWritten entire plot to %s%s' % (ansi.OKCYAN, dest,
            ansi.ENDC))
     
     
        if DISPLAY_IMAGES:
          plt.show ()

        plt.close (fig)

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


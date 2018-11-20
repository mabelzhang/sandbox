#!/usr/bin/env python

# Mabel Zhang
# 20 Oct 2018
#
# Convert PNG images to .npz format
#
# Usage:
#   $ rosrun tactile_occlusion_heatmaps png_to_npz.py [--path <path>] --single-channel
#

import os
import argparse
import glob
import time

import numpy as np

# Custom
from util.ansi_colors import ansi_colors as ansi
from util.image_util import np_from_depth
from depth_scene_rendering.config_read_yaml import ConfigReadYAML
from grasp_collection.config_consts import ENERGY_ABBREV, THRESH_INVALID_ENERGY

# Local
from tactile_occlusion_heatmaps.config_paths import get_data_path, \
  get_renders_data_path, get_heatmaps_data_path, get_depth_fmt, \
  get_heatmap_blob_fmt, get_label_fmt
from depth_to_image import RawDepthScaling
from labels_io import LabelsIO


def main ():

  arg_parser = argparse.ArgumentParser ()

  # Variable number of args http://stackoverflow.com/questions/13219910/argparse-get-undefined-number-of-arguments
  arg_parser.add_argument ('--path', type=str,
    help='Path containing PNG files to convert. All PNG files in the directory will be converted to .npz (withOUT removing original files), in a subdirectory npz/')

  # NOTE Assume all images are depth images, will take a single channel
  #arg_parser.add_argument ('--single-channel', action='store_true',
  #  help='Specify this flag to indicate that all three RGB channels of the image are the same, extract only one, and save the single channel to npz.')
  #arg_parser.add_argument ('--all-channels', action='store_true',
  #  help='Specify this flag to indicate saving all channels, i.e. treating the image as a regular picture, to npz.')

  args = arg_parser.parse_args ()


  #if args.single_channel and args.all_channels:
  #  print ('%sERROR: Both --single-channel and --all-channels are specified. Must specify exactly one.%s' % (
  #    ansi.FAIL, ansi.ENDC))
  #  return
  #elif args.single_channel:
  #  N_CHANNELS = 1
  #elif args.all_channels:
  #  N_CHANNELS = 3


  # Sanity checks
  if not args.path:
    heatmaps_dir = get_heatmaps_data_path ()
    renders_dir = get_renders_data_path ()
  else:
    if not os.path.exists (args.path):
      print ('%sERROR: Path specified does not exist: %s%s' % (
        ansi.FAIL, args.path, ansi.ENDC))
      return
    in_dir = args.path
    heatmaps_dir = os.path.join (in_dir, 'heatmaps')
    renders_dir = os.path.join (in_dir, 'renders')

  out_dir = os.path.join (get_data_path (), 'npz')
  if not os.path.exists (out_dir):
    os.makedirs (out_dir)

  print ('Reading from %s' % os.path.dirname (heatmaps_dir))



  # Get all input file formats for wildcard
  depth_fmt = get_depth_fmt ()

  heatmap_fmts = get_heatmap_blob_fmt ()
  vis_heatmap_fmt = heatmap_fmts [0]
  occ_heatmap_fmt = heatmap_fmts [1]

  lbl_fmt = get_label_fmt ()

  # Init all lists
  depth_png_list = []
  vis_png_list = []
  occ_png_list = []
  lbl_list = []
  obj_lbls = []


  start_time = time.time ()

  # Compile a list of all data samples

  # Get all scene names from YAML
  objs = ConfigReadYAML.read_object_names ()
  # String
  obj_names = objs [ConfigReadYAML.NAME_IDX]
  # Int
  obj_ids = objs [ConfigReadYAML.ID_IDX]
  # List of list of strings, paths to .pcd scene files
  scene_paths = objs [ConfigReadYAML.SCENE_IDX]

  # For each object
  for o_i in range (len (obj_names)):

    #obj_name = obj_names [o_i]
    #print ('%s' % obj_names [o_i])

    # Number of examples in this object
    n_obj_examples = 0

    # For each .pcd scene for this object
    for s_i in range (len (scene_paths [o_i])):

      scene_path = scene_paths [o_i] [s_i] 
      #print (scene_path)

      # Prefix of each data file for predictor
      scene_name = os.path.splitext (os.path.basename (scene_path)) [0]

      # There are multiple grasps per scene, so there will be many more
      #   vis, occ, and lbls than there are depth images. Depth images will be
      #   duplicated for each example, to make correpsonding btw different npz
      #   files easier, and `.` CNN wants all data in a matrix in memory anyway.
      depth_path = os.path.join (renders_dir, depth_fmt % (scene_name))

      # Per-grasp heatmaps and label data
      # Use glob instead of reading contacts file for number of grasps, `.`
      #   glob is safer - won't read any files that don't exist if I skipped it
      #   because of 0 contacts or 0 points in point cloud, and is faster.
      # Replace the %d formatting for grasp number with *, to get all grasps
      vis_wildcard = vis_heatmap_fmt.replace ('%d', '*')
      vis_wildcard = vis_wildcard % scene_name
      vis_sublist = glob.glob (os.path.join (heatmaps_dir, vis_wildcard))
      vis_png_list.extend (vis_sublist)

      occ_wildcard = occ_heatmap_fmt.replace ('%d', '*')
      occ_wildcard = occ_wildcard % scene_name
      occ_sublist = glob.glob (os.path.join (heatmaps_dir, occ_wildcard))
      occ_png_list.extend (occ_sublist)

      lbl_wildcard = lbl_fmt.replace ('%d', '*')
      lbl_wildcard = lbl_wildcard % scene_name
      lbl_sublist = glob.glob (os.path.join (heatmaps_dir, lbl_wildcard))
      lbl_list.extend (lbl_sublist)

      if len (vis_sublist) != len (occ_sublist) or \
         len (occ_sublist) != len (lbl_sublist):
        print ('%sERROR: Wildcard lists for visible heatmap, occluded heatmap, and/or label files are not of the same size! You may have error in correspondence between the outputted npz files.%s' % (ansi.FAIL, ansi.ENDC))

      # Per-scene depth image. Same for all grasps in this scene
      depth_png_list.extend ([depth_path] * len (vis_sublist))

      n_obj_examples += len (vis_sublist)

    # Per-object object-id label. Same for all scenes in this object
    obj_lbls.extend ([obj_ids [o_i]] * n_obj_examples)


  print ('%d files' % (len (depth_png_list)))
  in_png_lists = [depth_png_list, vis_png_list, occ_png_list, lbl_list]

  # TODO: ENERGY_ABBREV should be read from the file name of the energy csv
  #   file (new code, haven't trained yet, need to rerun GraspIt training).
  #   otherwise no way to keep track of which file used which energy if I change
  #   the energy in code!!!
  npz_prefix = ['depth', 'vis', 'occ', 'lbl_' + ENERGY_ABBREV]

  glob_time = time.time () - start_time


  # To scale RGB integers back to raw depths
  scaler = RawDepthScaling ()
  success, depth_range = scaler.load_depth_range ()
  if not success:
    return
  MIN_DEPTH = depth_range [0]
  MAX_DEPTH = depth_range [1]


  # Load one image to get dimensions, to initialize array
  img = None
  for png_list in in_png_lists:
    if len (png_list) == 0:
      continue

    img = np_from_depth (vis_png_list [0])
    break

  # If still nothing loaded, there is no files in the list
  if img is None:
    print ('No files to process. Terminating.')
    return

  NROWS = {'.png': img.shape [0], '.yaml': 1}
  NCOLS = {'.png': img.shape [1], '.yaml': 1}

  BATCH_SIZE = 1000
  #BATCH_SIZE = 10  # For quick testing

  #n_skipped_grasps = 0


  #####
  # Output PNG images and grasp scalar float label to .npz files

  # Output npz files for each data sample
  # Loop through each type of file
  for l_i in range (len (in_png_lists)):

    png_list = in_png_lists [l_i]

    print ('List %d out of %d. %d png files in path' % (l_i+1,
      len (in_png_lists), len (png_list)))
 
    if len (png_list) == 0:
      print ('No files to process in this list.')
      continue

    # Get extension of file in this type
    ext = os.path.splitext (png_list [0]) [1].lower ()


    # Number of rows filled in the current batch
    rows_filled = 0
    # Number of batches filled
    batches_filled = 0
 
    # Init array to write to file
    # (1000, height, width, 1) for images
    # (1000, 1, 1, 1) for grasp energy
    npz_arr = np.zeros ((BATCH_SIZE, NROWS [ext], NCOLS [ext], 1))
      #dtype=np.float64)  # This doesn't eliminate warning from tensorflow

 
    # Loop through all files in this type
    for png_name in png_list:
 
      #print ('Batch %d, image [%d] out of max %d: %s' % (batches_filled,
      #  rows_filled, BATCH_SIZE, png_name))

      # Load PNG
      if ext == '.png':

        img = np_from_depth (png_name)

        # Calculate raw depths from the integers in image
        img = scaler.scale_ints_to_depths (img)
       
        # Append to last row of array
        # Assumption: All channels are the same. Take just a single channel
        npz_arr [rows_filled, :, :, 0] = img [:, :, 0]

      # Load grasp label
      elif ext == '.yaml':

        labels = LabelsIO.read_label (png_name)
        energy = labels [1]

        # TODO: Another option, instead of skipping, is to cap them to
        #   a constant, THRESH_INVALID_ENERGY. Then the example can still be
        #   used, and simply counted as a bad grasp.
        # TODO: This should really be done in grasp_collect.py. Can't skip
        #   here, because this only skips the lbls files, not the grasps and
        #   heatmap files! Would need ot add that if want to skip in this file.
        # Skip grasps with an energy above an invalid threshold. Invalid grasps
        #if energy > THRESH_INVALID_ENERGY:
        #  print ('%sGrasp energy %g, skipping%s' % (
        #    ansi.WARNING, energy, ansi.ENDC))
        #  n_skipped_grasps += 1
        #  TODO Just implemented, test it.
        #  energy = THRESH_INVALID_ENERGY
        #  #continue

        # Scalar
        npz_arr [rows_filled, :, :, 0] = energy

      # Increment AFTER appending
      rows_filled += 1
 
 
      # Batch is full, write to file
      # 1000 32x32 images each .npz file
      if rows_filled == BATCH_SIZE:

        out_path = os.path.join (out_dir,
          npz_prefix [l_i] + ('_%05d.npz' % batches_filled))

        # Write npz. Inspect it with
        #   m = np.load(path);
        #   m['arr_0'].shape;
        #   np.unique (m['arr_0']);
        #   m.close()  # Must close, else can have leakage, per load() API
        #np.savez_compressed (out_path, np.squeeze (npz_arr))
        np.savez_compressed (out_path, npz_arr)
 
        print ('%sWritten %d-row matrix to %s%s' % (ansi.OKCYAN,
          npz_arr.shape [0], out_path, ansi.ENDC))
 
        # Reset
        npz_arr = np.zeros ((BATCH_SIZE, NROWS [ext], NCOLS [ext], 1))
          #dtype=np.float64)  # This doesn't eliminate warning from tensorflow
        rows_filled = 0
        batches_filled += 1

        print ('Batch %d filled, batch size %d' % (batches_filled, BATCH_SIZE))
 
 
    # Last npz file
    # Remove unfilled rows
    if rows_filled > 0:
 
      if rows_filled < 1000:
        npz_arr = np.delete (npz_arr, np.s_ [rows_filled::], 0)
 
      out_path = os.path.join (out_dir,
        npz_prefix [l_i] + ('_%05d.npz' % batches_filled))
 
      #np.savez_compressed (out_path, np.squeeze (npz_arr))
      np.savez_compressed (out_path, npz_arr)
 
      print ('%sWritten %d-row matrix to %s%s' % (ansi.OKCYAN,
        npz_arr.shape [0], out_path, ansi.ENDC))


  #####
  # Output object scalar integer labels to .npz files
  # Write obj_lbls object labels to a separate npz file, so can evaluate
  #   per-obj prediction accuracies, to get ideas of how to improve predictions.

  # Number of rows filled in the current batch
  rows_filled = 0
  # Number of batches filled
  batches_filled = 0

  # Same loop structure as above.
  # NOTE: If change loop structure above, update here accordingly, so that each
  #   row in the matrix here corresponds to each row in the data written above!
  npz_arr = np.zeros ((BATCH_SIZE, 1, 1, 1))
  for lbl in obj_lbls:

    npz_arr [rows_filled, :, :, 0] = lbl

    # Increment AFTER appending
    rows_filled += 1

    # Batch is full, write to file
    if rows_filled == BATCH_SIZE:

      out_path = os.path.join (out_dir, 'obj_id_%05d.npz' % batches_filled)

      np.savez_compressed (out_path, npz_arr)

      print ('%sWritten %d-row matrix to %s%s' % (ansi.OKCYAN,
        npz_arr.shape [0], out_path, ansi.ENDC))

      # Reset
      npz_arr = np.zeros ((BATCH_SIZE, NROWS [ext], NCOLS [ext], 1))
      rows_filled = 0
      batches_filled += 1

      print ('Batch %d filled, batch size %d' % (batches_filled, BATCH_SIZE))
 
  # Last npz file
  # Remove unfilled rows
  if rows_filled > 0:

    if rows_filled < 1000:
      npz_arr = np.delete (npz_arr, np.s_ [rows_filled::], 0)

    out_path = os.path.join (out_dir, 'obj_id_%05d.npz' % batches_filled)

    np.savez_compressed (out_path, npz_arr)

    print ('%sWritten %d-row matrix to %s%s' % (ansi.OKCYAN,
      npz_arr.shape [0], out_path, ansi.ENDC))


  end_time = time.time () - start_time

  print ('%d total examples actually written' % (batches_filled * BATCH_SIZE + rows_filled))
  #print ('%d skipped and not written' % (n_skipped_grasps))
  print ('Elapsed time: %g seconds, in which %g seconds was mainly glob() calls' % (
    end_time, glob_time))


if __name__ == '__main__':
  main ()


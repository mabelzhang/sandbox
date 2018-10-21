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

import numpy as np

# Custom
from util.ansi_colors import ansi_colors as ansi
from util.image_util import np_from_depth, np_from_image

# Local
from tactile_occlusion_heatmaps.config_paths import get_data_path, \
  get_heatmap_blob_fmt
from depth_to_image import RawDepthScaling



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
    in_dir = get_data_path ()
  else:
    if not os.path.exists (args.path):
      print ('%sERROR: Path specified does not exist: %s%s' % (
        ansi.FAIL, args.path, ansi.ENDC))
      return
    in_dir = args.path

  out_dir = os.path.join (in_dir, 'npz')
  if not os.path.exists (out_dir):
    os.makedirs (out_dir)


  # Get all input file paths
  heatmap_fmts = get_heatmap_blob_fmt ()

  vis_heatmap_fmt = heatmap_fmts [0].replace ('%s', '*')
  vis_heatmap_fmt = vis_heatmap_fmt.replace ('%d', '*')
  vis_png_list = glob.glob (os.path.join (in_dir, vis_heatmap_fmt))

  occ_heatmap_fmt = heatmap_fmts [1].replace ('%s', '*')
  occ_heatmap_fmt = occ_heatmap_fmt.replace ('%d', '*')
  occ_png_list = glob.glob (os.path.join (in_dir, occ_heatmap_fmt))

  in_png_lists = [vis_png_list, occ_png_list]

  npz_prefix = ['vis', 'occ']


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


  NROWS = img.shape [0]
  NCOLS = img.shape [1]

  #BATCH_SIZE = 1000
  # TODO: Temporary for testing
  BATCH_SIZE = 10


  for l_i in range (len (in_png_lists)):

    png_list = in_png_lists [l_i]

    print ('List %d out of %d. %d png files in path' % (l_i+1,
      len (in_png_lists), len (png_list)))
 
    if len (png_list) == 0:
      print ('No files to process in this list.')
      continue


    # Number of rows filled in the current batch
    rows_filled = 0
    # Number of batches filled
    batches_filled = 0
 
    # Init array to write to file
    npz_arr = np.zeros ((BATCH_SIZE, NROWS, NCOLS, 1))

 
    # Loop through all files in directory
    for png_name in png_list:
 
      print ('Batch %d, image [%d] out of %d: %s' % (batches_filled,
        rows_filled, BATCH_SIZE, png_name))
 
      # Load PNG
      img = np_from_depth (png_name)

      # Calculate raw depths from the integers in image
      img = scaler.scale_ints_to_depths (img)

      # Append to last row of array
      # Assumption: All channels are the same. Take just a single channel
      npz_arr [rows_filled, :, :, 0] = img [:, :, 0]

      # Increment AFTER appending
      rows_filled += 1
 
 
      # Batch is full, write to file
      # 1000 32x32 images each .npz file
      if rows_filled == BATCH_SIZE:

        out_path = os.path.join (out_dir,
          npz_prefix [l_i] + ('_%05d.npz' % batches_filled))
 
        # Write npz. Load it with m = np.load(path), m['arr_0'], m.close()
        np.savez_compressed (out_path, npz_arr)
 
        print ('%sWritten %d x %d matrix to %s%s' % (ansi.OKCYAN,
          npz_arr.shape [0], npz_arr.shape [1], out_path, ansi.ENDC))
 
        # Reset
        npz_arr = np.zeros ((BATCH_SIZE, NROWS, NCOLS, 1))
        rows_filled = 0
        batches_filled += 1
 
 
    # Last npz file
    # Remove unfilled rows
    if rows_filled > 0:
 
      if rows_filled < 1000:
        npz_arr = np.delete (npz_arr, np.s_ [rows_filled::], 0)
 
      out_path = os.path.join (out_dir,
        npz_prefix [l_i] + ('_%05d.npz' % batches_filled))
 
      np.savez_compressed (out_path, npz_arr)
 
      print ('%sWritten %d x %d matrix to %s%s' % (ansi.OKCYAN,
        npz_arr.shape [0], npz_arr.shape [1], out_path, ansi.ENDC))


if __name__ == '__main__':
  main ()


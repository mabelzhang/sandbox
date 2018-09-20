#!/usr/bin/env python

# Mabel Zhang
# 20 Sep 2018
#
# Crop images at the paths in the given text file. One path per line.
#

import os
import argparse
import time

import cv2

from util.ansi_colors import ansi_colors

from config_paths import get_render_path


# Crop out the center portion of specified dimensions from an image
def crop_image_center (img, crop_w, crop_h):

  height = img.shape [0]
  width = img.shape [1]

  # Crop
  return (img [0.5 * (height - crop_h) : 0.5 * (height + crop_h),
    0.5 * (width - crop_w) : 0.5 * (width + crop_w)])



def main ():

  arg_parser = argparse.ArgumentParser ()

  # Variable number of args http://stackoverflow.com/questions/13219910/argparse-get-undefined-number-of-arguments
  arg_parser.add_argument ('--textfile', type=str,
    help='Full path to text file containing a list of full paths to images to crop')

  args = arg_parser.parse_args ()

  meta_path = args.textfile


  if not meta_path:
    render_path = get_render_path ()
    meta_path = os.path.join (render_path, 'translations', 'uncropped.txt')


  start_time = time.time ()

  with open (meta_path, 'rb') as meta_f:

    # For each line in file
    for img_path in meta_f:

      img_path = img_path.strip ()

      print ('Reading %s' % img_path)
 
      img = cv2.imread (img_path)
 
      crop = crop_image_center (img, 100, 100)
 
      # NOTE: No file existence is checked. Human user makes sure old files
      #   are not overwritten.
      crop_path = os.path.join (os.path.dirname (img_path), 'crop',
        os.path.basename (img_path))

      if not os.path.exists (os.path.dirname (crop_path)):
        os.makedirs (os.path.dirname (crop_path))

      cv2.imwrite (crop_path, crop)

      print ('%sWritten cropped image to %s%s' % (ansi_colors.OKCYAN, crop_path,
        ansi_colors.ENDC))

  end_time = time.time ()

  print ('Elapsed time: %g seconds' % (end_time - start_time))


if __name__ == '__main__':
  main ()


#!/usr/bin/env python

# Mabel Zhang
# 4 Sep 2018
#
# Convert .pcd files from BlenSor (outputted by scene_generation.py, which
#   calls scan_kinect.py), to 2D images with raw depth values, using OpenCV and
#   intrinsic camera matrix from BlenSor.
#

def main ():

  # TODO: Convert pcd to png, using OpenCV depth() and intrinsics matrix from
  #   Blender.
  # pcd_name
  # noisy_pcd_name
  # This can't be done from within Blender! Blender doesn't know about
  #   PCL libraries. Must do as post-processing from terminal.



if __name__ == '__main__':
  main ()


#!/usr/bin/env python

# Mabel Zhang
# 4 Sep 2018
#
# Inspect images of scenes generated in Blender, converted from PCD using
#   different methods.
#

import numpy as np

# Custom packages
from util.image_util import np_from_depth, show_image, matshow_image



def main ():

  # Compare the 3 image formats.
  # Conclusion: None are good. None retain raw depth values from camera!
  #   PGM is probably the best. PGM and pcl_pcd2png values are opposite in
  #   increasing or decreasing according to far or close to camera.

  # Outputted from PCL pcl_pcd2png
  print ('pcl_pcd2png output')
  im_png = np_from_depth ('/media/master/Data_Ubuntu/courses/research/graspingRepo/train/visuotactile_grasping/2018-09-04-15-41-2200000_pclpcd2png.png')
  print (im_png.shape)
  # This prints True. So can simply matshow() on one channel and look at
  #   colorbar to tell value.
  print (np.all (im_png [:, :, 0] == im_png [:, :, 1]))
  print (np.all (im_png [:, :, 1] == im_png [:, :, 2]))
  print ('min %f, median %f, max %f' % (np.min (im_png), np.median (im_png),
    np.max (im_png)))
  print (np.unique (im_png))
  matshow_image (im_png [:, :, 0], title='PCL pcd2png')

  # Outputted from pcd2png2. Raw depth values
  print ('pcd2png2 output')
  im_png_JPpcd2png2 = np_from_depth ('/media/master/Data_Ubuntu/courses/research/graspingRepo/train/visuotactile_grasping/2018-09-04-15-41-2200000_JPpcd2png2.png')
  print (im_png_JPpcd2png2.shape)
  # These print True. So all 3 channels are the same.
  print (np.all (im_png_JPpcd2png2 [:, :, 0] == im_png_JPpcd2png2 [:, :, 1]))
  print (np.all (im_png_JPpcd2png2 [:, :, 1] == im_png_JPpcd2png2 [:, :, 2]))
  print ('min %f, median %f, max %f' % (np.min (im_png_JPpcd2png2),
    np.median (im_png_JPpcd2png2), np.max (im_png_JPpcd2png2)))
  print (np.unique (im_png_JPpcd2png2))
  matshow_image (im_png_JPpcd2png2 [:, :, 0], title='JP pcd2png2')

  print ('BlenSor Kinect camera PGM output')
  # Outputted from BlenSor Kinect camera directly
  im_pgm = np_from_depth ('/media/master/Data_Ubuntu/courses/research/graspingRepo/train/visuotactile_grasping/2018-09-04-18-35-4700000.pgm')
  print (im_pgm.shape)
  # This prints True. So can simply matshow() on one channel and look at
  #   colorbar to tell value.
  print (np.all (im_pgm [:, :, 0] == im_pgm [:, :, 1]))
  print (np.all (im_pgm [:, :, 1] == im_pgm [:, :, 2]))
  print ('min %f, median %f, max %f' % (np.min (im_pgm), np.median (im_pgm),
    np.max (im_pgm)))
  print (np.unique (im_pgm))
  matshow_image (im_pgm [:, :, 0], title='BlenSor PGM')


  print ('My custom postprocess_scenes.cpp PNG output')
  # Outputted from BlenSor Kinect camera directly
  im_png_rawdepth = np_from_depth ('/media/master/Data_Ubuntu/courses/research/graspingRepo/train/visuotactile_grasping/2018-09-04-15-41-22_noisy00000_view0.png')
  print (im_png_rawdepth.shape)
  # This prints True. So can simply matshow() on one channel and look at
  #   colorbar to tell value.
  print (np.all (im_png_rawdepth [:, :, 0] == im_png_rawdepth [:, :, 1]))
  print (np.all (im_png_rawdepth [:, :, 1] == im_png_rawdepth [:, :, 2]))
  print ('min %f, median %f, max %f' % (np.min (im_png_rawdepth), np.median (im_png_rawdepth),
    np.max (im_png_rawdepth)))
  print (np.unique (im_png_rawdepth))
  matshow_image (im_png_rawdepth [:, :, 0], title='My Custom Raw Depth PNG')


  raw_input ('Press enter: ')


if __name__ == '__main__':
  main ()


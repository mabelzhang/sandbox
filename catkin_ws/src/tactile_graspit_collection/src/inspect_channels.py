#!/usr/bin/env python

# Mabel Zhang
# 13 Sep 2018
#
# Inspect images.
#

import numpy as np

from matplotlib.pyplot import *
from matplotlib import pyplot as plt

# Custom packages
from util.image_util import np_from_depth, show_image, matshow_image
from util.filter import blob_kernel



def main ():

  # Single-dots binary image outputted by postprocess_scenes.cpp
  # Outputted from BlenSor Kinect camera directly
  #ims = [np_from_depth ('/media/master/Data_Ubuntu/courses/research/graspingRepo/train/visuotactile_grasping/2018-09-13-00-08-53_noisy00000_vis.png')]

  # Visible and occluded tactile channels outputted by occlusion_test.cpp
  ims = [
    '/media/master/Data_Ubuntu/courses/research/graspingRepo/train/visuotactile_grasping/2018-09-13-01-17-46_noisy00000_vis_blob.png',
    '/media/master/Data_Ubuntu/courses/research/graspingRepo/train/visuotactile_grasping/2018-09-13-01-17-46_noisy00000_occ_blob.png'
  ]

  for i in range (len (ims)):

    im = np_from_depth (ims [i])

    print (im.shape)
    # This prints True. So can simply matshow() on one channel and look at
    #   colorbar to tell value.
    print (np.all (im [:, :, 0] == im [:, :, 1]))
    print (np.all (im [:, :, 1] == im [:, :, 2]))
    print ('min %f, median %f, max %f' % (np.min (im), np.median (im),
      np.max (im)))
    print (np.unique (im))
    print (np.nonzero (im))

    # For output of postprocess_scenes.cpp
    #convolved = blob_kernel (im [:, :, 0])
    #matshow (convolved)
 
    #matshow_image (im [:, :, 0], title='Visible Channel')
    #show_image (im, title='Visible Channel')
    matshow (im [:, :, 0])
    colorbar ()
 
    plt.show ()
    #raw_input ('Press enter: ')


if __name__ == '__main__':
  main ()


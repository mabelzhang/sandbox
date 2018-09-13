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

  print ('My custom postprocess_scenes.cpp PNG output')
  # Outputted from BlenSor Kinect camera directly
  im = np_from_depth ('/media/master/Data_Ubuntu/courses/research/graspingRepo/train/visuotactile_grasping/2018-09-13-00-08-53_noisy00000_vis.png')
  print (im.shape)
  # This prints True. So can simply matshow() on one channel and look at
  #   colorbar to tell value.
  print (np.all (im [:, :, 0] == im [:, :, 1]))
  print (np.all (im [:, :, 1] == im [:, :, 2]))
  print ('min %f, median %f, max %f' % (np.min (im), np.median (im),
    np.max (im)))
  print (np.unique (im))
  print (np.nonzero (im))

  convolved = blob_kernel (im [:, :, 0])

  #matshow_image (im [:, :, 0], title='Visible Channel')
  #show_image (im, title='Visible Channel')
  #matshow (im [:, :, 0])

  matshow (convolved)

  plt.show ()


  raw_input ('Press enter: ')


if __name__ == '__main__':
  main ()


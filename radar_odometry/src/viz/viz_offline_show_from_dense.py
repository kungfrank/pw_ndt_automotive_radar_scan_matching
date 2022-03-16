import numpy as np
from scipy.stats import multivariate_normal
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
import os
import cv2
import time
from PIL import Image
import imutils
import sys

part_num = '1'
dir = '/mnt/Disk2/Oxford/radar_cart_0.125_1001_timestamped/part'+part_num
ndmap_dir_path = '/mnt/Disk2/Oxford/radar_cart_0.125_1001_timestamped/part1/ndmap_2019-01-10-11-46-21-radar-oxford-10k_new'

init = 0

for filename in os.listdir(dir):
    print(filename)
    path = dir+'/' + filename + '/'

    ndmap_dir_path = dir+'/'+'ndmap_'+filename+'_new'
    timestamps_path = path+'radar_t_list'
    radar_timestamps = np.loadtxt(timestamps_path, delimiter=' ', usecols=[0], dtype=np.float64)
    for radar_timestamp in radar_timestamps:

      if radar_timestamp<=65:
        continue

      print(radar_timestamp)
      curr_radar_t_str = str("%010.5f"%radar_timestamp)
      print(curr_radar_t_str)

      img = cv2.imread(path+curr_radar_t_str+'.png',0)
      ret,thresh_img = cv2.threshold(img,70,255,cv2.THRESH_TOZERO)

      cv2.namedWindow('img', cv2.WINDOW_NORMAL)
      cv2.resizeWindow('img', 621,690)
      cv2.imshow('img',img*2)

      cv2.namedWindow('sd_img', cv2.WINDOW_NORMAL)
      cv2.resizeWindow('sd_img', 621,690)
      cv2.imshow('sd_img',thresh_img*2)

      ndmap_img = cv2.imread(ndmap_dir_path+'/'+curr_radar_t_str+'.png', cv2.IMREAD_COLOR)
      #ndmap_img = cv2.rotate(ndmap_img, cv2.ROTATE_90_COUNTERCLOCKWISE) 
      cv2.namedWindow('ndmap_img', cv2.WINDOW_NORMAL)
      cv2.resizeWindow('ndmap_img', 621,690)
      cv2.imshow('ndmap_img', ndmap_img)
      if init == 0:
        cv2.waitKey(0)
        init = 1
      else:
        cv2.waitKey(250)





#!/usr/bin/env python2

import rospy
import message_filters
import cv2

from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from ro_msg.msg import Cov2D
from ro_msg.msg import Cov2DArrayStamped
import time
from scipy.stats import multivariate_normal
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
import time


def main():

  img = cv2.imread('/home/joinet/ro_viz/sd_from_sparse_img/sec2_s3_small_N500/1.png', 0)
  print(img.shape)
  img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
  img = cv2.flip(img, 1)

  img = img.astype('float64')
  img = img/255.

  cv2.namedWindow('img', cv2.WINDOW_NORMAL)
  #cv2.resizeWindow('img', 621,690) # 648,720 #540,600
  cv2.imshow('img', img*1) #30 70 150

  cv2.namedWindow('img*2', cv2.WINDOW_NORMAL)
  cv2.imshow('img*2', img*3)

  cv2.waitKey(0)

#/home/joinet/ro_viz/sd_from_sparse_img/sec2_s3_small_N500

if __name__ == '__main__':
  main()

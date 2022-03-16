#!/usr/bin/env python2

import rospy
import message_filters
import cv2

from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from ro_msg.msg import Cov2D
from ro_msg.msg import Cov2DArrayStamped
#import pcl
import time
from scipy.stats import multivariate_normal
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D

sd_dir_path = '/home/joinet/ro_viz/sd_from_sparse_img/sec2_s3_N350/'
ndmap_dir_path = '/home/joinet/ro_viz/ndmap_from_sparse_img/sec2_s3_g7_e20_small/'
costmap_dir_path = '/home/joinet/ro_viz/costmap_from_sparse_img/sec3_s3_g3/'

small_sd_dir_path = '/home/joinet/ro_viz/sd_from_sparse_img/sec2_s3_small_N500/'

index = 0

def listener():

  rospy.init_node('listener', anonymous=True)
  rospy.Subscriber('/conti_stack_cov/radar_stack', PointCloud2, cb, queue_size=10)
  rospy.spin()

def cb(data):
  global index
  index = index+1
  print(index)
  sd_img = cv2.imread(sd_dir_path+str(index)+'.png', 0)
  sd_img = sd_img.astype('float64')
  sd_img = sd_img/255.
  print(sd_img.shape)
  sd_img = cv2.rotate(sd_img, cv2.ROTATE_90_COUNTERCLOCKWISE) 
  sd_img = cv2.flip(sd_img, 1)
  cv2.namedWindow('sd_img', cv2.WINDOW_NORMAL)
  cv2.resizeWindow('sd_img', 621,690) # 648,720 #540,600
  cv2.imshow('sd_img', sd_img*100.) #30 70 150
  '''
  ndmap_img = cv2.imread(ndmap_dir_path+'sparse_ndmap'+str(index)+'.jpg', cv2.IMREAD_COLOR)
  #ndmap_img = cv2.resize(ndmap_img,(2000,2000));
  print(ndmap_img.shape)
  ndmap_img = cv2.rotate(ndmap_img, cv2.ROTATE_90_COUNTERCLOCKWISE) 
  cv2.namedWindow('ndmap_img', cv2.WINDOW_NORMAL)
  cv2.resizeWindow('ndmap_img', 621,690)
  cv2.imshow('ndmap_img', ndmap_img)
  '''
  small_sd_img = cv2.imread(small_sd_dir_path+str(index)+'.png', 0)
  small_sd_img = small_sd_img.astype('float64')
  small_sd_img = small_sd_img/255.
  print(small_sd_img.shape)
  small_sd_img = cv2.rotate(small_sd_img, cv2.ROTATE_90_COUNTERCLOCKWISE)
  small_sd_img = cv2.flip(small_sd_img, 1)
  cv2.namedWindow('small_sd_img', cv2.WINDOW_NORMAL)
  cv2.resizeWindow('small_sd_img', 621,690) # 648,720 #540,600
  cv2.imshow('small_sd_img', small_sd_img*2.) #30 70 150

  #if index>1:
    #costmap_img = cv2.imread(costmap_dir_path+'costmap'+str(index-1)+'.jpg', cv2.IMREAD_COLOR)
    #costmap_img = cv2.rotate(costmap_img, cv2.ROTATE_90_COUNTERCLOCKWISE) 
    #cv2.namedWindow('costmap_img', cv2.WINDOW_NORMAL)
    #cv2.resizeWindow('costmap_img', 540,600)
    #cv2.imshow('costmap_img', costmap_img)
  cv2.waitKey(100)

if __name__ == '__main__':

  listener()

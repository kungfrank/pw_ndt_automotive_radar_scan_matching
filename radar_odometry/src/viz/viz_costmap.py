#!/usr/bin/env python2

import rospy
import message_filters

from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from ro_msg.msg import Cov2D
from ro_msg.msg import Cov2DArrayStamped
from scipy.stats import multivariate_normal
import cv2
import time

import numpy as np

import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt

from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D

#dir_path = '/home/joinet/ro_viz/ndmap/sparse_temp'

N = 41

index = 0


def multivariate_gaussian(pos, mu, Sigma):
  """
  Return the multivariate Gaussian distribution on array pos.

  pos is an array constructed by packing the meshed arrays of variables
  x_1, x_2, x_3, ..., x_k into its _last_ dimension.
  """

  n = mu.shape[0]
  Sigma_det = np.linalg.det(Sigma)
  Sigma_inv = np.linalg.inv(Sigma)
  N = np.sqrt((2*np.pi)**n * Sigma_det)
  # This einsum call calculates (x-mu)T.Sigma-1.(x-mu) in a vectorized
  # way across all the input variables.
  fac = np.einsum('...k,kl,...l->...', pos-mu, Sigma_inv, pos-mu)

  return np.exp(-fac / 2) / N

def calc_mean_and_variance(pointcloud, pointcloud_cov, id_list, i_, j_):
  n_ = len(id_list)
  print('n_: ', n_)

  if n_ == 0:
    mean_ = np.array([[0, 0]], dtype=np.float32).T
    covar_ = np.array([[0, 0], [0, 0]], dtype=np.float32)
    return mean_, covar_

  sx = np.array([[0, 0]], dtype=np.float32).T
  sxx = np.array([[0, 0], [0, 0]], dtype=np.float32)
  stack_covar = np.array([[0, 0], [0, 0]], dtype=np.float32)

  mean_ = np.array([[0, 0]], dtype=np.float32).T
  covar_ = np.array([[0, 0], [0, 0]], dtype=np.float32)

  for i in id_list:
    #print('pointcloud id: ', i)
    p = np.array([[pointcloud[i][0], pointcloud[i][1]]]).T
    cov = np.array([[pointcloud_cov[i][0], pointcloud_cov[i][1]], [pointcloud_cov[i][2], pointcloud_cov[i][3]]])
    #print('p')
    #print(p)
    #print('cov')
    #print(cov)
    sx = sx + p
    sxx = sxx + p.dot(p.T)
    stack_covar = stack_covar + cov

  if n_ >= 3:
    mean_ = sx / float(n_)
    covar_ = (sxx - 2 * (sx.dot(mean_.T))) / float(n_) + mean_.dot(mean_.T)
    stack_covar = stack_covar / float(n_)
    covar_ = covar_ + stack_covar
  elif n_ > 0 and n_ < 3:
    mean_ = sx / float(n_)
    stack_covar = stack_covar / float(n_)
    covar_ = stack_covar

  grid_mean_x = -grid_extend/2 * grid_size + i_ * grid_size + float(grid_size)/2.0
  grid_mean_y = -grid_extend/2 * grid_size + j_ * grid_size + float(grid_size)/2.0
  grid_mean = np.array([[grid_mean_x, grid_mean_y]], dtype=np.float32).T
  unbias_mean_ = mean_ - grid_mean

  return unbias_mean_, covar_

def main():

  rospy.init_node('Viz_costmap', anonymous=True)
  rospy.loginfo("Start Viz_costmap")
  rospy.Subscriber("/stack_gnd_costmap/ndt_cost_map", PointCloud2, callback, queue_size=10000)

  rospy.spin()


def callback(pc_msg):
  global index
  index = index + 1
  print(index)
  #print('pc_msg.height:')
  #print(pc_msg.height)
  #print('pc_msg.width:')
  #print(pc_msg.width)

  assert isinstance(pc_msg, PointCloud2)
  gen = point_cloud2.read_points(pc_msg, field_names=("x", "y", "z"), skip_nans=True)
  time.sleep(0.001)

  pointcloud = np.empty([3,])
  for p in gen:
    #print " x : %.3f  y: %.3f  z: %.3f" %(p[0],p[1],p[2])
    point = np.asarray(p).reshape((3,))
    pointcloud = np.vstack((pointcloud, point))
  pointcloud = np.delete(pointcloud, 0, 0) # remove first empty item

  costmap_img = np.zeros((N,N,1),np.uint8)

  max_cost = np.max(pointcloud[:,2])
  #print(max_cost)
  for i in range(pointcloud.shape[0]):
    costmap_img[int(round(pointcloud[i][0])+(N-1)/2)][int(round(pointcloud[i][1])+(N-1)/2)] = int(round(pointcloud[i][2]/max_cost*255))
    #print(int(round(pointcloud[i][0])))

  costmap_img = cv2.applyColorMap(costmap_img, cv2.COLORMAP_RAINBOW) #COLORMAP_JET
  costmap_img = cv2.rotate(costmap_img, cv2.ROTATE_90_COUNTERCLOCKWISE)
  cv2.namedWindow('costmap_img', cv2.WINDOW_NORMAL)
  cv2.imshow('costmap_img', costmap_img)

  cv2.imwrite('/home/joinet/ro_viz/costmap_from_sparse_img/sec2_s3_g3' + '/origin_costmap'+str(index)+'.jpg', costmap_img)

  dim = (N*100, N*100)
  costmap_img = cv2.resize(costmap_img, dim, interpolation = cv2.INTER_LINEAR)#INTER_AREA

  cv2.imwrite('/home/joinet/ro_viz/costmap_from_sparse_img/sec2_s3_g3' + '/costmap'+str(index)+'.jpg', costmap_img)
  cv2.waitKey(5000)



if __name__ == '__main__':
  main()

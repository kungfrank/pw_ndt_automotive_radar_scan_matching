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

dir_path = '/home/joinet/ro_viz/ndmap/sparse_temp'
grid_size = 7.0 # 5.0 meters
grid_extend = 20 # 30

N = 30 # resolution in grid

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


  rospy.init_node('Viz_NDMap', anonymous=True)
  rospy.loginfo("Start Viz_NDMap")

  pc_sub = message_filters.Subscriber('/conti_stack_cov/radar_stack', PointCloud2)
  cov_sub = message_filters.Subscriber('/conti_stack_cov/radar_stack_cov', Cov2DArrayStamped)
  ts = message_filters.ApproximateTimeSynchronizer([pc_sub, cov_sub], 10000, 0.1)
  ts.registerCallback(callback)
  rospy.spin()


def callback(pc_msg, cov_msg):
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

  pointcloud_cov = np.empty([4,])
  for cov in cov_msg.data:
    #print(cov)
    point_cov = np.asarray(cov.covariance).reshape((4,))
    pointcloud_cov = np.vstack((pointcloud_cov, point_cov))
  pointcloud_cov = np.delete(pointcloud_cov, 0, 0) # remove first empty item
  #print(pointcloud_cov)


  ################################### Draw ND Map #####################################

	#origin_coordinate = np.array([[0, 0]]).T
  origin = np.array([-grid_extend/2 * grid_size, -grid_extend/2 * grid_size])

  point_set_map = np.empty((grid_extend, grid_extend),dtype=object)
  print(point_set_map)
  for i in range(grid_extend):
      for j in range(grid_extend):
          point_set_map[i,j] = []

  print(point_set_map)

  for i in range(len(point_set_map)):
    for j in range(len(point_set_map[0])):
      print(i,j,len(point_set_map[i][j]))

  for i in range(pointcloud.shape[0]):
    x_ = pointcloud[i][0]
    y_ = pointcloud[i][1]
    sigma_xx = pointcloud_cov[i][0]
    sigma_xy = pointcloud_cov[i][1]
    sigma_yx = pointcloud_cov[i][2]
    sigma_yy = pointcloud_cov[i][3]
    #print(x_,y_)
    for k in range(grid_extend):
      for j in range(grid_extend):
        b_min_x = origin[0] + j * grid_size
        b_min_y = origin[1] + k * grid_size
        b_max_x = b_min_x + grid_size
        b_max_y = b_min_y + grid_size
        #print('bb', b_min_x, b_min_y, b_max_x, b_max_y)
        if x_ > b_min_x and x_ < b_max_x and y_ > b_min_y and y_ < b_max_y:
          #print('check', b_min_x, b_min_y, b_max_x, b_max_y)
          point_set_map[j][k].append(i)
          #print('append~', j, k)

    #
  print('points distribute done.')

  for i in range(len(point_set_map)):
    for j in range(len(point_set_map[0])):
      print(i,j,len(point_set_map[i][j]))
      print(point_set_map[i][j])


  for i in range(grid_extend):
    for j in range(grid_extend):
      print('point_set_map index: ', i, j)
      id_list = point_set_map[i][j]
      q, sigma = calc_mean_and_variance(pointcloud, pointcloud_cov, id_list, i, j)
      print('q:')
      print(q)
      print('sigma:')
      print(sigma)

      # Our 2-dimensional distribution will be over variables X and Y
      X = np.linspace(-grid_size/2, grid_size/2, N)
      Y = np.linspace(-grid_size/2, grid_size/2, N)
      X, Y = np.meshgrid(X, Y)
      # Pack X and Y into a single 3-dimensional array
      pos = np.empty(X.shape + (2,))
      pos[:, :, 0] = X
      pos[:, :, 1] = Y

      if sigma[0,0] == 0 and sigma[0,1] == 0:
        gaussian_img = np.zeros((int(grid_size), int(grid_size)))
        plt.imshow(gaussian_img, cmap = 'jet')

      else:
        rv = multivariate_normal([q[0,0], q[1,0]], sigma)
        gaussian_img = plt.contourf(X, Y, rv.pdf(pos), 30, cmap = 'jet') #viridis

      plt.xticks(())
      plt.yticks(())
      plt.savefig(dir_path+"/ndmap"+str(i)+"_"+str(j)+".png", dpi=100, bbox_inches='tight')
      plt.close()

  ### Combine local distribution to build ND Map ###
  print('Combine local distribution to build ND Map...')
  for i in range(grid_extend):
    for j in range(grid_extend):

      im = cv2.imread(dir_path+"/ndmap"+str(i)+"_"+str(j)+".png")
      #im = cv2.rotate(im, cv2.ROTATE_90_CLOCKWISE)
      s_im = cv2.resize(im,(300,300));
      #cv2.waitKey(0)

      if j==0:
        ndmap_col = s_im
      else:
        ndmap_col = cv2.vconcat([s_im, ndmap_col]) #get_concat_h(ndmap_row, s_im)
        #print(ndmap_col.shape)

      if j==grid_extend-1:
        if i == 0:
          ndmap = ndmap_col
        else:
          ndmap = cv2.hconcat([ndmap, ndmap_col]) #get_concat_v(ndmap, ndmap_row)
  print('done')
  cv2.namedWindow('ndmap', cv2.WINDOW_NORMAL)
  cv2.imshow('ndmap', ndmap)

  #cv2.imwrite('/home/joinet/ro_viz/ndmap_from_sparse_img/sec3_s3_g7_e20_' + '/sparse_ndmap'+str(index)+'.jpg', ndmap)
  cv2.waitKey(0)
    
    
    
    



if __name__ == '__main__':
  main()

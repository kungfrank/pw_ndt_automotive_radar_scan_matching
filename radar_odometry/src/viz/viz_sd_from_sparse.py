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
import time

dir_path = '/home/joinet/ro_viz/sd_from_sparse_txt'

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

def main():

  rospy.init_node('Viz_sd_from_sparse', anonymous=True)
  rospy.loginfo("Start Viz_sd_from_sparse")

  pc_sub = message_filters.Subscriber('/conti_stack_cov/radar_stack', PointCloud2)
  cov_sub = message_filters.Subscriber('/conti_stack_cov/radar_stack_cov', Cov2DArrayStamped)
  ts = message_filters.ApproximateTimeSynchronizer([pc_sub, cov_sub], 10000, 0.1)
  ts.registerCallback(callback)
  rospy.spin()


def callback(pc_msg, cov_msg):
  global index
  index = index+1
  print(pc_msg.header.stamp.to_sec())
  #print('pc_msg.height:')
  #print(pc_msg.height)
  print('pc_msg.width:')
  print(pc_msg.width)

  assert isinstance(pc_msg, PointCloud2)
  gen = point_cloud2.read_points(pc_msg, field_names=("x", "y", "z"), skip_nans=True)
  time.sleep(0.001)

  pointcloud = np.empty([3,])

  start_time = time.time()
  for p in gen:
    #print " x : %.3f  y: %.3f  z: %.3f" %(p[0],p[1],p[2])
    point = np.asarray(p).reshape((3,))
    pointcloud = np.vstack((pointcloud, point))
  pointcloud = np.delete(pointcloud, 0, 0) # remove first empty item
  #print(pointcloud)

  pointcloud_cov = np.empty([4,])
  for cov in cov_msg.data:
    #print(cov)
    point_cov = np.asarray(cov.covariance).reshape((4,))
    pointcloud_cov = np.vstack((pointcloud_cov, point_cov))
  pointcloud_cov = np.delete(pointcloud_cov, 0, 0) # remove first empty item
  #print(pointcloud_cov)
  end_time = time.time()
  print('load pc:', end_time-start_time)

  #for i in range(pointcloud.shape[0]):
  #  x_ = pointcloud[i][0]
  #  y_ = pointcloud[i][1]
  #  sigma_xx = pointcloud_cov[i][0]
  #  sigma_xy = pointcloud_cov[i][1]
  #  sigma_yx = pointcloud_cov[i][2]
  #  sigma_yy = pointcloud_cov[i][3]
  #  print " x : %.3f  y: %.3f, s_xx: %.3f  s_xy: %.3f  s_yx: %.3f s_yy: %.3f" %(x_, y_, sigma_xx,sigma_xy,sigma_yx,sigma_yy)

  # Our 2-dimensional distribution will be over variables X and Y
  #N = 350 #350
  #X = np.linspace(-75, 75, N)
  #Y = np.linspace(-75, 75, N)

  N = 500 #350
  X = np.linspace(-75, -50, N)
  Y = np.linspace(0, 25, N)
  X, Y = np.meshgrid(X, Y)

  # Pack X and Y into a single 3-dimensional array
  pos = np.empty(X.shape + (2,))
  pos[:, :, 0] = X
  pos[:, :, 1] = Y
  Z_all = np.zeros((N, N))

  '''  '''
  start_time = time.time()

  for i in range(pointcloud.shape[0]):
    #print(i)
    Mean = np.array([pointcloud[i][0], pointcloud[i][1]])
    #print Mean[0], Mean[1]
    if Mean[0]<-50 and Mean[0]>-75 and Mean[1]>0 and Mean[1]<25:
      Sigma = np.array([[pointcloud_cov[i][0], pointcloud_cov[i][1]], [pointcloud_cov[i][2], pointcloud_cov[i][3]]])
      rv = multivariate_normal(Mean, Sigma)
      Z = rv.pdf(pos)
      Z_all = Z_all + Z
    #else:
      #print('skip')

  end_time = time.time()
  print('draw distribution:', end_time-start_time)
  ################################ Save Z_all ################################
  '''
  a_file = open(dir_path+"/sec3_21.48_s3_N3000.txt", "w")
  for row in Z_all:
    np.savetxt(a_file, row)
  a_file.close()
  '''
  ################################ End ################################

  ################################ Load Z_all ################################
  '''
  print('Loading Z_all ...')
  Z_all = np.loadtxt(dir_path+"/sec3_21.48_s3.txt").reshape(1500, 1500) # 1500 3000
  print('Load Z_all')
  '''
  ################################ End ################################

  #print(Z_all.shape)
  #print(Z_all.dtype)
  Z_all_norm = Z_all / Z_all.max()
  #Z_all_norm = Z_all_norm * 255.0
  Z_all_norm = Z_all_norm.astype('float64')  #uint8
  #print(Z_all_norm.max())
  #print(Z_all_norm.min())
  print(Z_all_norm.dtype)
  #Z_all = np.expand_dims(Z_all, axis=2)
  #print(Z_all.shape)
  #cv_img = cv2.cvtColor(Z_all_norm, cv2.COLOR_GRAY2BGR)
  cv2.namedWindow('Image', cv2.WINDOW_NORMAL)
  cv2.imshow('Image', Z_all_norm*2)

  Z_all_norm = Z_all_norm * 255.0
  Z_all_norm = Z_all_norm.astype('uint8')
  cv2.namedWindow('Image_uint8', cv2.WINDOW_NORMAL)
  cv2.imshow('Image_uint8', Z_all_norm)

  cv2.imwrite('/home/joinet/ro_viz/sd_from_sparse_img/sec2_s3_small_N500/' + str(index) + '.png', Z_all_norm)
  cv2.waitKey(100)


  ### Generate semi-dense plot in paper ###
  '''
  Z_all[Z_all >= 5] = 5 #55
  gaussian_img = plt.contourf(X, Y, Z_all, 70, cmap = 'gray') #viridis jet
  plt.xticks(())
  plt.yticks(())
  plt.show()
  plt.close()
  '''

  '''
  Z_all[Z_all >= 85] = 85

  plt.contourf(X, Y, Z_all, 30, alpha=1.0, cmap='gray')
  plt.xticks(())
  plt.yticks(())
  plt.show()

  plt.contourf(X, Y, Z_all, 30, alpha=1.0)
  #C = plt.contour(X, Y, Z_all, 70, colors='red', linewidth=.2)
  #plt.clabel(C, inline=True, fontsize=10)
  plt.xticks(())
  plt.yticks(())
  plt.show()
  '''


if __name__ == '__main__':
  main()

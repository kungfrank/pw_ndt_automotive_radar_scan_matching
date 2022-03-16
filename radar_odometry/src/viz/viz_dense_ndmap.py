#!/usr/bin/env python2

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import numpy as np
from scipy.stats import multivariate_normal
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D

import cv2
import time
#from PIL import Image
import imutils

dir_path = '/home/joinet/ro_viz/ndmap/dense_temp'
grid_size = 30 # pixels
N = 30 # resolution in grid
thres = 70 #0~255

from pylab import rcParams
rcParams['figure.figsize'] = 10, 10

global bridge
global index
index = 0

def multivariate_gaussian(pos, mu, Sigma):
    """Return the multivariate Gaussian distribution on array pos.

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

def calc_mean_and_variance(x,y,img_):
  print('--------------')
  print(x,y)
  num = 0
  v = 0.0
  s = np.array([[0, 0]]).T
  q = np.array([[0, 0]]).T
  for i in range(x,x+grid_size):
    for j in range(y,y+grid_size):
      w = max(img_[i,j] - thres, 0)
      v = v + w
      s = s + w * np.array([[i,j]]).T
      if w > 0:
        num = num+1

  if num < grid_size**2 /50:
    print('empty!')
    return np.array([[0, 0]]).T, np.array([[0, 0], [0, 0]])

  q = s/v
  print(q)
  q_unbias = q - np.array([[x+grid_size/2, y+grid_size/2]]).T
  #print(q_unbias)

  sigma = np.array([[0, 0],
                    [0, 0]])
  for i in range(x,x+grid_size):
    for j in range(y,y+grid_size):
      w = max(img_[i,j] - thres, 0)
      #if w>0:
      #  print(img_[i,j], i, j)
      sigma = sigma + np.matmul( (np.array([[i,j]]).T - q), (np.array([[i,j]]).T - q).T ) * w
  sigma = sigma/v

  print(sigma)

  return q_unbias, sigma


def main():

  rospy.init_node('Viz_dense_NDMap', anonymous=True)
  rospy.loginfo("Start Viz_dense_NDMap")
  rospy.Subscriber("radar_cart", Image, callback)
  rospy.spin()


def callback(radar_msg):

  global index
  index = index + 1
  print(index)

  global bridge
  img = bridge.imgmsg_to_cv2(radar_msg, "32FC1")
  print(img.max())
  print(img.min())

  img = img * 255.
  img = img.astype('uint8')

  print(img.max())
  print(img.min())

  #img_ = cv2.imread('/home/joinet/thres_dense.png', 0)
  #print(img_.max())
  #print(img_.min())

  cv2.namedWindow('image', cv2.WINDOW_NORMAL)
  cv2.imshow('image',img)
  cv2.waitKey(0)


  print(img.shape)
  img = cv2.resize(img,(900,900));
  print(img.shape)

  for i in range(img.shape[0]/grid_size):
    for j in range(img.shape[1]/grid_size):
      q, sigma = calc_mean_and_variance(i*grid_size, j*grid_size, img)

      # Our 2-dimensional distribution will be over variables X and Y
      X = np.linspace(-grid_size/2, grid_size/2, N)
      Y = np.linspace(-grid_size/2, grid_size/2, N)
      X, Y = np.meshgrid(X, Y)
      # Pack X and Y into a single 3-dimensional array
      pos = np.empty(X.shape + (2,))
      pos[:, :, 0] = X
      pos[:, :, 1] = Y

      if sigma[0,0] == 0 and sigma[0,1] == 0:
        #gaussian_img = plt.contourf(X, Y, f(X, Y), 70)
        gaussian_img = np.zeros((grid_size, grid_size))
        plt.imshow(gaussian_img, cmap = 'jet')

      else:
        rv = multivariate_normal([q[0,0], q[1,0]], sigma)
        gaussian_img = plt.contourf(X, Y, rv.pdf(pos), 70, cmap = 'jet') #viridis
        #C = plt.contour(X, Y, Z*10, 30, colors='black', linewidth=.5)
        #plt.clabel(C, inline=True, fontsize=10)

      plt.xticks(())
      plt.yticks(())
      plt.savefig(dir_path+"/ndmap"+str(i)+"_"+str(j)+".png", dpi=100, bbox_inches='tight')
      plt.close()

  for i in range(img.shape[0]/grid_size):
    for j in range(img.shape[1]/grid_size):

      im = cv2.imread(dir_path+"/ndmap"+str(i)+"_"+str(j)+".png")
      im = cv2.rotate(im, cv2.ROTATE_90_CLOCKWISE)
      s_im = cv2.resize(im,(100,100));
      #cv2.waitKey(0)

      if j==0:
        ndmap_row = s_im
      else:
        ndmap_row = cv2.hconcat([ndmap_row, s_im]) #get_concat_h(ndmap_row, s_im)
        print(ndmap_row.shape)

      if j==img.shape[1]/grid_size-1:
        if i == 0:
          ndmap = ndmap_row
        else:
          ndmap = cv2.vconcat([ndmap, ndmap_row]) #get_concat_v(ndmap, ndmap_row)

  cv2.namedWindow('ndmap', cv2.WINDOW_NORMAL)
  cv2.imshow('ndmap', ndmap)
  cv2.waitKey(0)

  #cv2.imwrite('/home/joinet/ro_viz/ndmap_from_sparse_img/sec3_s3_g7_e20' + '/sparse_ndmap'+str(index)+'.jpg', ndmap)
  #cv2.waitKey(3000)





if __name__ == '__main__':
  global bridge
  bridge = CvBridge()
  main()








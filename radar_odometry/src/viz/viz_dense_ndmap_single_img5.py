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

temp_dir_path = '/home/joinet/ro_viz/ndmap/dense_temp5'
grid_size = 50 #56 # pixels
N = 30 # resolution in grid
shift = 70 #0~255

from pylab import rcParams
rcParams['figure.figsize'] = 10, 10

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
  ##print('--------------')
  ##print(x,y)
  num = 0
  v = 0.0
  s = np.array([[0, 0]]).T
  q = np.array([[0, 0]]).T
  for i in range(x,x+grid_size):
    for j in range(y,y+grid_size):
      w = max(img_[i,j] - shift, 0)
      v = v + w
      s = s + w * np.array([[i,j]]).T
      if w > 0:
        num = num+1

  if num < grid_size**2 /50:
    ##print('empty!')
    return np.array([[0, 0]]).T, np.array([[0, 0], [0, 0]])

  q = s/v
  ##print(q)
  q_unbias = q - np.array([[x+grid_size/2, y+grid_size/2]]).T
  #print(q_unbias)

  sigma = np.array([[0, 0],
                    [0, 0]])
  for i in range(x,x+grid_size):
    for j in range(y,y+grid_size):
      w = max(img_[i,j] - shift, 0)
      #if w>0:
      #  print(img_[i,j], i, j)
      sigma = sigma + np.matmul( (np.array([[i,j]]).T - q), (np.array([[i,j]]).T - q).T ) * w
  sigma = sigma/v

  ##print(sigma)

  return q_unbias, sigma



def get_concat_h(im1, im2):
    dst = Image.new('RGB', (im1.width + im2.width, im1.height))
    dst.paste(im1, (0, 0))
    dst.paste(im2, (im1.width, 0))
    return dst

def get_concat_v(im1, im2):
    dst = Image.new('RGB', (im1.width, im1.height + im2.height))
    dst.paste(im1, (0, 0))
    dst.paste(im2, (0, im1.height))
    return dst



#img = cv2.imread('/home/joinet/ro_viz/0270.30553.png',0) #0262.55042.png

'''
print 'Number of arguments:', len(sys.argv), 'arguments.'
if len(sys.argv)<2:
  print 'please assign part number.'
  exit()
else:
  print(sys.argv[1])
  part_num = sys.argv[1]
'''

part_num = '1'
dir = '/mnt/Disk2/Oxford/radar_cart_0.125_1001_timestamped/part'+part_num

for filename in os.listdir(dir):
    print(filename)
    path = dir+'/' + filename + '/'

    #ndmap_dir_path = os.path.join(dir+'/', 'ndmap_'+filename+'_new')
    #os.mkdir(ndmap_dir_path)
    #print("Directory '%s' created" %('ndmap_'+filename))

    thres_dir_path = os.path.join(dir+'/', 'thres_'+filename)
    #os.mkdir(thres_dir_path)
    #print("Directory '%s' created" %('thres_'+filename))

    ###--- Write Radar Image ---###
    radar_init = False

    timestamps_path = path+'radar_t_list'
    radar_timestamps = np.loadtxt(timestamps_path, delimiter=' ', usecols=[0], dtype=np.float64)

    #radar_timestamps = np.genfromtxt(timestamps_path,dtype='str')

    for radar_timestamp in radar_timestamps:

      if radar_timestamp<=430: #128
        continue
      print(radar_timestamp)

      #print(radar_timestamp)
      curr_radar_t_str = str("%010.5f"%radar_timestamp)
      print(curr_radar_t_str)

      img = cv2.imread(path+curr_radar_t_str+'.png',0)
      #cv2.namedWindow('image', cv2.WINDOW_NORMAL)
      #cv2.imshow('image',img)
      #cv2.waitKey(0)

      '''
      hist = cv2.calcHist([img], [0], None, [256], [0, 256])
      plt.figure()
      plt.title("Grayscale Histogram")
      plt.xlabel("Bins")
      plt.ylabel("# of Pixels")
      plt.plot(hist)
      plt.xlim([0, 256])
      plt.show()
      '''
      #print(img.dtype)

      ret,thresh_img = cv2.threshold(img,70,255,cv2.THRESH_TOZERO)
      #cv2.namedWindow('thres', cv2.WINDOW_NORMAL)
      #cv2.imshow('thres',thresh_img)
      #cv2.imwrite(dir+'/'+'thres_'+filename+'/'+curr_radar_t_str+'.png', thresh_img)
      #print(thresh_img.dtype)
      '''
      for i in range(thresh_img.shape[0]):
        for j in range(thresh_img.shape[1]):
          if thresh_img[i,j] < 70 and thresh_img[i,j]!=0:
            print(thresh_img[i,j])
      '''
      '''
      hist = cv2.calcHist([thresh_img], [0], None, [256], [0, 256])
      plt.figure()
      plt.title("Grayscale Histogram")
      plt.xlabel("Bins")
      plt.ylabel("# of Pixels")
      plt.plot(hist)
      plt.xlim([0, 256])
      plt.show()
      '''
      cv2.waitKey(20)

      img = thresh_img

      #print(img.shape)
      #img = cv2.resize(img,(900,900)); /////////////////////////////////////
      #print(img.shape)

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
            gaussian_img = plt.contourf(X, Y, rv.pdf(pos), 70, cmap = 'jet') #viridis 70
            #C = plt.contour(X, Y, Z*10, 30, colors='black', linewidth=.5)
            #plt.clabel(C, inline=True, fontsize=10)

          plt.xticks(())
          plt.yticks(())
          plt.savefig(temp_dir_path+"/ndmap"+str(i)+"_"+str(j)+".png", dpi=100, bbox_inches='tight')
          plt.close()

          #plt.show()

      ########################################################################################
      for i in range(img.shape[0]/grid_size):
        for j in range(img.shape[1]/grid_size):

          im = cv2.imread(temp_dir_path+"/ndmap"+str(i)+"_"+str(j)+".png")
          im = cv2.rotate(im, cv2.ROTATE_90_CLOCKWISE)
          s_im = cv2.resize(im,(100,100)); # 100
          #cv2.waitKey(0)

          if j==0:
            ndmap_row = s_im
          else:
            ndmap_row = cv2.hconcat([ndmap_row, s_im]) #get_concat_h(ndmap_row, s_im)
            #print(ndmap_row.shape)

          if j==img.shape[1]/grid_size-1:
            if i == 0:
              ndmap = ndmap_row
            else:
              ndmap = cv2.vconcat([ndmap, ndmap_row]) #get_concat_v(ndmap, ndmap_row)

      #cv2.namedWindow('ndmap', cv2.WINDOW_NORMAL)
      #cv2.imshow('ndmap', ndmap)
      cv2.imwrite(dir+'/'+'ndmap_'+filename+'_new/'+curr_radar_t_str+'.png', ndmap)
      cv2.waitKey(10)







#!/usr/bin/env python2

import rospy
import message_filters
import cv2
import time

from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from ro_msg.msg import Cov2D
from ro_msg.msg import Cov2DArrayStamped
import time
import numpy as np

index = 0

def main():

  rospy.init_node('buffer_test', anonymous=True)
  rospy.loginfo("Start buffer_test")

  pc_sub = message_filters.Subscriber('/conti_stack_cov/radar_stack', PointCloud2)
  #cache = message_filters.Cache(sub, 100)
  cov_sub = message_filters.Subscriber('/conti_stack_cov/radar_stack_cov', Cov2DArrayStamped)
  #cache = message_filters.Cache(sub, 100)
  ts = message_filters.ApproximateTimeSynchronizer([pc_sub, cov_sub], 1000000, 0.1)
  ts.registerCallback(callback)
  rospy.spin()


def callback(pc_msg, cov_msg):
  global index
  index = index+1
  print(index)
  print(pc_msg.header.stamp.to_sec())
  print(cov_msg.header.stamp.to_sec())
  time.sleep(0.5)
  


if __name__ == '__main__':
  main()

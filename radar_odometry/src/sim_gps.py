#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

pub = rospy.Publisher('gps', PoseWithCovarianceStamped, queue_size=10)
count = 0

def callback(data):
    global count
    count = count + 1

    if count == 120:
      rospy.loginfo(rospy.get_caller_id() + " I heard \n %s", data.pose.pose.position)

      noise = np.random.normal(0,1,2)
      #print(noise)
      #print(noise[0])

      msg = PoseWithCovarianceStamped()
      msg.header = data.header
      msg.header.frame_id = "origin"
      msg.pose.pose.position.x = data.pose.pose.position.x + noise[0]
      msg.pose.pose.position.y = data.pose.pose.position.y + noise[1]

      cov = np.array([0.0]*36).reshape(6,6)
      # position covariance
      cov[0,0] = 10
      cov[1,1] = 10

      msg.pose.covariance = tuple(cov.ravel().tolist())

      pub.publish(msg)

      count = 0



def listener():

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("gt_odom", Odometry, callback)
    #pub = rospy.Publisher('gps', PoseWithCovarianceStamped, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    listener()

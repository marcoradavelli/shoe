#!/usr/bin/env python

#import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/kinect2/sd/image_depth_rect",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
      
      cv_image_array = np.array(cv_image, dtype = np.dtype('f8'))
      # Normalize the depth image to fall between 0 (black) and 1 (white)
      # http://docs.ros.org/electric/api/rosbag_video/html/bag__to__video_8cpp_source.html lines 95-125
      cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
      # Resize to the desired size
      #cv_image_resized = cv2.resize(cv_image_norm, self.desired_shape, interpolation = cv2.INTER_CUBIC)
      
      print(len(cv_image))
#      for point in cv_image:
#        sys.stdout.write(" %d %f" %(len(point),point[0]))
      
    except CvBridgeError as e:
      print(e)

    #(rows,cols,channels) = cv_image.shape
    #if cols > 60 and rows > 60 :
    #  cv2.circle(cv_image, (50,50), 10, 255)#

    #cv2.imshow("Image window", cv_image_norm)
    plt.imshow(cv_image)
    plt.colorbar()
    plt.show()
    cv2.waitKey(3)
    
def main(args):
  ic = image_converter()
  rospy.init_node('kinect_listener', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
#def listener():

#	self.bridge = CvBridge()

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #rospy.init_node('kinect_listener', anonymous=True)

    #rospy.Subscriber("/kinect2/sd/image_color_rect", Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
#    rospy.spin()

#if __name__ == '__main__':
#    listener()

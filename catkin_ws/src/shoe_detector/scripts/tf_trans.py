#!/usr/bin/env python
import roslib
import rospy
from fiducial_msgs.msg import FiducialTransformArray
import tf

def callback(data):


    br = tf.TransformBroadcaster()


    br.sendTransform((data.transforms[0].transform.translation.x,data.transforms[0].transform.translation.y,data.transforms[0].transform.translation.z),
                     (data.transforms[0].transform.rotation.x,data.transforms[0].transform.rotation.y,data.transforms[0].transform.rotation.z,data.transforms[0].transform.rotation.w),
                     rospy.Time.now(),
                     'mark1',
                     "kinect2_rgb_optical_frame")

if __name__ == '__main__':
    rospy.init_node('tf_trans', anonymous=True)

    rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, callback)

    print("Listener")
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

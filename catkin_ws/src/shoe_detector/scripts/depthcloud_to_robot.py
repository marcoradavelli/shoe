#!/usr/bin/env python
import rospy
import numpy
#from std_msgs.msg import String
from sensor_msgs.msg import Image
from fiducial_msgs.msg import FiducialTransformArray

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    #print(data.transforms)
    print("It works")
    #print (data.transforms[0])
    #print (PointCoordinRobotFrame)


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('kinect_to_robot_tfm', anonymous=True)

    rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, callback)

    print("Listener")
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()

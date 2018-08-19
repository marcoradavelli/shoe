#!/usr/bin/env python
import rospy
import numpy
#from std_msgs.msg import String
from sensor_msgs.msg import Image
from fiducial_msgs.msg import FiducialTransformArray

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    #print(data.transforms)
    print("-------------------------")
    print (data.transforms[0])
    # we extract it for our calculations
    # ID of our marker
    ArUco_ID = data.transforms[0].fiducial_id
    # Vector 3, position of marker
    X = data.transforms[0].transform.translation.x
    Y = data.transforms[0].transform.translation.y
    Z = data.transforms[0].transform.translation.z
    # Quaternion of our marker
    qX = data.transforms[0].transform.rotation.x
    qY = data.transforms[0].transform.rotation.y
    qZ = data.transforms[0].transform.rotation.z
    qW = data.transforms[0].transform.rotation.w
    TF1 = numpy.matrix ([ [1 - 2*qY*qY - 2*qZ*qZ,  2*qX*qY - 2*qZ*qW,  2*qX*qZ + 2* qY*qW,  X],
            [2*qX*qY + 2*qZ*qW,  1 - 2*qX*qX - 2*qZ*qZ, 2*qY*qZ - 2*qX*qW,  Y],
            [2*qX*qZ - 2*qY*qW,  2*qY*qZ + 2*qX*qW,  1 - 2*qX*qX - 2*qY*qY, Z],
            [0, 0, 0, 1 ]])
    TF2 = numpy.matrix ([ [ 1,   0,  0,  0.28725],
                         [ 0,   1,  0,  0.30425],
                         [ 0,   0,  1,  0.0004],
                         [ 0,   0,  0,      1]])

    TF = numpy.matmul(TF1,TF2)
    TFresult = numpy.linalg.inv(TF)

    print (TFresult)



    #for p in data.transforms:
    #  print(p)

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

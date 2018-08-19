#!/usr/bin/env python
import rospy
import numpy
#from std_msgs.msg import String
#from sensor_msgs.msg import Image
from fiducial_msgs.msg import FiducialTransformArray

#pub = None
#sub = None

def callback(data):
    matric_list = None
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    #print(data.transforms)
    is_world_marker = None
    is_foot_in_world = None
    #print("-------------------------")
    #print (type(data.transforms))

    markers = iter(data.transforms)
    for marker in markers:
        if (marker.fiducial_id == 11):
            is_world_marker = True

            X = marker.transform.translation.x
            Y = marker.transform.translation.y
            Z = marker.transform.translation.z
            # Quaternion of our marker
            qX = marker.transform.rotation.x
            qY = marker.transform.rotation.y
            qZ = marker.transform.rotation.z
            qW = marker.transform.rotation.w
            # Transformation matrix from kinect to marker
            TF1 = numpy.matrix ([   [1 - 2*qY*qY - 2*qZ*qZ,     2*qX*qY - 2*qZ*qW,      2*qX*qZ + 2*qY*qW,     X],
                                    [2*qX*qY + 2*qZ*qW,         1 - 2*qX*qX - 2*qZ*qZ,  2*qY*qZ - 2*qX*qW,      Y],
                                    [2*qX*qZ - 2*qY*qW,         2*qY*qZ + 2*qX*qW,      1 - 2*qX*qX - 2*qY*qY,  Z],
                                    [0,                         0,                      0,                      1 ]])
            #Transformation matrix from marker to robot
            TF2 = numpy.matrix ([[ 1,   0,  0,  0.28725],
                                 [ 0,   1,  0,  0.30425],
                                 [ 0,   0,  1,  -0.00045],
                                 [ 0,   0,  0,      1]])

            # position of robot in kinect space
            TF = numpy.matmul(TF1,TF2)
            # position of kinect2 in robot space
            TFresult = numpy.linalg.inv(TF)
            #Precision setting in printing results
            #numpy.set_printoptions(precision=3,suppress=True,linewidth=120)
            #printing position of kinect2 in robot space
            #print("\n\nkinect2 coordinates in robot's frame")
            #print (TFresult)
            break


    if is_world_marker is not None:
        for marker in markers:
            if (marker.fiducial_id == 123):
                is_foot_in_world = True
                X = marker.transform.translation.x
                Y = marker.transform.translation.y
                Z = marker.transform.translation.z

                # Coordinates of foot point in kinect's space
                FootCoordinKinectFrame = numpy.matrix([[X], [Y], [Z], [1]])
                # Finding foot coordinates in robot's space
                FootCoordinRobotFrame = numpy.matmul(TFresult, FootCoordinKinectFrame)
                # printing position of foot in robot space
                #print ("\n\foot coordinates in robot's frame")
                #print (FootCoordinRobotFrame)
                matric_list = [[TFresult],[FootCoordinRobotFrame]]
                break

    #pub.publish(matric_list)

def random_number_publisher():
    rospy.init_node('random_number')
    pub=rospy.Publisher('rand_no', String, queue_size=10)
    rate= rospy.Rate(2)
#generate a random number at every 2 seconds
    while not rospy.is_shutdown():
        rospy.loginfo("12345")
        pub.publish("12345")
        rate.sleep()

def tfm_calculator():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #global sub, pub
    rospy.init_node('kinect_to_robot_tfm', anonymous=True)
    #Publisher
    #pub = rospy.Publisher('output_data', list)
    # Listen
    sub = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, callback)
    print ("Calculation finished. \n")
    #print("Listener")
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        random_number_publisher()
    except rospy.ROSInterruptException:
        pass


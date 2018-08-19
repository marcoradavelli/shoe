#!/usr/bin/env python
import rospy
import numpy
import pcl
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from fiducial_msgs.msg import FiducialTransformArray
import math
import urx

fl = 1
fl2 = 1
TFresult = []
pub = rospy.Publisher('Hello', PointCloud2, queue_size=1000)


def sortByX(inputList):
    return inputList[0]

def sortByY(inputList):
    return inputList[1]

def sortByZ(inputList):
    return inputList[2]

def ros_to_pcl(ros_cloud):

    global fl
    global TFresult
    if fl ==1 and not (TFresult == []):
        fl = 0
        print(TFresult)
        print("front side")
        print(numpy.matmul(TFresult, [-0.207,0.056,1.1151,1]))
        print("right side")
        print(numpy.matmul(TFresult, [-0.302,0.06,1.156,1]))
        print("highest point")
        print(numpy.matmul(TFresult, [-0.306,-0.023,1.168,1]))
        print("furthest left point")
        print(numpy.matmul(TFresult, [-0.358, 0.031, 1.2, 1]))
        print("I am here")
        """ Converts a ROS PointCloud2 message to a pcl PointXYZRGB
    
            Args:
                ros_cloud (PointCloud2): ROS PointCloud2 message
    
            Returns:
                pcl.PointCloud_PointXYZRGB: PCL XYZRGB point cloud
        """
        points_list = []
        for data in pc2.read_points(ros_cloud, skip_nans=True):
            points_list.append([data[0], data[1], data[2], data[3]])
        robotPointsList = []
        for curInfo in points_list:
            temp = curInfo[:-1]
            temp.append(1)
            finall = (numpy.matmul(TFresult, temp))
            finall = finall.tolist()
            finall = finall[0][:-1]
            robotPointsList.append(finall)
        # print(robotPointsList)
        print(len(robotPointsList))
        robotCorpPointsList = []
        ii = 0
        for q in robotPointsList:
            if q[2]> 0.01 and q[2] < 0.2 and q[1] > -0.2 and q[1] < 0.1 and q[0] > -0.8 and q[0] < -0.5:

                q.append(0)
                q.append(0)
                if ii%2:
                    q.append(0)
                else:
                    q.append(math.pi/2)
                ii= ii + 1
                robotCorpPointsList.append(q)
        robotCorpPointsList.sort(key=sortByX,reverse=True)

        # Find the shoe rectangle
        maxX =robotCorpPointsList
        maxX.sort(key=sortByX,reverse=True)
        maxX = maxX[0]
        print(maxX)
        minY =robotCorpPointsList
        minY.sort(key=sortByY)
        minY = minY[0]
        middlePointsList = []
        eps = 0.05
        shoeL = 0.2

        print(len(robotCorpPointsList))
        # for curList in robotCorpPointsList:
        #     if curList[1] <= (maxX[1] + eps) and curList[1] >= (maxX[1] - eps) and curList[0] < (maxX[0] - shoeL):
        #         middlePointsList.append(curList)
        #
        # print(middlePointsList)
        # # make points more sparse
        # for i in range(1):
        #     indexes = sorted(list(range(0, len(middlePointsList), 2)), reverse=True)
        #     for index in indexes:
        #         del middlePointsList[index]

        print(maxX)
        for curstep in range(0,int(shoeL/0.02)):
            middlePointsList.append([maxX[0]-curstep*0.015,maxX[1],maxX[2] + curstep*0.0058,0,0,0])
        print(len(middlePointsList))

        startPointsList = []
        endPointLists = []

        for curList in middlePointsList:
            temp = [curList[0],(curList[1] - (curList[1] - minY[1])),minY[2],0,0,0]
            startPointsList.append(temp)
            temp = [curList[0], (curList[1] + (curList[1] - (minY[1] - 0.035))), minY[2], 0, 0, 0]
            endPointLists.append(temp)

        #movement
        robot = urx.Robot("10.0.0.2", use_rt=True)

        robot.movel([-0.52,-0.05,0.24,0,0,0], 0.5, 1, 0.01)

        robot.movel(startPointsList[0], 0.5, 1, 0.01)

        # cmd = "def movec_soft():\n" + "robot.movec(middlePointsList[tar], endPointLists[tar], acc=0.5, vel=0.6)\n " + "robot.stopl(0.5)\n" + "end\n"

        for tar in range(len(middlePointsList)):
            if tar%2 :
                robot.movec(middlePointsList[tar], endPointLists[tar], acc=0.3, vel=0.25)
            else:
                robot.movec(middlePointsList[tar], startPointsList[tar], acc=0.3, vel=0.25)


        robot.movel([-0.52, -0.05, 0.24, 0, 0, 0], 0.5, 1, 0.01)
        #  FOR SIMPLE CASE
        # #make points more sparse
        # for i in range(7):
        #     indexes = sorted(list(range(0,len(robotCorpPointsList),3)), reverse=True)
        #     for index in indexes:
        #         del robotCorpPointsList[index]
        #
        # print(len(robotCorpPointsList))
        # with open('robotCorpedFrame.txt','w') as f:
        #     for item in robotCorpPointsList:
        #         f.write("%s\n" % item)
        #
        # with open('robotFrame.txt','w') as f:
        #     for item in robotPointsList:
        #         f.write("%s\n" % item)
        #
        #
        # # inverse task
        #
        # kineticPoints = []
        #
        # TFnew = numpy.linalg.inv(TFresult)
        # for curInfo in robotCorpPointsList:
        #     finall = (numpy.matmul(TFnew, temp))
        #     finall = finall.tolist()
        #     finall = finall[0][:-1]
        #     kineticPoints.append(finall)
        #
        # base_link_point2d = PointCloud2()
        # base_link_point2d.header.frame_id = "kinect2_rgb_optical_frame"
        # received_point = pc2.create_cloud_xyz32(base_link_point2d.header,points=kineticPoints)
        #
        # global pub
        #
        # pub.publish(received_point)
        #
        #
        #
        # robot = urx.Robot("10.0.0.2", use_rt=True)
        #
        # robot.movels(robotCorpPointsList, 2, 2, 0.1, wait=True)
        # robot.movej([math, -math.radians(70),math.radians(60),0,0,0], 0.5, 0.5, 0.01)



        # pcl_data = pcl.PointCloud_PointXYZRGB()
        # pcl_data.from_list(points_list)
        # fl = 0
        # return pcl_data
        print("it moves")
    else:
        pass

def callback(data):
    global TFresult
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

            # print(TFresult)
            #Precision setting in printing results
            #numpy.set_printoptions(precision=3,suppress=True,linewidth=120)
            #printing position of kinect2 in robot space
            #print("\n\nkinect2 coordinates in robot's frame")
            #print (TFresult)
            # PointCoordinKinectFrame = [X,Y,Z,1]
            #     # Finding point coordinates in robot's space
            # PointCoordinRobotFrame = numpy.matmul(TFresult, PointCoordinKinectFrame)
            # #Precision setting in printing results
            # numpy.set_printoptions(precision=3,suppress=True,linewidth=120)
            #
            # #printing position of kinect2 in robot space
            # print("\n\nkinect2 coordinates in roots frame")
            # # print (TFresult)
            #
            # #printing position of point in robot space
            # print ("\n\npoint coordinates in robot frame")
            # print (PointCoordinRobotFrame)

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
                global fl2
                if fl2 == 1:
                    fl2=0
                    print("ID 123")
                    print (FootCoordinRobotFrame)

                matric_list = [[TFresult],[FootCoordinRobotFrame]]
                break

    
def listener():
    # robot = urx.Robot(args['robot_ip_adress'],use_rt=args['realtime_controller'])
    rospy.init_node('pointCloud', anonymous=True)

    rospy.Subscriber("kinect2/qhd/points", PointCloud2, ros_to_pcl)
    rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


def update_args(arg_defaults):
    '''
    Look up parameters starting in the driver's private parameter space, but
    also searching outer namespaces.  Defining them in a higher namespace allows
    the axis_ptz.py script to share parameters with the driver
    :return: available args from roslaunch files
    :rtype: dict
    '''
    args = {}
    for name, val in arg_defaults.iteritems():
        full_name = rospy.search_param(name)  # search without postfix
        if full_name is None:  # search with postfix
            full_name = rospy.search_param(name)
        if full_name is None:  # use default
            args[name] = val
        else:
            args[name] = rospy.get_param(full_name, val)
    return (args)

if __name__ == '__main__':
    args_default = {
        'robot_ip_adress': '',
        'realtime_controller': ''
    }
    args = update_args(args_default)
    listener()

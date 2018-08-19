#!/usr/bin/env python
import rospy

from geometry_msgs.msg._WrenchStamped import WrenchStamped


def callback(data):

    #print(data.header)
    print ('\n\nListenig Force and Torque of NetF/T mini 40 sensor')
    print (data.wrench)

    #Extracting date from sensor
    '''
    force_x = data.wrench.force.x
    force_y = data.wrench.force.y
    force_z = data.wrench.force.z
    torque_x = data.wrench.torque.x
    torque_y = data.wrench.torque.y
    torque_z = data.wrench.torque.z
    '''

def node_listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #global sub, pub
    rospy.init_node('tf_sensor_listener', anonymous=True)
    # Listen
    sub = rospy.Subscriber("/netft_data", WrenchStamped, callback)
    print("Listener")
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    node_listener()


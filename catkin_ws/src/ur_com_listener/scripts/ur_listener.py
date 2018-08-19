#!/usr/bin/env python
import rospy
import urx
import math3d as m3d
import tf


rospy.spin()


from std_msgs.msg import String


def callback(data):
    rospy.loginfo(data.data)
    
def listener():
    robot = urx.Robot(args['robot_ip_adress'],use_rt=args['realtime_controller'])
    rospy.init_node('ur_listener', anonymous=True)

    rospy.Subscriber("Hello", String, callback)

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

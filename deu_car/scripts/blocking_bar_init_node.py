#! /usr/bin/env python

import rospy
from blocking_bar import Blocking_Bar
if __name__ == '__main__':
    rospy.init_node('blocking_bar')
    rospy.spin()
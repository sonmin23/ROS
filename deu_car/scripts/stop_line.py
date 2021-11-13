#! /usr/bin/env python
import cv2
import numpy
import numpy as np

import cv_bridge
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Image

##detector stop_line
class Stop_Line():

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        # cv2.namedWindow("window", 1)
        self.stop_line_toggle = None
        # self.twist = Twist()
        self.stop_line_pub = rospy.Publisher('stop_line', String, queue_size=1)
        # self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 255, 255])
        wmask = cv2.inRange(hsv, lower_white, upper_white)
        h, w, d = image.shape

        wmask[0:360, 0:w] = 0
        wmask[380:h, 0:w] = 0
        wmask[0:h, 0:w / 3] = 0
        wmask[0:h, w / 3 + 30:w] = 0
        _, stop_line_check , _ = cv2.findContours(wmask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(stop_line_check) >= 1:
            self.stop_line_toggle = 'STOP_LINE'
            self.stop_line_pub.publish(self.stop_line_toggle)
            # self.twist.linear.x = 0.0
            # self.cmd_vel_pub.publish(self.twist)
        else:
            self.stop_line_toggle = 'NO_STOP_LINE'
            self.stop_line_pub.publish(self.stop_line_toggle)

if __name__ == "__main__":
    rospy.init_node('stop_line')
    # while not rospy.is_shutdown():
    detector_stop_line = Stop_Line()
      # rate = rospy.Rate(20)
      # rate.sleep()
    rospy.spin()
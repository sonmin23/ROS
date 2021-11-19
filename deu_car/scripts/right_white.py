#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        #    cv2.namedWindow("window", 1)
        self.image_sub = rospy.Subscriber('right_camera/rgb/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.twist = Twist()

    def image_callback(self, msg):
        right_white_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(right_white_image, cv2.COLOR_BGR2HSV)

        lower_white = numpy.array([0, 0, 190])
        upper_white = numpy.array([255, 40, 255])

        mask = cv2.inRange(hsv, lower_white, upper_white)

        h, w, d = right_white_image.shape
        mask[0:h, 0:240] = 0
        mask[0:120, 0:w] = 0
        M = cv2.moments(mask)

        if M['m00'] > 0:
            cx_white = int(M['m10'] / M['m00'])
            cy_white = int(M['m01'] / M['m00'])

            cx = (cx_white + cx_white - 200) // 2
            # cx = (cx_yellow + cx_white) //2
            cy = int(M['m01'] / M['m00'])

            cv2.circle(right_white_image, (cx, cy), 20, (0, 0, 255), -1)
            #      cv2.circle(image, (cx_white, cy_white), 20, (255,255,255), -1)
            #      cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
            # BEGIN CONTROL
            err = cx - w / 2
            self.twist.linear.x = 0.9

            self.twist.angular.z = -float(err) / 100
            self.cmd_vel_pub.publish(self.twist)
            # END CONTROL
        cv2.imshow("window", mask)
        cv2.imshow("RIGHT", right_white_image)
        cv2.waitKey(3)


rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
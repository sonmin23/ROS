#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from base_move import BaseMove

stop_line_toggle = False

#stop_line sub callback
def stop_line_ch(msg):
    global stop_line_toggle
    if msg.data == 'STOP_LINE':
        stop_line_toggle = True
    if msg.data == 'NO_STOP_LINE':
        stop_line_toggle = False
#stop_line_toggle sub
sto_line_sub = rospy.Subscriber('stop_line', String, stop_line_ch)

class RightWhiteLine:
    def __init__(self):
        self.robot_controller = BaseMove()
        self.bridge = cv_bridge.CvBridge()
        #cv2.namedWindow("window", 1)
        self.image_sub = rospy.Subscriber('right_camera/rgb/image_raw', Image, self.image_callback)
        global stop_line_toggle
        self.stop_line_cnt = 0
        self.areaindex = 0
        self.time = rospy.Time.now()
        self.area = 0
        self.lower_color = numpy.array([0, 0, 200])
        self.upper_color = numpy.array([0, 0, 255])
        self.parking = 0
        self.parking_check = False
        self.velocity = 0.4
        self.status = 0

    def image_callback(self, msg):
        try:
            if stop_line_toggle == True:
                if self.time + rospy.Duration(5) < rospy.Time.now():
                    rospy.loginfo('DETECTOR STOP LINE')
                    self.stop_line_cnt += 1
                    # if self.check == 1:
                    #     self.check = 0
                    # self.check = 1
                    rospy.sleep(3)
                    self.time = rospy.Time.now()
            if self.stop_line_cnt == 1:
                self.velocity = 0.2
            elif self.stop_line_cnt == 2:
                self.image_sub.unregister()
            right_yellow_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            hsv = cv2.cvtColor(right_yellow_image, cv2.COLOR_BGR2HSV)

            mask = cv2.inRange(hsv, self.lower_color, self.upper_color)

            h, w, d = right_yellow_image.shape
            search_top = 1 * h / 2
            search_bot = 3 * h / 4 + 20

            mask[0:h, 0:w/2] = 0
            mask[0:h/2, 0:w] = 0

            # mask[0:320, 0:w] = 0
            # mask[0:h, 0:480] = 0

            gray = cv2.cvtColor(hsv, cv2.COLOR_BGR2HSV)
            gray = cv2.GaussianBlur(gray, (7, 7), 0)
            edges = cv2.Canny(gray, 40, 120)

            M = cv2.moments(mask)
            _,contours,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) > 0:
                cnt = contours[len(contours) - 1]
                self.area = max(list(map(lambda x: cv2.contourArea(x), contours)))
                # rospy.loginfo(self.area)


                if self.area < 20000:
                    if M['m00'] > 0:
                        cx_white = int(M['m10'] / M['m00'])
                        cy_white = int(M['m01'] / M['m00'])

                        # cx = (cx_white + cx_white - 350) // 2
                        # cx = (cx_yellow + cx_white) //2
                        if self.area < 1000:
                            cx = cx_white - 235
                        else:
                            cx = cx_white - 245
                        cy = (cy_white + cy_white) // 2

                        cv2.circle(right_yellow_image, (cx_white, cy_white), 10, (0, 255, 255), -1)


                        err = cx - w / 2
                        # print('angle:', err)
                        self.robot_controller.set_velocity(self.velocity)
                        self.robot_controller.set_angle(-float(err) / 100)
                        self.robot_controller.go_forward()
                else:
                    self.robot_controller.set_velocity(self.velocity)
                    self.robot_controller.set_angle(-0.1)
                    self.robot_controller.go_forward()
            else:
                self.robot_controller.set_velocity(self.velocity)
                self.robot_controller.set_angle(-0.1)
                self.robot_controller.go_forward()
            # cv2.imshow('whiteline',mask)
            # cv2.waitKey(3)
        except:
            pass


# class RightWhiteLine:
#     def __init__(self):
#         self.robot_controller = BaseMove()
#         self.bridge = cv_bridge.CvBridge()
#         #    cv2.namedWindow("window", 1)
#         self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
#         self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
#         self.twist = Twist()
#
#     def image_callback(self, msg):
#         image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
#
#         lower_white = numpy.array([0, 0, 200])
#         upper_white = numpy.array([0, 0, 255])
#
#         wmask = cv2.inRange(hsv, lower_white, upper_white)
#         h, w, d = image.shape
#         M = cv2.moments(wmask)
#         wmask[0:150, 0:w] = 0
#         wmask[150:440, 0: w / 2] = 0
#         wmask[440:h, 0:w] = 0
#         _, qq, _ = cv2.findContours(wmask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#
#
#         if M['m00'] > 0:
#             cx = int(M['m10'] / M['m00']) - 200
#             cy = int(M['m01'] / M['m00'])
#             cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
#             # BEGIN CONTROL
#             err = cx - w / 2
#
#             self.robot_controller.set_velocity(0.5)
#             self.robot_controller.set_angle(-float(err) / 90)
#             self.robot_controller.go_forward()
#             # END CONTROL
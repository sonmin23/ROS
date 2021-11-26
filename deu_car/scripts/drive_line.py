#! /usr/bin/env python
import rospy
from drive_straight_detector import StraightDetector
from drive_course_detector import CourseDetector
from base_move import BaseMove
from nav_msgs.msg import Odometry

class LineTracer:
    def __init__(self):
        self.robot_controller = BaseMove()
        self.left_line = StraightDetector('left_camera/rgb/image_raw', 'left')
        self.right_line = StraightDetector('right_camera/rgb/image_raw', 'right')
        self.left_line2 = CourseDetector('left_camera/rgb/image_raw', 'left')
        self.right_line2 = CourseDetector('right_camera/rgb/image_raw', 'right')
        # self.odome_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.err = 0
        self.sumangle = 0
        self.check = 0
        self.angle = 0

    # def odom_callback(self, odom_data):
    #     if self.left_line.center > 40:
    #         self.angle = -0.21
    #         # self.angle = 0
    #     else:
    #         if self.left_line.center > 10:
    #             self.angle = 0
    #         else:
    #             self.angle = odom_data.pose.pose.orientation.w - 0.6
    #             self.angle = -float(self.angle)


    def start_line_trace(self, flag, speed, angle=0):
        if angle == 0:
            if flag == 'TRUE':
                if self.left_line.center == 0 and self.right_line.center == 0:
                    self.robot_controller.set_velocity(0.1)
                    self.robot_controller.go_forward()
                elif self.right_line.center >= 600:
                    self.robot_controller.set_velocity(0.2)
                    self.robot_controller.set_angle(-float(80) / 100)
                    self.robot_controller.go_forward()
                elif self.left_line.center > 110 and self.left_line.center < 160 and self.right_line.center > 100:
                    # print("zickzin")
                    self.robot_controller.set_velocity(speed)
                    self.robot_controller.set_angle(0.0)
                    self.robot_controller.go_forward()
                else:
                    # self.err = (self.right_line.center + self.left_line.center) / 2 - 310
                    self.err = (self.right_line.center + self.left_line.center) / 2 - 295
                    angle = -float(self.err) / 100
                    # print 'angle = ', angle
                    if abs(angle) < 0.4:
                        self.robot_controller.set_velocity(speed)
                    if abs(angle) >= 0.4:
                        self.robot_controller.set_velocity(0.2)
                    self.robot_controller.set_angle(angle)
                    self.robot_controller.go_forward()

############obstacle course and final course
            if flag == 'STRAIGHT2':
                if self.left_line.center == 0 and self.right_line.center == 0:
                    self.robot_controller.set_velocity(0.1)
                    self.robot_controller.go_forward()
                elif self.right_line.center >= 600:
                    self.robot_controller.set_velocity(0.2)
                    self.robot_controller.set_angle(-float(80) / 100)
                    self.robot_controller.go_forward()
                elif self.left_line.center > 110 and self.left_line.center < 160 and self.right_line.center > 100:
                    # print("zickzin")
                    self.robot_controller.set_velocity(speed)
                    self.robot_controller.set_angle(0.0)
                    self.robot_controller.go_forward()
                else:
                    self.err = (self.right_line.center + self.left_line.center) / 2 - 300
                    # self.err = (self.right_line.center + self.left_line.center) / 2 - 295
                    angle = -float(self.err) / 100
                    # print 'angle = ', angle
                    if abs(angle) < 0.4:
                        self.robot_controller.set_velocity(speed)
                    if abs(angle) >= 0.4:
                        self.robot_controller.set_velocity(0.2)
                    self.robot_controller.set_angle(angle)
                    self.robot_controller.go_forward()

            if flag == 'COURSE':
                if self.left_line2.center == 0 and self.right_line2.contour == 0:
                    self.robot_controller.set_velocity(0.3)
                    #  if abs(angle) >= 0.5:
                    # self.robot_controller.set_velocity(0.35)
                    self.robot_controller.go_forward()
                elif self.left_line2.contour - self.right_line2.contour > 30:
                    self.robot_controller.set_velocity(0.57)
                    # self.err = (self.right_line.center)/2 - 320
                    self.robot_controller.set_angle(-float(50) / 100)
                    self.robot_controller.go_forward()

                elif self.right_line2.contour - self.left_line2.contour > 30:
                    self.robot_controller.set_velocity(0.57)
                    # self.err = (self.right_line.center)/2 - 320
                    self.robot_controller.set_angle(float(50) / 100)
                    self.robot_controller.go_forward()
                 # self.robot_controller.set_velocity(0.2)
                 # self.robot_controller.set_angle(float(80) / 100)
                 # self.robot_controller.go_forward()
                else:
                    self.err = (self.right_line2.center + self.left_line2.center) / 2 - 305
                    # self.err = (self.right_line2.center + self.left_line2.center) / 2 - 305
                    # print self.err
                    angle = -float(self.err) / 100
                    # print 'angle = ', angle
                    if abs(angle) < 0.4:
                        self.robot_controller.set_velocity(0.5)
                    if abs(angle) >= 0.4:
                        self.robot_controller.set_velocity(0.2)
                    self.robot_controller.set_angle(angle)
                    self.robot_controller.go_forward()


            if flag == 'SECONDCOURSE':
                if self.left_line2.center == 0 and self.right_line2.contour == 0:
                    self.robot_controller.set_velocity(0.2)
                    #  if abs(angle) >= 0.5:
                    # self.robot_controller.set_velocity(0.35)
                    self.robot_controller.go_forward()
                elif self.left_line2.contour - self.right_line2.contour > 20:
                    self.robot_controller.set_velocity(speed)
                    # self.err = (self.right_line.center)/2 - 320
                    self.robot_controller.set_angle(-float(50) / 100)

                    self.robot_controller.go_forward()

                elif self.right_line2.contour - self.left_line2.contour > 20:
                    self.robot_controller.set_velocity(speed)
                    # self.err = (self.right_line.center)/2 - 320
                    self.robot_controller.go_forward()
                # self.robot_controller.set_velocity(0.2)
                # self.robot_controller.set_angle(float(80) / 100)
                # self.robot_controller.go_forward()

                else:
                    self.err = (self.right_line2.center + self.left_line2.center) / 2 - 310
                    # print self.err
                    angle = -float(self.err) / 95
                    # print 'angle = ', angle
                    if abs(angle) < 0.4:
                        self.robot_controller.set_velocity(speed)
                    if abs(angle) >= 0.4:
                        self.robot_controller.set_velocity(0.2)
                    self.robot_controller.set_angle(angle)
                    self.robot_controller.go_forward()

            if flag == 'WHITELINE':
                if self.left_line.center == 0 and self.right_line.contour == 0:
                    self.robot_controller.set_velocity(0.2)
                    #  if abs(angle) >= 0.5:
                    # self.robot_controller.set_velocity(0.35)
                    self.robot_controller.go_forward()
                else:
                    self.err = self.right_line.center - 320
                    angle = -float(self.err) / 100
                    if abs(angle) < 0.4:
                        self.robot_controller.set_velocity(speed)
                    if abs(angle) >= 0.4:
                        self.robot_controller.set_velocity(0.2)
                    self.robot_controller.set_angle(angle)
                    self.robot_controller.go_forward()

            if flag == 'CONTROL':

                # print(self.err)
                self.robot_controller.go_forward()


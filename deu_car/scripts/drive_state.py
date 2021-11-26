#! /usr/bin/env python

import rospy
import time

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from smach import State
from drive_line import LineTracer
from parallel_parking_right_line import RightLineParallelParking
from detect_parking import DetectParking
from scourse_right_line import DetectRightLineScource
from final_left_line import LeftLineFinal
from final_right_line import RightLineFinal
from base_move import BaseMove
from scourse_left_line import DetectLeftLineScourse
from Tparking_right_line import DetectRightLineTparking

blocking_toggle = False
stop_line_toggle = False
test = False
driving_bot = None
obstacle_toggle = None
stop_sign_toggle = None

#blocking_toggle sub callback
def blocking_bar_ch(msg):
    global blocking_toggle
    if msg.data == 'ON':
        blocking_toggle = True
    if msg.data == 'OFF':
        blocking_toggle = False

#stop_line sub callback
def stop_line_ch(msg):
    global stop_line_toggle
    if msg.data == 'STOP_LINE':
        stop_line_toggle = True
    if msg.data == 'NO_STOP_LINE':
        stop_line_toggle = False

#object sub callback
def obstacle_ch(msg):
    global obstacle_toggle
    if msg.data == 'DETECT_OBSTACLE':
        obstacle_toggle = True
    if msg.data == 'NO_OBSTACLE':
        obstacle_toggle = False

#stop_sign sub callback
def stop_sign_ch(msg):
    global stop_sign_toggle
    if msg.data == 'STOP_SIGN':
        stop_sign_toggle = True
    if msg.data == 'NO_STOP_SIGN':
        stop_sign_toggle = False

#Move for a second at the start
class StartLine(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.twist = Twist()
        self.change_time = time.time()

    def execute(self, userdata):
        while True:
            self.twist.linear.x = 1.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)

            if self.change_time + 2.5 < time.time():
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.cmd_vel_pub.publish(self.twist)
                return 'success'

#blocking_toggle sub
blocking_toggle_sub = rospy.Subscriber('blocking_bar', String, blocking_bar_ch)

#blockingbar drive
class BlockingBar(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        global blocking_toggle
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.twist = Twist()
        self.change_time = time.time()

    def execute(self, userdata):
        rate = rospy.Rate(20)
        while True:
          if blocking_toggle == True:
             rospy.loginfo("blocking bar open")
             self.twist.linear.x = 0.9
             self.twist.angular.z = 0.0
             self.cmd_vel_pub.publish(self.twist)
             if self.change_time + 2.6 < time.time():
                 return 'success'
          rate.sleep()

#stop_line_toggle sub
stop_line_sub = rospy.Subscriber('stop_line', String, stop_line_ch)

#object_toggle sub
object_sub = rospy.Subscriber('obstacle', String, obstacle_ch)

#stop_sign_toggle sub
stop_sign_sub = rospy.Subscriber('stop_sign', String, stop_sign_ch)

#drive bot
class DriveLine(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        global stop_line_toggle
        self.stop_line_check = 0
        self.change_time = time.time()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.twist = Twist()
        self.time = rospy.Time.now()
        self.check = 0

    def execute(self, userdata):
        global driving_bot
        driving_bot = LineTracer()
        rate = rospy.Rate(20)
        while True:
            if stop_line_toggle == True:
                if self.time + rospy.Duration(5) < rospy.Time.now():
                    rospy.loginfo('DETECTOR STOP LINE')
                    self.stop_line_check = self.stop_line_check + 1
                    if self.check == 1:
                        self.check = 0
                    self.check = 1
                    rospy.sleep(3)
                    self.time = rospy.Time.now()
            if self.stop_line_check == 2:
                rospy.loginfo('FIRST COURSE')
                return 'success'
            else:
                driving_bot.start_line_trace('TRUE', 0.57)
            rate.sleep()


#first course
class RefractionCourse(State):
    def __init__(self):
        global stop_line_toggle
        State.__init__(self, outcomes=['success'])
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.twist = Twist()
        self.stop_line_check = 0
        self.check = 0
        self.time = rospy.Time.now()

    def execute(self, userdata):
        global driving_bot
        rate = rospy.Rate(20)
        while True:
            if stop_line_toggle == True:
                if self.time + rospy.Duration(7) < rospy.Time.now():
                    rospy.loginfo('DETECTOR STOP LINE')
                    self.stop_line_check = self.stop_line_check + 1
                    if self.check == 1:
                        self.check = 0
                    self.check = 1
                    rospy.sleep(3)
                    self.time = rospy.Time.now()
            if self.stop_line_check == 1:
                driving_bot.start_line_trace('COURSE', 0.57)
            elif self.stop_line_check == 2:
                rospy.loginfo('ENTERING THE INTERSECTION')
                return 'success'
            else:
                driving_bot.start_line_trace('COURSE', 0.57)
            rate.sleep()

#Obstacle course code
class Obstacle(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        global obstacle_toggle
        global stop_sign_toggle
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.twist = Twist()
        self.change_time = time.time()
        self.check = 0
        self.stop_sign = False
        self.state = 0

    def execute(self, userdata):
        bot = DetectParking('right')
        global driving_bot
        rate = rospy.Rate(20)
        #########################################
        # test code
        # driving_bot = LineTracer()
        # stop_sign = DetectStopSign()
        ############################################
        while True:
            if stop_sign_toggle == True and self.state == 0:
                self.stop_sign = True
            if self.stop_sign == True:
                if self.state == 0:
                    # rospy.loginfo('DETECTOR STOP SIGN')
                    self.change_time = time.time()
                    self.state += 1
                elif self.state == 1:
                    if self.change_time + 0.6 < time.time():
                        # rospy.loginfo('DETECTOR STOP SIGN')
                        self.change_time = time.time() - 20
                        self.state += 1
                elif self.state == 2:
                    if self.change_time + 10 < time.time():
                        rospy.loginfo('DETECTOR STOP SIGN')
                        # rospy.loginfo('DETECTOR STOP SIGN')
                        rospy.sleep(3)
                        self.stop_sign = False
                        self.change_time = time.time()

            if obstacle_toggle == True:
               rospy.loginfo("obstacle")
               self.twist.linear.x = 0.0
               self.twist.angular.z = 0.0
               self.cmd_vel_pub.publish(self.twist)
            elif self.check == 0 and bot.cnt >= 39:
               self.change_time = time.time()
               self.check = 1
            if self.check == 1 and self.change_time + 0.5 < time.time():
                rospy.loginfo("parking")
                return 'success'
            elif self.check == 1 and self.change_time + 0.5 > time.time():
                self.twist.linear.x = 0.8
                self.twist.angular.z = -0.2
                self.cmd_vel_pub.publish(self.twist)
            else:
                driving_bot.start_line_trace('STRAIGHT2', 0.7)
            rate.sleep()

##ParallelParking code
class ParallelParking(State):
    def __init__(self):
        global stop_line_toggle
        State.__init__(self, outcomes=['success'])
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.twist = Twist()
        self.stop_line_check = 0
        self.state = 0
        self.check = 0
        self.course = 0
        self.checkp = 0
        self.stop_sign = False
        self.change_time = time.time()
        self.time = rospy.Time.now()

    def execute(self, userdata):
        global driving_bot
        bot = None
        gogo = None
        rate = rospy.Rate(20)
        self.twist.angular.z = 0.1
        self.cmd_vel_pub.publish(self.twist)
        self.time = rospy.Time.now()
        driving = RightLineParallelParking()
        driving.areaindex = 5000
        driving.time = rospy.Time.now()
        # driving.status = 4
        while True:

            if driving.parking_check == True:
                if driving.status == 0 and self.time + rospy.Duration(3) < rospy.Time.now():
                    rospy.sleep(3)
                    self.time = rospy.Time.now()
                    driving.status = 1
            if driving.status == 1:
                if self.time + rospy.Duration(4) > rospy.Time.now():
                    self.twist.angular.z = 0.85
                    self.twist.linear.x = 0.2
                    self.cmd_vel_pub.publish(self.twist)
                else:
                    self.time = rospy.Time.now()
                    driving.status = 2
            elif driving.status == 2:
                if self.time + rospy.Duration(1) > rospy.Time.now():
                    self.twist.angular.z = -0.9
                    self.twist.linear.x = 1.0
                    self.cmd_vel_pub.publish(self.twist)
                else:
                    self.time = rospy.Time.now()
                    driving.status = 3
            elif driving.status == 3:
                if self.time + rospy.Duration(1) > rospy.Time.now():
                    self.twist.angular.z = 0.0
                    self.twist.linear.x = 0.2
                    self.cmd_vel_pub.publish(self.twist)
                else:
                    driving.status = 4
            elif driving.status == 4:
                driving_bot.start_line_trace('STRAIGHT2', 0.3)
                if stop_line_toggle == True:
                    if self.time + rospy.Duration(5) < rospy.Time.now():
                        rospy.loginfo('DETECTOR STOP LINE')
                        self.stop_line_check = self.stop_line_check + 1
                        if self.check == 1:
                            self.check = 0
                        self.check = 1
                        rospy.sleep(3)
                        self.time = rospy.Time.now()
                if self.stop_line_check == 1:
                    self.time = rospy.Time.now()
                    driving.status = 5
            elif driving.status == 5:
                if self.time + rospy.Duration(2) > rospy.Time.now():
                    self.twist.angular.z = 0.0
                    self.twist.linear.x = 0.5
                    self.cmd_vel_pub.publish(self.twist)
                else:
                    self.time = rospy.Time.now()
                    bot = DetectParking('left')
                    leftline_bot = LeftLineFinal()
                    leftline_bot.state = 0
                    driving.status = 6
            elif driving.status == 6:
                if self.time + rospy.Duration(18) < rospy.Time.now():
                    leftline_bot.state = 1
                    driving.status = 7
            if driving.status == 7:
                bot = RightLineFinal()
                bot.stop_line_cnt = 0
                driving.status = 8
            if stop_sign_toggle == True and self.state == 0:
                self.state = 1
                self.change_time = time.time()
            if self.state == 1:
                if self.change_time + 1.0 < time.time():
                    rospy.loginfo('DETECTOR STOP SIGN')
                    # rospy.loginfo('DETECTOR STOP SIGN')
                    bot.stop_sign_check = True
                    return 'success'

            rate.sleep()

# lane 2 scourse
# class Scourse(State):
#     def __init__(self):
#         global stop_line_toggle
#         State.__init__(self, outcomes=['success'])
#         self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
#         self.twist = Twist()
#         self.stop_line_check = 0
#         self.check = 0
#         self.course = 0
#         self.checkp = 0
#         self.time = rospy.Time.now()
#
#     def execute(self, userdata):
#         rate = rospy.Rate(20)
#         self.twist.angular.z = 0.1
#         self.cmd_vel_pub.publish(self.twist)
#         driving = RightWhiteLine()
#         driving.areaindex = 20000
#         while True:
#             if stop_line_toggle == True:
#                 if self.time + rospy.Duration(5) < rospy.Time.now():
#                     # rospy.loginfo('DETECTOR STOP LINE')
#                     self.stop_line_check = self.stop_line_check + 1
#                     # rospy.sleep(3)
#                     self.time = rospy.Time.now()
#             if self.stop_line_check == 2:
#                 rospy.loginfo('ENTERING THE INTERSECTION')
#                 return 'success'
#             if test == True:
#                 return 'success'
#             rate.sleep()

#lane 1 scourse
class Scourse(State):
    def __init__(self):
        global stop_line_toggle
        State.__init__(self, outcomes=['success'])
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.twist = Twist()
        self.stop_line_check = 0
        self.check = 0
        self.time = time.time()
        self.state = 0

    def execute(self, userdata):
        #########
        # driving_bot = LineTracer()
        scourse = DetectRightLineScource()
        scourse.stop_line_cnt = 0
        scourse.state = 0
        ########
        # scourse.stop_line_cnt = 0
        # scourse.state = 0
        rate = rospy.Rate(20)

        while True:
            if self.state == 0 and scourse.stop_line_cnt == 3:
                self.time = time.time()
                self.state += 1
            elif self.state == 1:
                if self.time + 3.0 > time.time():
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(self.twist)
                else:
                    self.time = time.time()
                    self.state += 1
            elif self.state == 2:
                if self.time + 4.5 > time.time():
                    self.twist.linear.x = 0.9
                    self.twist.angular.z = -0.27
                    self.cmd_vel_pub.publish(self.twist)
                else:
                    self.time = time.time()
                    self.state += 1
                    scourse = DetectRightLineScource()
                    scourse.state = 0
                    scourse.stop_line_cnt = 2
            elif self.state == 3:
                if scourse.stop_line_cnt == 3:
                    return 'success'
            rate.sleep()

class EnterTcourse(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        # self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.twist = Twist()
        self.change_time = time.time()
        self.move = BaseMove()
        self.flag = 0
        # self.towardCourse2 = TowardCourse2()

    def execute(self, userdata):
        self.change_time = time.time()

        while True:

            if self.flag == 0:

                if self.change_time + 1.0 > time.time():
                    self.move.set_velocity(0)
                    self.move.set_angle(0.25)
                    # rospy.loginfo('turn!')
                    self.move.go_forward()

                else:
                    self.change_time = time.time()
                    self.flag += 1

            elif self.flag == 1:
                self.move.set_velocity(0.9)
                self.move.set_angle(0)
                self.move.go_forward()

                if self.change_time + 5.3 < time.time():
                    self.move.set_velocity(0)
                    self.move.set_angle(0)
                    self.move.go_forward()
                    return 'success'


class EnterTparking(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        self.time = rospy.Time.now()
        self.flag = 0

    def execute(self, userdata):
        detectRight = DetectRightLineTparking()
        detectRight.time = rospy.Time.now()
        rate = rospy.Rate(20)

        while True:

            if detectRight.state == 1:
                return 'success'
            self.time = rospy.Time.now()
            # print(">>>>>>>>>>>>>>>>>:", detectRight.state)
            rate.sleep()

class OutTparkingCourse(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        self.time = rospy.Time.now()
        self.move = BaseMove()
        self.flag = 0

    def execute(self, userdata):
        detectleft = DetectLeftLineScourse()
        detectleft.state = 0
        detectleft.state = 0
        rate = rospy.Rate(20)
        self.change_time = time.time()

        while True:

            if self.flag == 0:
                self.move.set_velocity(0.3)
                self.move.set_angle(-0.9)
                self.move.go_forward()

                if self.change_time + 0.9 < time.time():
                    self.move.set_velocity(0)
                    self.move.set_angle(0)
                    self.move.go_forward()
                    self.change_time = time.time()
                    self.flag += 1
            #     else:
            #         detectright.state = 1
            if detectleft.state == 1:
                rospy.loginfo('DETECTOR STOP SIGN')
                return 'success'
            self.time = rospy.Time.now()
            # print(">>>>>>>>>>>>>>>>>:", detectRight.state)
            rate.sleep()

class StartTparking(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        # self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.twist = Twist()
        self.change_time = time.time()
        self.move = BaseMove()
        self.flag = 0
        # self.towardCourse2 = TowardCourse2()

    def execute(self, userdata):
        self.change_time = time.time()

        while True:

            if self.flag == 0:
                self.move.set_velocity(1.0)
                self.move.set_angle(0.27)
                self.move.go_forward()

                if self.change_time + 4.5 < time.time():
                    self.move.set_velocity(0)
                    self.move.set_angle(0)
                    self.move.go_forward()
                    self.change_time = time.time()
                    self.flag += 1

            elif self.flag == 1:

                if self.change_time + 3.0 > time.time():
                    self.move.set_velocity(0)
                    self.move.set_angle(-1.06)
                    # rospy.loginfo('turn!')
                    self.move.go_forward()

                else:
                    self.change_time = time.time()
                    self.flag += 1

            elif self.flag == 2:

                if self.change_time + 4.5 > time.time():
                    self.move.set_velocity(0.9)
                    self.move.set_angle(0.69)
                    self.move.go_forward()

                else:
                    self.flag += 1
                    self.change_time = time.time()

            elif self.flag == 3:

                if self.change_time + 1.0 > time.time():
                    self.move.set_velocity(0)
                    self.move.set_angle(-1.8)
                    self.move.go_forward()

                else:
                    self.change_time = time.time()
                    self.flag += 1

            elif self.flag == 4:

                if self.change_time + 1.9 > time.time():
                    self.move.set_velocity(0.8)
                    self.move.set_angle(-0.6)
                    self.move.go_forward()

                else:
                    self.flag += 1
                    self.change_time = time.time()
                    return 'success'


#stopline
class ProjectEnd(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        return 'success'

class Intersection3(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        # self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.twist = Twist()
        self.change_time = time.time()
        self.move = BaseMove()
        self.flag = 0
        # self.towardCourse2 = TowardCourse2()

    def execute(self, userdata):
        self.change_time = time.time()

        while True:

            if self.flag == 0:
                if self.change_time + 3.0 > time.time():
                    self.move.set_velocity(0.0)
                    self.move.set_angle(0.0)
                    # rospy.loginfo('turn!')
                    self.move.go_forward()
                else:
                    self.flag += 1
                    self.change_time = time.time()
            elif self.flag == 1:
                if self.change_time + 2.5 > time.time():
                    self.move.set_velocity(1.0)
                    self.move.set_angle(0.5)
                    # rospy.loginfo('turn!')
                    self.move.go_forward()

                else:
                    self.change_time = time.time()
                    self.flag += 1
                    return 'success'

# Course 1 clear Course 2 in
class Intersection(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        # self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.twist = Twist()
        self.change_time = time.time()
        self.move = BaseMove()
        self.flag = 0
        # self.towardCourse2 = TowardCourse2()

    def execute(self, userdata):
        self.change_time = time.time()

        while True:

    # see left
            if self.flag == 0:
                if self.change_time + 1.0 > time.time():
                    self.move.set_velocity(0)
                    self.move.set_angle(0.0)
                    # rospy.loginfo('turn!')
                    self.move.go_forward()

                else:
                    self.change_time = time.time()
                    self.flag += 1

            elif self.flag == 1:
                self.move.set_velocity(1.0)
                self.move.set_angle(0.0) # coreect -0.1
                self.move.go_forward()

                if self.change_time + 5.0 < time.time():
                    self.move.set_velocity(0)
                    self.move.set_angle(0.4)
                    self.move.go_forward()
                    return 'success'



class T_Parking(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        # self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.twist = Twist()
        self.change_time = time.time()
        self.move = BaseMove()
        self.flag = 0
        # self.towardCourse2 = TowardCourse2()

    def execute(self, userdata):
        self.change_time = time.time()

        while True:

            if self.flag == 0:
                self.move.set_velocity(1.0)
                self.move.set_angle(0.31)
                self.move.go_forward()

                if self.change_time + 4.5 < time.time():
                    self.move.set_velocity(0)
                    self.move.set_angle(0)
                    self.move.go_forward()
                    self.change_time = time.time()
                    self.flag += 1

            elif self.flag == 1:

                if self.change_time + 1.0 > time.time():
                    self.move.set_velocity(1.0)
                    self.move.set_angle(0.0)
                    # rospy.loginfo('turn!')
                    self.move.go_forward()

                else:
                    self.change_time = time.time()
                    self.flag += 1

            elif self.flag == 2:

                if self.change_time + 3.0 > time.time():
                    self.move.set_velocity(0)
                    self.move.set_angle(-1.12)
                    # rospy.loginfo('turn!')
                    self.move.go_forward()

                else:
                    self.change_time = time.time()
                    self.flag += 1

            elif self.flag == 3:

                if self.change_time + 1.0 > time.time():
                    self.move.set_velocity(1.0)
                    self.move.set_angle(0.0)
                    # rospy.loginfo('turn!')
                    self.move.go_forward()

                else:
                    self.change_time = time.time()
                    self.flag += 1

            elif self.flag == 4:

                if self.change_time + 4.5 > time.time():
                    self.move.set_velocity(0.9)
                    self.move.set_angle(0.68)
                    self.move.go_forward()

                else:
                    self.flag += 1
                    self.change_time = time.time()

            elif self.flag == 5:

                if self.change_time + 1.0 > time.time():
                    self.move.set_velocity(0)
                    self.move.set_angle(-1.2)
                    self.move.go_forward()

                else:
                    self.change_time = time.time()
                    self.flag += 1

            elif self.flag == 6:

                if self.change_time + 0.5> time.time():
                    self.move.set_velocity(1.0)
                    self.move.set_angle(-0.6)
                    self.move.go_forward()

                else:
                    self.flag += 1
                    self.change_time = time.time()
                    return 'success'
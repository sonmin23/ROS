#! /usr/bin/env python

import rospy
import time

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from smach import State
from right_yellow_line import Right_Yellow_Line

blocking_toggle = False
stop_line_toggle = False
test = False

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

#Move for a second at the start
class StartLine(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.twist = Twist()
        self.change_time = time.time()

    def execute(self, userdata):
        while True:
            self.twist.linear.x = 0.9
            self.cmd_vel_pub.publish(self.twist)

            if self.change_time + 1.3 < time.time():
                self.twist.linear.x = 0.0
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

    def execute(self, userdata):
        rate = rospy.Rate(20)
        while True:
          if blocking_toggle == True:
             rospy.loginfo("blocking bar open")
             return 'success'
          rate.sleep()

#stop_line_toggle sub
sto_line_sub = rospy.Subscriber('stop_line', String, blocking_bar_ch)

#drive bot
class DriveLine(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        global stop_line_toggle
        self.change_time = time.time()
        # self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        # self.twist = Twist()

    def execute(self, userdata):
        rate = rospy.Rate(20)
        driving_bot = Right_Yellow_Line()
        while True:
            if test == True:
                return 'success'
            rospy.loginfo("drive")
            if stop_line_toggle == True:
                rospy.loginfo("Stop_Line_Detector")
                if self.change_time + 5 < time.time():
                    self.twist.linear.x = 0.0
                    self.cmd_vel_pub.publish(self.twist)
            rate.sleep()

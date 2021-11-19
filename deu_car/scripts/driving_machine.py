#! /usr/bin/env python

import smach_ros
import rospy
from smach import StateMachine
from drive_state import StartLine, BlockingBar, DriveLine

class AutonomousDriving:

    def __init__(self):
        self.driving_machine = StateMachine(outcomes=['success'])
    def autonomos_drive(self):
        rospy.init_node('drive')
        with self.driving_machine:
            StateMachine.add('START', StartLine(), transitions={'success': 'BLOCKINGBAR'})
            StateMachine.add('BLOCKINGBAR', BlockingBar(), transitions={'success': 'DRIVELINE'})
            StateMachine.add('DRIVELINE', DriveLine(), transitions={'success': 'success'})

        self.driving_machine.execute()
        rospy.spin()

if __name__ == '__main__':
    drive = AutonomousDriving()
    drive.autonomos_drive()
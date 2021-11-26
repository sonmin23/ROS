#! /usr/bin/env python

import smach_ros
import rospy
from smach import StateMachine
from drive_state import StartLine, BlockingBar, DriveLine, RefractionCourse, Intersection, ProjectEnd, Scourse, Obstacle, ParallelParking, EnterTcourse, EnterTparking, StartTparking, OutTparkingCourse,Intersection3,T_Parking

class AutonomousDriving:
    def __init__(self):
        self.driving_machine = StateMachine(outcomes=['success'])
    def autonomos_drive(self):
        rospy.init_node('drive')
        with self.driving_machine:
            StateMachine.add('START', StartLine(), transitions={'success': 'BLOCKINGBAR'}) # END CODE
            StateMachine.add('BLOCKINGBAR', BlockingBar(), transitions={'success': 'DRIVELINE'}) # END CODE
            StateMachine.add('DRIVELINE', DriveLine(), transitions={'success': 'REFRACTIONCOURSE'}) # END CODE
            StateMachine.add('REFRACTIONCOURSE', RefractionCourse(), transitions={'success': 'INTERSECTION'}) #END CODE
            StateMachine.add('INTERSECTION', Intersection(), transitions={'success': 'SCOURSE'}) # END CODE
            StateMachine.add('SCOURSE', Scourse(), transitions={'success': 'INTERSECTION_2'}) # END CODE
            StateMachine.add('INTERSECTION_2', EnterTcourse(), transitions={'success': 'ENTER-TPARKING'}) # END CODE
            StateMachine.add('ENTER-TPARKING', EnterTparking(), transitions={'success': 'START-TPARKING'}) # END CODE
            StateMachine.add('START-TPARKING', T_Parking(), transitions={'success': 'OUT-TPARKINGCOURSE'}) # AZIC
            StateMachine.add('OUT-TPARKINGCOURSE', OutTparkingCourse(), transitions={'success': 'INTERSECTION_3'}) # AZIC
            StateMachine.add('INTERSECTION_3', Intersection3(), transitions={'success': 'OBSTACLE'}) # AZIC
            StateMachine.add('OBSTACLE', Obstacle(), transitions={'success': 'PARALLEPARKING'})
            StateMachine.add('PARALLEPARKING', ParallelParking(), transitions={'success': 'PROJECTEND'})
            StateMachine.add('PROJECTEND', ProjectEnd(), transitions={'success': 'success'})

        self.driving_machine.execute()
        rospy.spin()

if __name__ == '__main__':
    drive = AutonomousDriving()
    drive.autonomos_drive()
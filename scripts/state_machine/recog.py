#!/usr/bin/env python3
# coding: UTF-8

import os
import csv
import time
import rospy
import smach
from moveit_commander import RobotCommander, MoveGroupCommander


class RecogTPipe(smach.State):
    # Initialize any variables or resources here
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes)
        # rospy.init_node("xArm6")
        self.robot = RobotCommander()
        self.xarm = MoveGroupCommander("xarm6")

    def execute(self, userdata):
        # recognize TPipe
        

        return 'success'
  

# For debug
if __name__ == '__main__':
    rospy.init_node("xArm6")
    sm = smach.StateMachine(outcomes=['success', 'failure'])
    with sm:
        smach.StateMachine.add('RecogTPipe', RecogTPipe(['success', 'failure']), transitions={'success':'success', 'failure':'failure'})
    outcome = sm.execute()
    print(outcome)
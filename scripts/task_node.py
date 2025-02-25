#!/usr/bin/env python3
# coding: UTF-8

import rospy
import smach
import smach_ros
from state_machine import (
    standard,
    # move,
    recog,
    pick_work,
    place_work,
    update_pathseed,
)


class PathseedsEncoderStateMachine:
    def __init__(self):

        # Initialize any necessary ROS publishers, subscribers, and services

        # Set the initial state of the state machine
        self.state_machine = smach.StateMachine(outcomes=['exit'])

        # Create and add states to the state machine
        with self.state_machine:
            # Add states to the state machine
            smach.StateMachine.add(
                'InitialSettings', 
                standard.InitialSettings(['success', 'failure', 'loop']),
                transitions={'success': 'START', 'failure': 'exit', 'loop': 'InitialSettings'}
            )
            smach.StateMachine.add(
                'START',
                standard.Start(['success', 'failure', 'loop']),
                transitions={'success': 'PICK_WORK', 'failure': 'exit', 'loop': 'START'}
            )
            # smach.StateMachine.add(
            #     'RecogTPipe',
            #     recog.RecogTPipe(['success', 'failure', 'loop']),
            #     transitions={'success': 'PICK_WORK', 'failure': 'exit', 'loop': 'RecogTPipe'}
            # )
            smach.StateMachine.add(
                'PICK_WORK', 
                pick_work.PickWork(['success', 'failure', 'loop']),
                transitions={'success': 'PICK_BACK', 'failure': 'exit', 'loop': 'PICK_WORK'}
            )
            smach.StateMachine.add(
                'PICK_BACK',
                pick_work.PICK_BACK(['success', 'failure', 'loop']),
                transitions={'success': 'PLACE_WORK', 'loop': 'PICK_BACK', 'failure': 'exit'}
            )
            
            # smach.StateMachine.add(
            #     'PLACE_WORK', 
            #     place_work.PlaceWork(['finish', 'failure', 'loop', 'update']),
            #     transitions={'finish': 'InitialSettings', 'failure': 'exit', 'loop': 'PLACE_WORK', 'update': 'UPDATE_PATHSEED'}
            # )
            smach.StateMachine.add(
                'PLACE_WORK', 
                place_work.PlaceWork(['success', 'failure', 'loop']),
                transitions={'success': 'PLACE_BACK', 'failure': 'exit', 'loop': 'PLACE_WORK'}
            )
            smach.StateMachine.add(
                'PLACE_BACK',
                place_work.PLACE_BACK(['success', 'failure', 'loop']),
                transitions={'success': 'PICK_WORK', 'loop': 'PLACE_BACK', 'failure': 'exit'}
            )
            # smach.StateMachine.add(
            #     'UPDATE_PATHSEED',
            #     update_pathseed.UpdatePathSeed(['success', 'failure']),
            #     transitions={'success': 'START', 'failure': 'exit'}
            # )
            smach.StateMachine.add(
                'exit',
                standard.Exit(['success', 'failure']),
                transitions={'success': 'START', 'failure': 'failure'}
            )
            smach.StateMachine.add(
                'failure',
                standard.Failure(['success', 'failure']),
                transitions={'success': 'START', 'failure': 'exit'}
            )

        # Create and start the introspection server
        self.sis = smach_ros.IntrospectionServer(
            'pathseeds_encode_state_machine', self.state_machine, '/SM_ROOT'
        )
        self.sis.start()

    def __del__(self):
        # Stop the introspection server
        self.sis.stop()

    def run(self):
        self.state_machine.execute()

def main():
    rospy.init_node('pathseeds_encode')
    
    # Create an instance of the PathseedsEncoderStateMachine class
    state_machine = PathseedsEncoderStateMachine()
    rospy.on_shutdown(state_machine.__del__)
    try:
        # Run the state machine
        state_machine.run()
    except Exception as e:
        rospy.logerr(e)


if __name__ == '__main__':
    main()
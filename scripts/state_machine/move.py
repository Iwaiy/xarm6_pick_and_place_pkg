#!/usr/bin/env python3
# coding: UTF-8

import yaml
import os
import csv
import rospy
import rospkg
import smach
import moveit_commander
from moveit_commander import RobotCommander, MoveGroupCommander
 
rospack = rospkg.RosPack()
package_path = rospack.get_path('xarm6_pick_and_place_pkg')  # パッケージ名を指定

class MoveToAccessPoint1(smach.State):
    def __init__(self, outcomes):
        # Declare input_keys and output_keys
        smach.State.__init__(self, outcomes=outcomes, input_keys=['start_time', 'plan_time', 'plan_size'], output_keys=['start_time', 'plan_time', 'plan_size'])
        self.robot = RobotCommander()
        self.xarm = MoveGroupCommander("xarm6")

        # self.goal_joint_angles = rospy.get_param("~Joint")

    def execute(self, userdata):
        # 計測開始
        userdata.start_time = rospy.Time.now()
        userdata.plan_time = 0
        userdata.plan_size = 0
        rospy.loginfo("Measurement started in Start state.")
        rospy.loginfo('Starting...')

        start_joint_angles = controller.current_joint_angles

        # スタート位置の設定(関節角度で指定)
        fixed_joint_values = self.goal_joint_angles["AccessPoint1"]
        print(fixed_joint_values)
        self.xarm.set_start_state_to_current_state()
        self.xarm.set_joint_value_target(fixed_joint_values)

        success = controller.execute(fixed_joint_values)

        if success is True:
            rospy.loginfo('Planning succeeded, executing plan')
            return 'success'
        else:
            print("Planning failed.")
            return 'failure'
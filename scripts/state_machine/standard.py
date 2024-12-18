#!/usr/bin/env python3
# coding: UTF-8

import os
import csv
import time
import rospy
import smach
from moveit_commander import RobotCommander, MoveGroupCommander
import geometry_msgs.msg
from node.grasp_control import GraspControl


class Wait4Start(smach.State):
    # Initialize any variables or resources here
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes)
        # self.gripper = GraspControl()

    def execute(self, userdata):
        # Implement the logic for the Wait4Start state here
        # This method will be called when the state is active
        input("Start>>>")
        # self.gripper.open()
        return 'success'
    

class Start(smach.State):
    def __init__(self, outcomes):
        # Declare input_keys and output_keys
        smach.State.__init__(self, outcomes=outcomes)

        self.robot = RobotCommander()
        self.xarm = MoveGroupCommander("xarm6")
        self.gripper = GraspControl()

    def execute(self, userdata):
        try:
            # self.xarm.set_max_velocity_scaling_factor(0.1)  # 10% の速度
            # self.xarm.set_max_acceleration_scaling_factor(0.1)  # 10% の加速度
            self.xarm.stop()
            # グリッパーを開く
            if not self.gripper.open():
                return "loop"

            # ゴールの設定(関節角度で指定)0.01342425 -0.8442685  -0.29798153  0.03872918  1.15796757  0.03068345
            #fixed_joint_values = [0.0027496605180203915, 0.104049913585186, -1.1940333843231201, 0.027469761669635773, 1.089946985244751, 0.008956530131399632]
            fixed_joint_values = [0.002444781828671694, 0.07742192596197128, -1.2735952138900757, 0.02975347451865673, 1.1962307691574097, 0.010912355966866016]

            # fixed_joint_values = [0.01342425, -0.6442685, -0.29798153, 0.03872918, 1.00, 0.03068345]

            # 現在のジョイント値（スタート状態）を取得して表示
            current_joint_values = self.xarm.get_current_joint_values()
            print(f"Current joint values (Start): {current_joint_values}")

            # ゴール状態（目標ジョイント値）を表示
            print(f"Target joint values (Goal): {fixed_joint_values}")

            # スタート状態を現在の状態に設定
            self.xarm.set_start_state_to_current_state()
            self.xarm.set_start_state_to_current_state()
            

            # ゴール状態を設定
            self.xarm.set_joint_value_target(fixed_joint_values)

            # プランニング
            success, plan, _, _ = self.xarm.plan()
            if not success:
                print("Planning failed.")
                return "loop"

            print("Planning succeeded. Executing plan...")
            success_exec = self.xarm.execute(plan)
            if success_exec:
                print("Plan executed successfully.")

                # アームの動作が完了した後、グリッパーを閉じる
                if not self.gripper.close():
                    return "loop"

                rospy.loginfo("Picking work Successfully")
                return "success"
            else:
                print("Execution failed.")
                return "loop"
        except Exception as e:
            print(f"Error in execute: {e}")
            return "loop"
    

class Exit(smach.State):
    def __init__(self, outcomes):
        # Declare input_keys and output_keys
        smach.State.__init__(self, outcomes=outcomes, input_keys=['start_time', 'plan_time'])
        #self.robot = RobotCommander()
        self.xarm = MoveGroupCommander("xarm6")
        # self.goal_joint_angles = rospy.get_param("~Joint")

    def execute(self, userdata):
        # log error message
        #rospy.logerr("Task failed.")
        return 'failure'
        

class Failure(smach.State):
    def __init__(self, outcomes):
        # Declare input_keys and output_keys
        smach.State.__init__(self, outcomes=outcomes, input_keys=['start_time', 'plan_time'])
        #self.robot = RobotCommander()
        self.xarm = MoveGroupCommander("xarm6")
        # self.goal_joint_angles = rospy.get_param("~Joint")

    def execute(self, userdata):
        # ros shutdown
        rospy.signal_shutdown("Task failed.")
        return 'failure'

            
    
# For debug
if __name__ == '__main__':
    rospy.init_node("xArm6")
    sm = smach.StateMachine(outcomes=['success', 'failure'])
    with sm:
        smach.StateMachine.add('WAIT4START', Wait4Start(['success', 'failure']), transitions={'success':'success', 'failure':'failure'})
    outcome = sm.execute()
    print(outcome)
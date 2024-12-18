#!/usr/bin/env python3
# coding: UTF-8

import os
import csv
import rospy
import rospkg
import smach
import moveit_commander
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_msgs.msg import ExecuteTrajectoryActionGoal
from node.grasp_control import GraspControl

rospack = rospkg.RosPack()
package_path = rospack.get_path('xarm6_pick_and_place_pkg')  # パッケージ名を指定


class PlaceWork(smach.State):
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

            # ゴールの設定(関節角度で指定)0.01342425 -0.8442685  -0.29798153  0.03872918  1.15796757  0.03068345
            #fixed_joint_values = [0.0027496605180203915, 0.104049913585186, -1.1940333843231201, 0.027469761669635773, 1.089946985244751, 0.008956530131399632]
            fixed_joint_values = [-0.724, 0.632, -1.553, 0.0, 0.921, 0.922]

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
                # グリッパーを開く
                if not self.gripper.open():
                    return "loop"
                rospy.loginfo("PlaceWork succeeded.")
                return "success"
            else:
                print("Execution failed.")
                return "loop"
        except Exception as e:
            print(f"Error in execute: {e}")
            return "loop"
    

class PLACE_BACK(smach.State):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes, input_keys=['start_time', 'plan_time', 'plan_size'], output_keys=['start_time', 'plan_time', 'plan_size'])
        self.robot = RobotCommander()
        self.xarm = MoveGroupCommander("xarm6")
        self.try_count = 0
        # self.goal_joint_angles = rospy.get_param("~Joint")

    def execute(self, userdata):
        print("------------------------------------")
        print("Executing PlaceBack")
        start_joint_values = self.xarm.get_current_joint_values()
        goal_joint_values = self.goal_joint_angles["Start"]

        # specify the pathseed file
        pathseed_params = rospy.get_param('/pathseed_param', {})
        # 逆再生を使用
        pathseed_params['reverse'] = True

        rospy.set_param('/pathseed_param', pathseed_params)

        self.xarm.set_start_state_to_current_state()
        self.xarm.set_joint_value_target(goal_joint_values)

        try:
            start_plan = rospy.Time.now()
            # プランニング
            self.xarm.set_goal_joint_tolerance(0.1)  # Increase the goal tolerance for joint position
            success_plan, plan, _, _ = self.xarm.plan()
            end_plan = rospy.Time.now()

            userdata.plan_time += (end_plan - start_plan).to_sec()
            plan_size = len(plan.joint_trajectory.points)
            userdata.plan_size += plan_size
            
            if success_plan:
                rospy.loginfo('Planning succeeded, executing plan')
                self.xarm.execute(plan)
            else:
                print("Planning failed.")
                if self.try_count < 3:
                    self.try_count += 1
                    return 'failure'
                return 'failure'
        
        except Exception as e:
            print(e)
            return 'failure'
        
        task_time = rospy.Time.now() - userdata.start_time
        task_time_seconds = task_time.to_sec()  # Convert time to seconds
        rospy.loginfo("Task time: %s seconds" % task_time_seconds)
        rospy.loginfo("Plan time: %s seconds" % userdata.plan_time)

        if rospy.get_param("write_csv") is True:
            # CSVファイルのパスをparam取得
            csv_file_path = rospy.get_param("/csv_file_path")
            # ヘッダーがまだ存在しない場合は追加する
            if not os.path.isfile(csv_file_path):
                with open(csv_file_path, 'w', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow(["Timestamp (s)", "Task Time (s)", "Plan Time (s)", "Plan Size[cols]","Success(1)/Failure(0)"])

            # データを書き込む
            with open(csv_file_path, 'a', newline='') as file:
                writer = csv.writer(file)
                timestamp = rospy.get_time()  # 現在のROS時間を取得
                writer.writerow([timestamp, task_time_seconds, userdata.plan_time, userdata.plan_size, 1])
        
        return 'success'
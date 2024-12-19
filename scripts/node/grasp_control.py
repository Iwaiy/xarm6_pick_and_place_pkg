#!/usr/bin/env python3
# coding: UTF-8

import rospy
from moveit_commander import RobotCommander, MoveGroupCommander
from control_msgs.msg import GripperCommandAction, GripperCommandActionGoal
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalID


class GraspControl:
    def __init__(self):
        """
        GraspControl クラスの初期化
        """
        # ロボット全体とグリッパーのグループを初期化
        self.robot = RobotCommander()
        self.gripper_group = MoveGroupCommander("xarm_gripper")
        rospy.loginfo("GraspControl initialized.")

    def close(self, position=0.85):
        """
        グリッパーを閉じる
        :param position: グリッパーの閉じる位置 (デフォルト: 0.85)
        :return: True if successful, False otherwise
        """
        pipeline = rospy.get_param("pipeline", "ompl")
        if pipeline == "ompl":
            try:
                rospy.loginfo(f"Closing gripper to position: {position}")
                self.gripper_group.set_joint_value_target({'drive_joint': position})
                success = self.gripper_group.go(wait=True)
                self.gripper_group.stop()
                if success:
                    rospy.loginfo("Gripper closed successfully.")
                else:
                    rospy.logwarn("Failed to close gripper.")
                return success
            except Exception as e:
                rospy.logerr(f"Error in closing gripper: {e}")
                return False
        # STOMP はグリッパーを閉じるプランニングができない
        else:
            try:
                # グリッパーを閉じる
                # ActionClientの作成
                gripper_client = SimpleActionClient('/xarm/xarm_gripper/gripper_action', GripperCommandAction)

                # サーバーが起動するのを待つ
                rospy.loginfo("Waiting for gripper action server...")
                gripper_client.wait_for_server()

                # 目標設定
                goal = GripperCommandActionGoal()
                print(f"Goal: {goal.goal}")
                goal.goal.command.position = position  # グリッパーを閉じる位置
                goal.goal.command.max_effort = 5.0  # 最大の力を設定

                # 目標を送信
                rospy.loginfo("Sending goal to close gripper...")
                gripper_client.send_goal(goal.goal)

                # 完了まで待機
                gripper_client.wait_for_result()
                rospy.loginfo("Gripper closed successfully.")

                return True
            except Exception as e:
                rospy.logerr(f"Error in closing gripper: {e}")
                return False

    def open(self, position=0.0):
        """
        グリッパーを開く
        :param position: グリッパーの開く位置 (デフォルト: 0.0)
        :return: True if successful, False otherwise
        """
        pipeline = rospy.get_param("pipeline", "ompl")
        if pipeline == "ompl":
            try:
                rospy.loginfo(f"Opening gripper to position: {position}")
                self.gripper_group.set_joint_value_target({'drive_joint': position})
                success = self.gripper_group.go(wait=True)
                self.gripper_group.stop()
                if success:
                    rospy.loginfo("Gripper opened successfully.")
                else:
                    rospy.logwarn("Failed to open gripper.")
                return success
            except Exception as e:
                rospy.logerr(f"Error in opening gripper: {e}")
                return False
        # STOMP はグリッパーを開くプランニングができない
        # STOMP はグリッパーを開くプランニングができない
        else:
            try:
                # グリッパーを開く
                # ActionClientの作成
                gripper_client = SimpleActionClient('/xarm/xarm_gripper/gripper_action', GripperCommandAction)

                # サーバーが起動するのを待つ
                rospy.loginfo("Waiting for gripper action server...")
                gripper_client.wait_for_server()

                # 目標設定
                goal = GripperCommandActionGoal()

                # goal.command に設定
                goal.goal.command.position = position  # グリッパーを開く位置
                goal.goal.command.max_effort = 5.0  # 最大の力を設定

                # 目標を送信
                rospy.loginfo("Sending goal to open gripper...")
                gripper_client.send_goal(goal.goal)

                # 完了まで待機
                gripper_client.wait_for_result()
                rospy.loginfo("Gripper opened successfully.")

                return True
            except Exception as e:
                rospy.logerr(f"Error in opening gripper: {e}")
                return False


if __name__ == "__main__":
    rospy.init_node("grasp_control_node", anonymous=True)
    grasp_control = GraspControl()

    # テスト: グリッパーを開閉
    rospy.loginfo("Testing gripper control...")
    for i in range(3):
        grasp_control.open()  # グリッパーを開く
        grasp_control.close()  # グリッパーを閉じる
    rospy.loginfo("Gripper control test completed.")

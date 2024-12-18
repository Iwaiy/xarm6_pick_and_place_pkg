#!/usr/bin/env python3
# coding: UTF-8

import rospy
from moveit_commander import RobotCommander, MoveGroupCommander


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

    def open(self, position=0.0):
        """
        グリッパーを開く
        :param position: グリッパーの開く位置 (デフォルト: 0.0)
        :return: True if successful, False otherwise
        """
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

if __name__ == "__main__":
    rospy.init_node("grasp_control_node", anonymous=True)
    grasp_control = GraspControl()

    # テスト: グリッパーを開閉
    rospy.loginfo("Testing gripper control...")
    grasp_control.open()  # グリッパーを開く
    rospy.sleep(2)  # 2秒待つ
    grasp_control.close()  # グリッパーを閉じる
    rospy.sleep(2)  # 2秒待つ
    rospy.loginfo("Gripper control test completed.")

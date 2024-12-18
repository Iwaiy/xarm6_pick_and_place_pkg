#! /usr/bin/env python3
# coding: UTF-8

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose
import moveit_commander
import ast
import random
import sys
import os
import rospkg

rospack = rospkg.RosPack()
file_path = os.path.join(rospack.get_path('xarm6_pick_and_place_pkg'), 'decoded_data', 'generated_path_20241001_002207.txt')
file_path = "/home/nishidalab/train_ws/src/xarm6_pick_and_place_pkg/decoded_data/Path_Check_20241017_154420/generated_path_7.txt"
file_paths = [file_path]

# MoveIt!の初期化
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
group = robot.get_group("xarm6")  # 使用するアームのグループ名を指定

# C空間（ジョイント角度）からT空間（エンドエフェクタ位置）のPoseを設定
def set_pose_from_joint_angles(joint_angles):
    # ジョイント角度を設定してエンドエフェクタの位置を計算
    group.set_joint_value_target(joint_angles)
    plan = group.plan()
    # モーションを実行
    group.go(wait=True)
    # エンドエフェクタの位置を取得
    pose = group.get_current_pose().pose
    return pose

# エンドエフェクタの位置を可視化
def add_marker(marker_array, pose, marker_id):
    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "end_effector_markers"
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose = pose
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05

    # Red color
    marker.color.r = 1.0
    marker.color.a = 1.0

    marker_array.markers.append(marker)

def parse_joint_data(line):
    # 行から不要な文字（, や ;）を取り除き、値を1つずつfloatに変換
    values = []
    for item in line.replace(';', ',').split(','):
        item = item.strip()
        if item:  # 空文字列でないことを確認
            try:
                values.append(float(item))
            except ValueError:
                rospy.logwarn(f"Cannot convert to float: {item} in line: {line.strip()}")
                return None  # 無効なデータ行はNoneを返す
    return values

def main():
    rospy.init_node("visualize_end_effector_trajectory")
    marker_pub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=10)
    rospy.sleep(1)  # パブリッシャーのセットアップ待ち

    poses_list = []
    marker_id = 0  # マーカーのIDを初期化
    marker_array = MarkerArray()

    for file_path in file_paths:
        with open(file_path, "r") as file:
            for line in file:
                # ジョイントデータをパース
                joint_data = parse_joint_data(line.strip())
                
                if joint_data is None or len(joint_data) != 6:
                    rospy.logwarn(f"Invalid data in line: {line.strip()}")
                    continue  # 無効なデータ行はスキップ
                
                try:
                    # 取得したジョイントデータを使ってポーズを設定
                    pose = set_pose_from_joint_angles(joint_data)
                    print("pose: ", pose)
                    poses_list.append(pose)
                    
                    # マーカーを追加
                    add_marker(marker_array, pose, marker_id)
                    marker_id += 1  # マーカーIDをインクリメント
                    
                    # マーカーをパブリッシュ
                    marker_pub.publish(marker_array)
                    
                    # rospy.sleep(0.2)  # マーカーが重ならないように少し待つ
                except Exception as e:
                    rospy.logwarn(f"Error in processing pose from joint angles: {e}")
    
    rospy.loginfo(f"Visualizing {len(poses_list)} poses")
    
    rospy.spin()

if __name__ == "__main__":
    main()

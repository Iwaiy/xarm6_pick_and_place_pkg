#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from moveit_msgs.msg import ExecuteTrajectoryActionGoal
import datetime
import os
import re
import sys
import numpy as np
import rospkg
import smach
import ast
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
import moveit_commander
from moveit_commander import RobotCommander, MoveGroupCommander
from node.decode import Decoder
from node.encode import Encoder

np.set_printoptions(threshold=np.inf) # 結果を省略せず全て表示
np.set_printoptions(linewidth=np.inf) # 行列が途中で自動改行されないようにする
np.set_printoptions(suppress=True) # 指数表記（e）を用いない

rospack = rospkg.RosPack()
package_path = rospack.get_path('xarm6_pick_and_place_pkg')  # パッケージ名を指定


class UpdatePathSeed(smach.State):
    def __init__(self, outcomes):
        # Declare input_keys and output_keys
        smach.State.__init__(self, outcomes=outcomes, input_keys=['start_time', 'plan_time', 'plan_size', 'plan_info'], output_keys=['start_time', 'plan_time', 'plan_size', 'plan_info'])
        self.count = 0
        self.pipeline = rospy.get_param("pipeline", "stomp") # Get the pipeline parameter(stomp or ompl)
        self.encoder = Encoder()
        self.decoder = Decoder()

    def save_pathseed(self, pathseed_data, file_name):
        """
        パスシードをファイルに保存する関数
        Args:
            pathseed_data: パスシードデータ
            file_name: ファイル名
        """
        try:
            with open(file_name, "w") as file:
                # pathseed_dataをファイルに書き込む
                # NumPy配列をリストに変換して保存
                if isinstance(pathseed_data['r1_array'], np.ndarray):
                    pathseed_data['r1_array'] = pathseed_data['r1_array'].tolist()
                if isinstance(pathseed_data['r2_array'], np.ndarray):
                    pathseed_data['r2_array'] = pathseed_data['r2_array'].tolist()
                if isinstance(pathseed_data['tau1_array'], np.ndarray):
                    pathseed_data['tau1_array'] = pathseed_data['tau1_array'].tolist()
                if isinstance(pathseed_data['tau2_array'], np.ndarray):
                    pathseed_data['tau2_array'] = pathseed_data['tau2_array'].tolist()
                if isinstance(pathseed_data['delta_array'], np.ndarray):
                    pathseed_data['delta_array'] = pathseed_data['delta_array'].tolist()
                file.write(str(pathseed_data))

            print(f"データが {file_name} に書き込まれました。")
        except IOError as e:
            print(f"ファイルの書き込み中にエラーが発生しました: {e}")

    def execute(self, userdata):
        print("------------------------------------")
        print("Executing UpdatePathSeed")

        # get the current phase
        self.phase = rospy.get_param("phase", "Initial_Phase")
        
        # pathseedsフォルダ内にupdate_pathseedsフォルダにtrajectoriesフォルダが存在しない場合はreturn
        update_pathseeds_dir = os.path.join(package_path, 'pathseeds', 'update_pathseeds')
        trajectories_dir = os.path.join(update_pathseeds_dir, 'trajectories')
        if not os.path.exists(trajectories_dir):
            rospy.logerr(f"{trajectories_dir} does not exist")
            return 'failure'
        
        # trajectoriesフォルダ内のファイルを取得
        traj_pick_file_path = os.path.join(trajectories_dir, 'traj_pick.txt')
        traj_place_file_path = os.path.join(trajectories_dir, 'traj_place.txt')

        if self.phase == 'Initial_Phase':
            rospy.loginfo(f"{self.phase} -> UpdatePathSeed")
            # anglesフォルダをなければ作成
            angles_dir = os.path.join(update_pathseeds_dir, 'angles')
            if not os.path.exists(angles_dir):
                os.makedirs(angles_dir, exist_ok=True)
            # anglesフォルダ内にway_point_pick.txt, way_point_place.txtを作成
            angle_file_pick_path = os.path.join(angles_dir, 'angle_pick.txt')
            angle_file_place_path = os.path.join(angles_dir, 'angle_place.txt')

            # trahectoryファイルからpositionを取得し，angleファイルに書き込む
            with open(traj_pick_file_path, 'r') as file:
                traj_pick_data = file.read()
                pattern = r'positions:\s*(\[.*?\])'
                matches_pick = re.findall(pattern, traj_pick_data)

            with open(angle_file_pick_path, 'w') as file:
                for match in matches_pick:
                    match_with_comma = match.replace(']', ']')
                    file.write(str(match_with_comma + '\n'))

            with open(traj_place_file_path, 'r') as file:
                traj_place_data = file.read()
                pattern = r'positions:\s*(\[.*?\])'
                matches_place = re.findall(pattern, traj_place_data)

            with open(angle_file_place_path, 'w') as file:
                for match in matches_place:
                    match_with_comma = match.replace(']', ']')
                    angle_place_data = match_with_comma + '\n'
                    file.write(str(angle_place_data))

            pathseed_pick = self.encoder.generate_pathseed(waypoint_file=angle_file_pick_path, directory=angles_dir)
            pathseed_place = self.encoder.generate_pathseed(waypoint_file=angle_file_place_path, directory=angles_dir)

            self.save_pathseed(pathseed_pick, os.path.join(update_pathseeds_dir, 'pathseed_pick.txt'))
            self.save_pathseed(pathseed_place, os.path.join(update_pathseeds_dir, 'pathseed_place.txt'))
            
            rospy.loginfo("Completed UpdatePathSeed")
            rospy.set_param('phase', 'Implement_Phase')
        else:
            rospy.loginfo(f"{self.phase} -> No need to update pathseed")

        # # For experiments
        # if rospy.get_param("write_angle_data", True) is True:
        #     angle_path_for_experiment = rospy.get_param("angle_path_for_experiment")
        #     angles_pick_dir_for_experiments = os.path.join(angle_path_for_experiment, self.pipeline, 'angle_pick')
        #     angles_place_dir_for_experiments = os.path.join(angle_path_for_experiment, self.pipeline, 'angle_place')
        #     if not os.path.exists(angles_pick_dir_for_experiments):
        #         os.makedirs(angles_pick_dir_for_experiments, exist_ok=True)
        #     if not os.path.exists(angles_place_dir_for_experiments):
        #         os.makedirs(angles_place_dir_for_experiments, exist_ok=True)
            
        #     with open(traj_pick_file_path, 'r') as file:
        #         traj_pick_data = file.read()
        #         pattern = r'positions:\s*(\[.*?\])'
        #         matches_pick = re.findall(pattern, traj_pick_data)
        #     with open(os.path.join(angles_pick_dir_for_experiments, f'angle_pick_{self.count}.txt'), 'w') as file:
        #         for match in matches_pick:
        #             match_with_comma = match.replace(']', ']')
        #             file.write(str(match_with_comma + '\n'))
        #     with open(traj_place_file_path, 'r') as file:
        #         traj_place_data = file.read()
        #         pattern = r'positions:\s*(\[.*?\])'
        #         matches_place = re.findall(pattern, traj_place_data)
        #     with open(os.path.join(angles_place_dir_for_experiments, f'angle_place_{self.count}.txt'), 'w') as file:
        #         for match in matches_place:
        #             match_with_comma = match.replace(']', ']')
        #             angle_place_data = match_with_comma + '\n'
        #             file.write(str(angle_place_data))
        #     self.count += 1

        return 'success'
#!/usr/bin/env python3
# coding: UTF-8

import os
import csv
import rospy
import rospkg
import smach
from std_msgs.msg import Bool
import moveit_commander
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_msgs.msg import ExecuteTrajectoryActionGoal
from node.grasp_control import GraspControl
from node.decode import Decoder
from node.compute_ik import ComputeIK

rospack = rospkg.RosPack()
package_path = rospack.get_path('xarm6_pick_and_place_pkg')  # パッケージ名を指定


class PlaceWork(smach.State):
    def __init__(self, outcomes):
        # Declare input_keys and output_keys
        smach.State.__init__(self, outcomes=outcomes)

        self.robot = RobotCommander()
        self.xarm = MoveGroupCommander("xarm6")
        self.gripper = GraspControl()
        self.compute_ik = ComputeIK()
        self.try_count = 0
        self.pipeline = rospy.get_param("pipeline", "stomp")
        self.is_finish_task = False

        # parameters
        self.params = rospy.get_param("~Params")

        # Publisher
        self.enable_pathseed_pub = rospy.Publisher("pathseed_control", Bool, queue_size=10)
        self.rate = rospy.Rate(50)  # 50 Hz

        # Subscriber
        self.subscriber = rospy.Subscriber("/execute_trajectory/goal", ExecuteTrajectoryActionGoal, self.execute_trajectory_callback)

    def execute_trajectory_callback(self, msg):
        # Callback to handle the subscribed data
        rospy.loginfo("Received trajectory goal data")
        self.msg = msg

    def set_stomp_params(self, phase: dict) -> dict:
        """
        Set the Stomp parameters
        Args:
            phase (dict): The phase parameters
        Returns:
            dict: The phase parameters
        """
        try:
            # set Stomp parameters
            rospy.set_param("move_group/stomp/xarm6/optimization/num_timesteps", phase['num_timesteps'])
            rospy.set_param("move_group/stomp/xarm6/optimization/num_iterations", phase['num_iterations'])
            rospy.set_param("move_group/stomp/xarm6/optimization/num_iterations_after_valid", phase['num_iterations_after_valid'])
            rospy.set_param("move_group/stomp/xarm6/optimization/num_rollouts", phase['num_rollouts'])
            rospy.set_param("move_group/stomp/xarm6/optimization/max_rollouts", phase['max_rollouts'])
            rospy.set_param("move_group/stomp/xarm6/optimization/initialization_method", phase['initialization_method'])
            rospy.set_param("move_group/stomp/xarm6/optimization/control_cost_weight", phase['control_cost_weight'])

            # パラメータの設定
            noise_generator_params = [
                {
                    'class': 'stomp_moveit/NormalDistributionSampling',
                    'stddev': phase['stddev']
                }
            ]
            # rosparamに設定
            rospy.set_param('/move_group/stomp/xarm6/task/noise_generator', noise_generator_params)

            return phase
        except Exception as e:
            rospy.logerr(f"Error occurred: {e}")
            return None

    def execute(self, userdata):
        # settings
        # xarmの速度と加速度を設定
        self.xarm.set_max_velocity_scaling_factor(0.5)  # 50% の速度
        self.xarm.set_max_acceleration_scaling_factor(0.25)  # 25% の加速度
        self.msg = None
        self.is_finish_task = False
        env = rospy.get_param("env", "task1")
        # set the start joint values and goal joint values
        start_joint_values = self.xarm.get_current_joint_values()

        # set the T space or C space coordinates
        mode_space = "C"
        if mode_space == "T" and "Tspace" in self.params[env]["Joint"]["PlacePoint"]:
            # Set the pick object pose (T space coordinates)
            # TODO: 決め打ちではなく，認識結果 or UI から取得する
            pick_obj_pose = self.params[env]["Joint"]["PlacePoint"]["Tspace"]
            joint_values = self.compute_ik.inverse_kinematics(x=pick_obj_pose[0], y=pick_obj_pose[1], z=pick_obj_pose[2])
        else:
            # Set the pick object pose (C space coordinates)
            joint_values = self.params[env]["Joint"]["PlacePoint"]["Cspace"]

        # ゴールの決定
        goal_joint_values = joint_values

        # log for the planning pipline (stomp or ompl)
        rospy.loginfo(f"Planning pipeline: {self.pipeline}")
        self.phase = rospy.get_param("phase", "Exception")

        # PR method の場合，前処理（デコード）を行う
        if self.pipeline == "stomp" and rospy.get_param("use_pathseed", False) is True:
            # enable the pathseed publisher
            rospy.loginfo("Publishing pathseed")
            self.enable_pathseed_pub.publish(Bool(data=True))
            rospy.set_param("is_publish_pathseed", False)
            # get the current phase
            rospy.loginfo(f"Current phase: {self.phase}")
            stomp_params = self.params[env]["StompParams"]['PlacePoint'][self.phase]
            if self.phase == "Initial_Phase":
                # specify the file path
                # file_path = os.path.join(package_path, 'pathseeds', 'new_ex', 'pathseed_straight.txt')
                file_path = os.path.join(package_path, 'pathseeds', 'task1', 'pathseed2.txt')
                print("parameters set")
                # set Stomp parameters
                stomp_params = self.set_stomp_params(stomp_params)
                rospy.loginfo(f"Stomp parameters set: {stomp_params}")
            elif self.phase == "Implement_Phase":
                try:
                    # specify the file path
                    file_path = os.path.join(package_path, 'pathseeds', 'update_pathseeds', 'pathseed_place.txt')
                except FileNotFoundError:
                    rospy.logerr("File not found")
                    self.try_count = 0
                    return 'failure'
                print("parameters set")
                stomp_params = self.set_stomp_params(stomp_params)
                rospy.loginfo(f"Stomp parameters set: {stomp_params}")
            else:
                rospy.logerr("Invalid phase")
                return 'failure'

            # decode the pathseed file
            decoder = Decoder()
            generated_path = decoder.generate_path(file_path, start_joint_values, goal_joint_values)

            # specify the pathseed file
            pathseed_params = rospy.set_param('/pathseed_param', {
                'path_data': generated_path,
                'reverse': False  # デフォルト値
            })

        # Planning
        try:
            # self.xarm.set_max_velocity_scaling_factor(0.1)  # 10% の速度
            # self.xarm.set_max_acceleration_scaling_factor(0.1)  # 10% の加速度
            self.xarm.stop()

            # ゴールの設定(関節角度で指定) [-0.724, 0.632, -1.553, 0.0, 0.921, 0.922]
            goal_joint_values = self.params["task1"]["Joint"]["PlacePoint"]["Cspace"]

            print(f"Current joint values (Start): {start_joint_values}")

            # ゴール状態（目標ジョイント値）を表示
            print(f"Target joint values (Goal): {goal_joint_values}")

            # スタート状態を現在の状態に設定
            self.xarm.set_start_state_to_current_state()

            # ゴール状態を設定
            self.xarm.set_joint_value_target(goal_joint_values)

            # プランニング
            success, plan, _, _ = self.xarm.plan()
            if not success:
                print("Planning failed.")
                return "loop"

            print("Planning succeeded. Executing plan...")
            success_exec = self.xarm.execute(plan)
            if success_exec:
                # create the directories if they do not exist
                if not os.path.exists(os.path.join(package_path, 'pathseeds', 'update_pathseeds', 'trajectories')):
                    os.makedirs(os.path.join(package_path, 'pathseeds', 'update_pathseeds', 'trajectories'), exist_ok=True)
                # write the execute_trajectory callback in the traj_pick.txt file
                with open(os.path.join(package_path, 'pathseeds', 'update_pathseeds', 'trajectories', 'traj_place.txt'), 'w') as file:
                    file.write(str(self.msg))
                rospy.loginfo("Trajectory data written to traj_place.txt")
                # グリッパーを開く
                if not self.gripper.open():
                    return "loop"
                rospy.loginfo("PlaceWork succeeded.")

                # check if the task is finished
                self.is_finish_task = rospy.get_param("is_finish_task", False)

                # タスク終了の場合，ホームポジションに戻る
                if self.is_finish_task is True:
                    # reset the is_finish_task parameter
                    rospy.set_param("is_finish_task", False)
                    # go back to the start position
                    rospy.loginfo("Go back to home position.")
                    # unuse the pathseed
                    use_pathseed = rospy.get_param("use_pathseed", False)
                    if use_pathseed is True:
                        rospy.set_param("use_pathseed", False)
                    goal_joint_values = self.params[env]["Joint"]["Start"]
                    stomp_params = self.params[env]["StompParams"]['Start']
                    _ = self.set_stomp_params(stomp_params)
                    rospy.loginfo(f"Goal joint values: {goal_joint_values}")
                    rospy.loginfo(f"Stomp parameters: {stomp_params}")
                    self.xarm.set_start_state_to_current_state()
                    self.xarm.set_joint_value_target(goal_joint_values)
                    success = self.xarm.go()
                    if success:
                        rospy.loginfo('Success to go back to home position')
                        rospy.set_param("phase", "Initial_Phase")
                        if use_pathseed is True:
                            rospy.set_param("use_pathseed", True)
                        return 'finish'
                    else:
                        rospy.logerr('Failed to go back to home position')
                        rospy.loginfo('Try to plan again')
                        self.xarm.set_named_target("home")
                        self.xarm.go()
                        rospy.set_param("phase", "Initial_Phase")
                        if use_pathseed is True:
                            rospy.set_param("use_pathseed", True)
                        return 'finish'
                
                # タスク終了でない場合，次のフェーズに進む
                return "update"
            else:
                print("Execution failed.")
                return "loop"
        except Exception as e:
            print(f"Error in execute: {e}")
            return "loop"
    

# class PLACE_BACK(smach.State):
#     def __init__(self, outcomes):
#         smach.State.__init__(self, outcomes=outcomes, input_keys=['start_time', 'plan_time', 'plan_size'], output_keys=['start_time', 'plan_time', 'plan_size'])
#         self.robot = RobotCommander()
#         self.xarm = MoveGroupCommander("xarm6")
#         self.try_count = 0
#         # self.goal_joint_angles = rospy.get_param("~Joint")

#     def execute(self, userdata):
#         print("------------------------------------")
#         print("Executing PlaceBack")
#         start_joint_values = self.xarm.get_current_joint_values()
#         goal_joint_values = self.goal_joint_angles["Start"]

#         # specify the pathseed file
#         pathseed_params = rospy.get_param('/pathseed_param', {})
#         # 逆再生を使用
#         pathseed_params['reverse'] = True

#         rospy.set_param('/pathseed_param', pathseed_params)

#         self.xarm.set_start_state_to_current_state()
#         self.xarm.set_joint_value_target(goal_joint_values)

#         try:
#             start_plan = rospy.Time.now()
#             # プランニング
#             self.xarm.set_goal_joint_tolerance(0.1)  # Increase the goal tolerance for joint position
#             success_plan, plan, _, _ = self.xarm.plan()
#             end_plan = rospy.Time.now()

#             userdata.plan_time += (end_plan - start_plan).to_sec()
#             plan_size = len(plan.joint_trajectory.points)
#             userdata.plan_size += plan_size
            
#             if success_plan:
#                 rospy.loginfo('Planning succeeded, executing plan')
#                 self.xarm.execute(plan)
#             else:
#                 print("Planning failed.")
#                 if self.try_count < 3:
#                     self.try_count += 1
#                     return 'failure'
#                 return 'failure'
        
#         except Exception as e:
#             print(e)
#             return 'failure'
        
#         task_time = rospy.Time.now() - userdata.start_time
#         task_time_seconds = task_time.to_sec()  # Convert time to seconds
#         rospy.loginfo("Task time: %s seconds" % task_time_seconds)
#         rospy.loginfo("Plan time: %s seconds" % userdata.plan_time)

#         if rospy.get_param("write_csv") is True:
#             # CSVファイルのパスをparam取得
#             csv_file_path = rospy.get_param("/csv_file_path")
#             # ヘッダーがまだ存在しない場合は追加する
#             if not os.path.isfile(csv_file_path):
#                 with open(csv_file_path, 'w', newline='') as file:
#                     writer = csv.writer(file)
#                     writer.writerow(["Timestamp (s)", "Task Time (s)", "Plan Time (s)", "Plan Size[cols]","Success(1)/Failure(0)"])

#             # データを書き込む
#             with open(csv_file_path, 'a', newline='') as file:
#                 writer = csv.writer(file)
#                 timestamp = rospy.get_time()  # 現在のROS時間を取得
#                 writer.writerow([timestamp, task_time_seconds, userdata.plan_time, userdata.plan_size, 1])
        
#         return 'success'
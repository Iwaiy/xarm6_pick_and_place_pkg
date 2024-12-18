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

rospack = rospkg.RosPack()
package_path = rospack.get_path('xarm6_pick_and_place_pkg')  # パッケージ名を指定


class PlaceWork(smach.State):
    def __init__(self, outcomes):
        # Declare input_keys and output_keys
        smach.State.__init__(self, outcomes=outcomes, input_keys=['start_time', 'plan_time', 'plan_size'], output_keys=['start_time', 'plan_time', 'plan_size'])
        self.robot = RobotCommander()
        self.xarm = MoveGroupCommander("xarm6")
        self.try_count = 0
        # self.goal_joint_angles = rospy.get_param("~Joint")
        self.subscriber = None  # Initialize the subscriber as None
        self.msg = None

    def execute_trajectory_callback(self, msg):
        # Callback to handle the subscribed data
        rospy.loginfo("Received trajectory goal data")
        self.msg = msg

    def execute(self, userdata):
        # init
        self.try_count = 0
        self.subscriber = None
        self.msg = None
        
        # get the current phase
        self.phase = rospy.get_param("phase", "Exception")
        rospy.loginfo(f"Current phase: {self.phase}")
        if self.phase == "Initial_Phase":
            # specify the file path
            file_path = os.path.join(package_path, 'pathseeds', 'new_ex', 'pathseed_straight.txt')
            print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
            print("parameters set")
            print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
            # set Stomp parameters
            rospy.set_param("move_group/stomp/xarm6/optimization/num_timesteps", 60)
            rospy.set_param("move_group/stomp/xarm6/optimization/num_iterations", 30)
            rospy.set_param("move_group/stomp/xarm6/optimization/num_iterations_after_valid", 10)
            rospy.set_param("move_group/stomp/xarm6/optimization/num_rollouts", 30)
            rospy.set_param("move_group/stomp/xarm6/optimization/max_rollouts", 30)
            rospy.set_param("move_group/stomp/xarm6/optimization/initialization_method", 1)
            rospy.set_param("move_group/stomp/xarm6/optimization/control_cost_weight", 0.0)
            # Subscribe only if in the Initial_Phase
            if not self.subscriber:
                self.subscriber = rospy.Subscriber("/execute_trajectory/goal", ExecuteTrajectoryActionGoal, self.execute_trajectory_callback)
                rospy.loginfo("Subscribed to /execute_trajectory/goal")

            # create the directories if they do not exist
            if not os.path.exists(os.path.join(package_path, 'pathseeds', 'update_pathseeds', 'trajectories')):
                os.makedirs(os.path.join(package_path, 'pathseeds', 'update_pathseeds', 'trajectories'), exist_ok=True)
        elif self.phase == "Implement_Phase":
            try:
                # specify the file path
                file_path = os.path.join(package_path, 'pathseeds', 'update_pathseeds', 'pathseed_place.txt')
                print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
                print("parameters set")
                print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
                # set Stomp parameters
                rospy.set_param("move_group/stomp/xarm6/optimization/num_timesteps", 10)
                rospy.set_param("move_group/stomp/xarm6/optimization/num_iterations", 5)
                rospy.set_param("move_group/stomp/xarm6/optimization/num_iterations_after_valid", 10)
                rospy.set_param("move_group/stomp/xarm6/optimization/num_rollouts", 10)
                rospy.set_param("move_group/stomp/xarm6/optimization/max_rollouts", 30)
                rospy.set_param("move_group/stomp/xarm6/optimization/initialization_method", 1)
                rospy.set_param("move_group/stomp/xarm6/optimization/control_cost_weight", 0.0)
            except FileNotFoundError:
                rospy.logerr("File not found")
                return 'failure'
        else:
            rospy.logerr("Invalid phase")
            return 'failure'

        # decode the pathseed file
        decoder = Decoder()
        start_joint_values = self.xarm.get_current_joint_values()
        goal_joint_values = self.goal_joint_angles["PlacePoint"]
        generated_path = decoder.generate_path(file_path, start_joint_values, goal_joint_values)

        # specify the pathseed file
        pathseed_params = rospy.set_param('/pathseed_param', {
            'path_data': generated_path,
            'reverse': False  # デフォルト値
        })

        rospy.loginfo('Going to place point')
        # スタート位置の設定(関節角度で指定)
        # fixed_joint_values = self.goal_joint_angles["PlacePoint"]
        # self.xarm.set_start_state_to_current_state()
        # self.xarm.set_joint_value_target(fixed_joint_values)
        self.xarm.set_start_state_to_current_state()
        self.xarm.set_joint_value_target(goal_joint_values)

        # # set Stomp parameters
        # rospy.set_param("move_group/stomp/xarm6/optimization/num_timesteps", 6)
        # rospy.set_param("move_group/stomp/xarm6/optimization/num_iterations", 6)
        # rospy.set_param("move_group/stomp/xarm6/optimization/num_iterations_after_valid", 10)
        # rospy.set_param("move_group/stomp/xarm6/optimization/num_rollouts", 6)
        # rospy.set_param("move_group/stomp/xarm6/optimization/max_rollouts", 15)
        # rospy.set_param("move_group/stomp/xarm6/optimization/initialization_method", 1)
        # rospy.set_param("move_group/stomp/xarm6/optimization/control_cost_weight", 0.0)

        # パラメータの設定
        noise_generator_params = [
            {
                'class': 'stomp_moveit/NormalDistributionSampling',
                'stddev': [0.001, 0.001, 0.008, 0.008, 0.005, 0.001]
            }
        ]
        # Cost function parameters
        cost_functions = [
            {
                'class': 'stomp_moveit/CollisionCheck',
                'collision_penalty': 100.0,
                'cost_weight': 100.0,
                'kernel_window_percentage': 0.2,
                'longest_valid_joint_move': 0.05
            }
        ]

        # Set the parameters
        rospy.set_param('/move_group/stomp/xarm6/task/cost_functions', cost_functions)

        # Confirm the parameters have been set correctly
        print(rospy.get_param('/move_group/stomp/xarm6/task/cost_functions'))

        # rosparamに設定
        rospy.set_param('/move_group/stomp/xarm6/task/noise_generator', noise_generator_params)

        # 設定した内容を確認
        rospy.loginfo("Noise generator parameters set: %s", rospy.get_param('/move_group/stomp/xarm6/task/noise_generator'))

        start_plan = rospy.Time.now()
        # プランニング
        self.xarm.set_goal_joint_tolerance(0.1)  # Increase the goal tolerance for joint position
        success, plan, _, _ = self.xarm.plan()
        
        end_plan = rospy.Time.now()

        userdata.plan_time += (end_plan - start_plan).to_sec()
        plan_size = len(plan.joint_trajectory.points)
        userdata.plan_size += plan_size

        # print(plan)

        if success:
            rospy.loginfo('Planning succeeded, executing plan')
            self.xarm.execute(plan)
        else:
            print("Planning failed.")
            if self.try_count < 3:
                    self.try_count += 1
                    return 'failure'
            return 'failure'
        
        if self.phase == "Initial_Phase":
            # write the execute_trajectory callback in the traj_pick.txt file
            with open(os.path.join(package_path, 'pathseeds', 'update_pathseeds', 'trajectories', 'traj_place.txt'), 'w') as file:
                file.write(str(self.msg))
            rospy.loginfo("Trajectory data written to traj_place.txt")
        return 'success'
    

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
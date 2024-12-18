#!/usr/bin/env python3
# coding: UTF-8

import os
import sys
import rospy
import rosparam
import rospkg
import numpy as np
from xarm.wrapper import XArmAPI
import tf
import tf2_ros
from tf2_geometry_msgs import PoseStamped, PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import Point, Vector3
import smach
import moveit_commander
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_msgs.msg import ExecuteTrajectoryActionGoal
import time

from geometry_msgs.msg import Pose
import tf.transformations as tft
from node.grasp_control import GraspControl

from tf.transformations import quaternion_from_euler, quaternion_multiply

rospack = rospkg.RosPack()
package_path = rospack.get_path('xarm6_pick_and_place_pkg')  # パッケージ名を指定
# IK計算用の関数


class PickWork(smach.State):
    def __init__(self, outcomes):
        # Declare input_keys and output_keys
        smach.State.__init__(self, outcomes=outcomes, input_keys=['start_time', 'plan_time', 'plan_size'], output_keys=['start_time', 'plan_time', 'plan_size'])
        self.robot = RobotCommander()
        self.xarm = MoveGroupCommander("xarm6")
        self.gripper = GraspControl()
        self.try_count = 0
        # self.goal_joint_angles = rospy.get_param("~Joint")
        self.subscriber = None  # Initialize the subscriber as None
        self.msg = None
        self.target_pose = None
        self.tf_buffer = Buffer()

        self.frame_id = None

        # Subscriber
        #TFブロードキャスト
        self.br = tf2_ros.StaticTransformBroadcaster()
        self.tf_subscriber = rospy.Subscriber("/tf_static", TFMessage, self.tf_static_callback, queue_size=100000)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_buffer.clear()


    def tf_static_callback(self, msg):
        # TFメッセージを受信した際に呼ばれる
        # rospy.loginfo("Received TF static message")
        if self.frame_id is not None:
            return
        self.target_pose = Pose()

        for transform in msg.transforms:
            if transform.child_frame_id == "Posture_of_object":
                self.frame_id = transform.header.frame_id
                self.child_frame_id = transform.child_frame_id
                # Directly set the target_pose from the transform data
                self.target_pose.position = transform.transform.translation
                self.target_pose.orientation = transform.transform.rotation

    def transform_pose(self, source_pose, source_frame, target_frame):
        """
        Transform a Pose from the source frame to the target frame.

        :param source_pose: The Pose to be transformed (geometry_msgs.msg.Pose)
        :param source_frame: The source frame ID (string)
        :param target_frame: The target frame ID (string)
        :return: The transformed Pose (geometry_msgs.msg.Pose)
        """
        self.tf_buffer.clear()
        listener = tf.TransformListener()

        # Create a PoseStamped object for the source pose
        source_pose_stamped = PoseStamped()
        source_pose_stamped.header.frame_id = source_frame
        source_pose_stamped.header.stamp = rospy.Time(0)  # Ensure current time is used
        source_pose_stamped.pose = source_pose

        try:
            # Wait for the transform to be available
            rospy.loginfo(f"Waiting for transform from {source_frame} to {target_frame}")
            listener.waitForTransform(target_frame, source_frame, time=rospy.Time(0), timeout=rospy.Duration(10))  # Adjust the wait duration as needed
            rospy.sleep(0.1)

            # Transform the pose to the target frame
            transformed_pose_stamped = listener.transformPose(target_frame, source_pose_stamped)
            #self.publish_transform(transformed_pose_stamped.pose, "grasp")
            
            # Return the transformed pose
            return transformed_pose_stamped.pose

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"Error during transformation: {e}")
            return None
        

    def publish_transform(self, pose: Pose, header_name: str, frame_name: str) -> None:
        """
        Publish a static transform from the given Pose to the given frame name.
        ::param pose: The Pose to be transformed (geometry_msgs.msg.Pose)
        ::param header_name: The name of the source frame (string)
        ::param frame_name: The name of the target frame (string)
        ::return: None
        """
        transform_msg = TransformStamped()
		
		# ROS Header
        transform_msg.header.stamp = rospy.Time.now()

        transform_msg.header.frame_id = header_name
        transform_msg.child_frame_id = frame_name  # ソースフレームの名前（適宜変更）

        translation = Vector3()
        
        # Convert Point to Vector3 by copying the x, y, z values
        translation.x = float(pose.position.x)
        translation.y = float(pose.position.y)
        translation.z = float(pose.position.z)

        # Quaternionに変換
        quaternion = pose.orientation

        # TransformStampedメッセージに設定
        transform_msg.transform.translation = translation
        transform_msg.transform.rotation = quaternion

        #print(translation)
        # トランスフォームをブロードキャスト
        self.br.sendTransform(transform_msg)

    def quaternion_to_euler(self, quaternion):
        """Convert Quaternion to Euler Angles

        quarternion: geometry_msgs/Quaternion
        euler: geometry_msgs/Vector3
        """
        e = np.degrees(tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w), axes='sxyz'))
        return Vector3(x=e[0], y=e[1], z=e[2])
    
    def move_to_relative_position(self, pose):       
        code = self._arm.set_tool_position(*pose, speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)  ###エンドエフェクタの先端を基準とする相対座標系
        if not self._check_code(code, 'set_position'):
            return False
        return True

    def execute(self, userdata):
        # init
        self.try_count = 0
        self.msg = None
        print("------------------------------------")
        print("Executing PickWork")
        # 姿勢推定によるtargetのPose取得 (Recognition)
        ################################################################################################################################
        
        # Wait for the posture of object transform for 10 seconds
        wait_timeout = 20
        rospy.loginfo("Waiting for the posture of object transform...")
        # Initialize the posture estimation flag
        ### False :: 認識を行う
        ### True :: 認識を行わない
        self.frame_id = None
        rospy.set_param('/posture_estimation_done', False)
        start_time = rospy.Time.now()
        while self.frame_id is None:
            if (rospy.Time.now() - start_time).to_sec() > wait_timeout:
                rospy.logerr("Failed to receive 'Posture_of_object' transform.")
                return 'failure'
            rospy.sleep(0.1)
        # Set the posture estimation flag to True
        rospy.set_param('/posture_estimation_done', True)
        rospy.loginfo("Received 'Posture_of_object' transform.")
        #rospy.loginfo(f"target_pose: {self.target_pose}")
        source_pose = self.target_pose

        ##############################################################################
        ##############################################################################
        # convert the link_base座標系 from camera_depth_optical_frame
        transformed_pose = self.transform_pose(source_pose, "camera_depth_optical_frame", "world")
        rospy.loginfo(f"transformed_pose: {transformed_pose}")
        ##############################################################################
        ##############################################################################

        # if transformed_pose is not None:
        #     # m -> mm に変換
        #     # transformed_pose.position.x *= 1000
        #     # transformed_pose.position.y *= 1000
        #     # transformed_pose.position.z *= 1000
        #     rospy.loginfo(f"Transformed Pose: {transformed_pose}")

        # else:
        #     rospy.logerr("Pose transformation failed.")

        transformed_euler = self.quaternion_to_euler(transformed_pose.orientation)
        source_euler = self.quaternion_to_euler(source_pose.orientation)

        rospy.loginfo(f"transformed_pose: {transformed_euler.x,transformed_euler.y,transformed_euler.z}")
        rospy.loginfo(f"source_pose: {source_euler.x,source_euler.y,source_euler.z}")
        rospy.loginfo(f"{transformed_euler.z + 90}")

        # Move to Target Position with offset
        ################################################################################################################################
        finger = 17  ###   finger size 0~40mm (0~15mm)
        offset_x = 32 * np.cos(np.radians((-transformed_euler.z + 90)))   ###  grasp at the position of x axis + 40mm  (T joint pipe)
        offset_y = 32 - 32 * np.sin(np.radians((-transformed_euler.z + 90)))   ##   realsense   https://github.com/IntelRealSense/realsense-ros
        offset_z = -finger  ##  Gripper size 160~170mm
        #offset_yaw = 90
        #rospy.loginfo(f"{np.cos(np.radians(transformed_euler.z + 90))}")
        #rospy.loginfo(f"{np.sin(np.radians(transformed_euler.z + 90))}")
        #target_pose = [52+offset_x, 2-offset_y, 375.5-offset_z, 0, 0, 0]  ### position_test1
        #target_pose = [52+offset_x, 2+offset_y, 170+offset_z, 0, 0, 0]  ### position_test2
        #target_pose = [44.2+offset_x, -4+offset_y, 350+offset_z, transformed_euler.x, transformed_euler.y, transformed_euler.z + 90]    ### rotaition_test1
        target_pose = [transformed_pose.position.x +offset_x, transformed_pose.position.y+offset_y, transformed_pose.position.z+offset_z, transformed_euler.x, transformed_euler.y, transformed_euler.z]  # いらない
        target_pose = Pose()
        
        target_pose.position.x = transformed_pose.position.x + (offset_x / 1000)
        target_pose.position.y = transformed_pose.position.y + (offset_y / 1000)
        target_pose.position.z = transformed_pose.position.z + (offset_z / 1000)
        ##############################################################################
        ##############################################################################
        # transformed_pose.orientationをリストに変換
        transformed_orientation = [transformed_pose.orientation.x,
                                transformed_pose.orientation.y,
                                transformed_pose.orientation.z,
                                transformed_pose.orientation.w]
        
        rotation_quaternion = tf.transformations.quaternion_from_euler(0, 0, np.radians(90)) 
        new_orientation = quaternion_multiply(transformed_orientation, rotation_quaternion)
        
        # 新しいorientationをtarget_poseに設定
        target_pose.orientation.x = new_orientation[0]
        target_pose.orientation.y = new_orientation[1]
        target_pose.orientation.z = new_orientation[2]
        target_pose.orientation.w = new_orientation[3]
        ##############################################################################
        ##############################################################################

        rospy.loginfo(f"target_pose: {target_pose}")

        # Open the gripper
        self.gripper.open()

        ##############################################################################
        ##############################################################################
        # Publish the grasp transform
        self.publish_transform(target_pose, "world", "grasp")
        ##############################################################################
        ##############################################################################

        # Move to the target position
        print("=======================")
        self.xarm.set_start_state_to_current_state()
        self.xarm.set_pose_target(target_pose)
        success, plan, _, _ = self.xarm.plan()
        print(self.xarm.get_planning_frame())
        if success:
            self.xarm.execute(plan)
        else:
            print("Planning failed.")

        eef_pose = self.xarm.get_current_pose().pose  # MoveItからのPose  (world座標系)
        base_frame = self.xarm.get_planning_frame()  # MoveItの基準座標系
        rospy.loginfo(f"MoveIt Planning Frame: {base_frame}")
        ##############################################################################
        ##############################################################################
        self.publish_transform(eef_pose, "world", "eef_pose")
        ##############################################################################
        ##############################################################################

        # TFを使ってEEFのPoseを取得
        eef_link = self.xarm.get_end_effector_link()  # EEFのリンク名
        eef_tf = self.tf_buffer.lookup_transform(base_frame, eef_link, rospy.Time(0), rospy.Duration(1.0))
        rospy.loginfo(f"TF Pose in {base_frame}: {eef_tf.transform}")
        
        #rospy.sleep(1)
        # self.xarm.clear_pose_targets()
        rospy.loginfo(f"target_pose:{target_pose}")
        print("=======================")

        self.gripper.close()

        

        
        return 'success'
        # try:
        #     # プランニング
        #     self.xarm.set_goal_joint_tolerance(0.01)  # Increase the goal tolerance for joint position
        #     success_plan, plan, _, _ = self.xarm.plan()
            
        #     if success_plan:
        #         rospy.loginfo('Planning succeeded, executing plan')
        #         success_execute = self.xarm.execute(plan)
            
        #         if success_execute is True:
        #             rospy.loginfo('Grasping')
        #             self.gripper_control(0)
        #             return 'success'
        #         else:
        #             return 'failure'
        #     else:
        #         print("Planning failed.")
        #         if self.try_count < 3:
        #             self.try_count += 1
        #             return 'loop'
        #         return 'failure'
        
        # except Exception as e:
        #     print(e)
        #     return 'failure'
        

class PICK_BACK(smach.State):
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
                rospy.loginfo("Picking work Successfully")
                return "success"
            else:
                print("Execution failed.")
                return "loop"
        except Exception as e:
            print(f"Error in execute: {e}")
            return "loop"
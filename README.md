# Xarm6_pick_and_place_pkg
### プログラム変更点（把持姿勢（TF）の定義）
pick_work.py
```bash
class PickWork(smach.State):
    def __init__(self, outcomes):
        # Declare input_keys and output_keys
        smach.State.__init__(self, outcomes=outcomes, input_keys=['start_time', 'plan_time', 'plan_size'], output_keys=['start_time', 'plan_time', 'plan_size'])
        self.robot = RobotCommander()
        self.xarm = MoveGroupCommander("xarm6")
        self.gripper = GraspControl()
        self.compute_ik = ComputeIK()

        self.try_count = 0
        # self.goal_joint_angles = rospy.get_param("~Joint")
        self.subscriber = None  # Initialize the subscriber as None
        self.msg = None
        self.target_pose = None
        self.tf_buffer = Buffer()

        self.frame_id = None

        # parameters
        self.params = rospy.get_param("~Params")
        self.pipeline = rospy.get_param("pipeline", "stomp")

        # Publisher
        self.enable_pathseed_pub = rospy.Publisher("pathseed_control", Bool, queue_size=10)
        rate = rospy.Rate(50)

        # Subscriber
        self.subscriber = rospy.Subscriber("/execute_trajectory/goal", ExecuteTrajectoryActionGoal, self.execute_trajectory_callback)

        #TFブロードキャスト
        self.br = tf2_ros.StaticTransformBroadcaster()
        self.tf_subscriber = rospy.Subscriber("/tf", TFMessage, self.tf_static_callback, queue_size=10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_buffer.clear()

    def execute_trajectory_callback(self, msg):
        # Callback to handle the subscribed data
        rospy.loginfo("Received trajectory goal data")
        # if self.phase == "Initial_Phase":
        #     # write the execute_trajectory callback in the traj_pick.txt file
        #     with open(os.path.join(package_path, 'pathseeds', 'update_pathseeds', 'trajectories', 'traj_pick.txt'), 'w') as file:
        #         file.write(str(msg))
        #     rospy.loginfo("Trajectory data written to traj_pick.txt")
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
            for param_name in phase.keys():
                if param_name == 'stddev':
                    noise_generator_params = [
                        {
                            'class': 'stomp_moveit/NormalDistributionSampling',
                            'stddev': phase['stddev']
                        }
                    ]
                    # rosparamに設定
                    rospy.set_param('/move_group/stomp/xarm6/task/noise_generator', noise_generator_params)
                else:
                    rospy.set_param(f"move_group/stomp/xarm6/optimization/{param_name}", phase[param_name])
            return phase
        except Exception as e:
            rospy.logerr(f"Error occurred: {e}")
            return None

    def tf_static_callback(self, msg):
        # TFメッセージを受信した際に呼ばれる
        # rospy.loginfo("Received TF static message")
        self.tf_buffer.clear()
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
                # rospy.loginfo(f"TF: {self.target_pose}")

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
    
    def compute_grasp_pose(self, source_pose, source_frame):
        """
        Compute the grasp pose by translating the source_pose +35mm along its x-axis.

        :param source_pose: The Pose to be transformed (geometry_msgs.msg.Pose)
        :param source_frame: The source frame ID (string)
        :return: The computed grasp Pose (geometry_msgs.msg.Pose)
        """
        # Transform source_pose to world frame
        transformed_pose = self.transform_pose(source_pose, source_frame, "posture_world")
        if transformed_pose is None:
            rospy.logerr("Failed to transform pose to world frame.")
            return None

        # そのままの位置
        t = transformed_pose.position

        # ワールド座標系で y 軸方向に 35mm 平行移動
        grasp_pose = Pose()
        grasp_pose.position.y = 40.0/1000

        return grasp_pose

    def execute(self, userdata):
        # init
        self.try_count = 0
        self.msg = None
        rospy.sleep(1)

        # settings
        # xarmの速度と加速度を設定
        # self.xarm.set_max_velocity_scaling_factor(0.5)  # 50% の速度
        # self.xarm.set_max_acceleration_scaling_factor(0.25)  # 25% の加速度
        env = rospy.get_param("env", "task1")

        print("------------------------------------")
        print("Executing PickWork")
        # 姿勢推定によるtargetのPose取得 (Recognition)
        ################################################################################################################################
        # Initialize the posture estimation flag
        ### False :: 認識を行う
        ### True :: 認識を行わない
        # Wait for the posture of object transform for 10 seconds
        self.frame_id = None
        threading.Thread(target=rospy.spin).start()
        rospy.set_param('/posture_estimation_done', False)
        start_time = rospy.Time.now()
        
        key = ''  # 初期値を設定
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            if key == '\n':  # Enterキーが押された
                rospy.loginfo("Enter key detected. Capturing TF data...")
        else:
            return 'loop'
            
        # Set the posture estimation flag to True
        rospy.set_param('/posture_estimation_done', True)
        rospy.loginfo("Received 'Posture_of_object' transform.")
        #rospy.loginfo(f"target_pose: {self.target_pose}")
        source_pose = self.target_pose

        ##############################################################################
        ##############################################################################
        # world座標系 from camera_depth_optical_frame
        transformed_pose = self.transform_pose(source_pose, "camera_depth_optical_frame", "world")
        rospy.loginfo(f"transformed_pose: {transformed_pose}")
        ##############################################################################
        ##############################################################################

        transformed_euler = self.quaternion_to_euler(transformed_pose.orientation)
        source_euler = self.quaternion_to_euler(source_pose.orientation)

        rospy.loginfo(f"transformed_pose: {transformed_euler.x,transformed_euler.y,transformed_euler.z}")
        rospy.loginfo(f"source_pose: {source_euler.x,source_euler.y,source_euler.z}")
        rospy.loginfo(f"{transformed_euler.z + 90}")
        self.publish_transform(transformed_pose, "world", "posture_world")

        # Transform posture_world to grasp_pose
        grasp_pose = self.compute_grasp_pose(transformed_pose, "posture_world")
        grasp_pose = self.transform_pose(grasp_pose, "posture_world", "world")
        if grasp_pose:
            self.publish_transform(grasp_pose, "world", "grasp_pose")
            rospy.loginfo("Published grasp_pose transform.")
        else:
            rospy.logerr("Failed to compute grasp_pose.")

        # Move to Target Position with offset
        ################################################################################################################################
        finger = 19  ###   finger size 0~40mm (0~15mm)
        base_coordinate_z = 90    ###  world座標系とeefの座標系には90度のずれがある
        offset_y = 15  ##   realsense   https://github.com/IntelRealSense/realsense-ros
        offset_z = -finger  ##  Gripper size 160~170mm 
        target_pose = Pose()
        target_pose.position.x = grasp_pose.position.x
        target_pose.position.y = grasp_pose.position.y + (offset_y / 1000)
        target_pose.position.z = grasp_pose.position.z + (offset_z / 1000)
        ##############################################################################
        ##############################################################################
        # transformed_pose.orientationをリストに変換
        transformed_orientation = [transformed_pose.orientation.x,
                                transformed_pose.orientation.y,
                                transformed_pose.orientation.z,
                                transformed_pose.orientation.w]
        
        rotation_quaternion = tf.transformations.quaternion_from_euler(0, 0, np.radians(base_coordinate_z)) 
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
        # get the current phase
        self.phase = rospy.get_param("phase", "Initial_Phase")
        rospy.loginfo(f"Current phase: {self.phase}")

        # PRmethodを使用する場合
        print(f"Use Pathseed: {rospy.get_param('use_pathseed', False)}")
        print(f"Pipeline: {self.pipeline}")
        print(f"Phase: {self.phase}")
        if rospy.get_param("use_pathseed", False) is True and self.pipeline == "stomp":
            print("|====================================|")
            print("|========= Using PR method ==========|") 
            print("|====================================|")
            # publish the pathseed
            rospy.loginfo("Publishing pathseed")
            self.enable_pathseed_pub.publish(Bool(True))

            rospy.set_param("is_publish_pathseed", False)
            if self.phase == "Initial_Phase":
                # specify the template pathseed file
                try:
                    file_path = os.path.join(package_path, 'pathseeds', 'task1', 'pathseed1.txt')
                except FileNotFoundError:
                    rospy.logerr("File not found")
                    self.try_count = 0
                    return 'failure'
                # set Stomp parameters
                stomp_params = self.params[env]["StompParams"]['PickPoint'][self.phase]
                self.set_stomp_params(stomp_params)
                rospy.loginfo(f"Stomp parameters set: {stomp_params}")
            elif self.phase == "Implement_Phase":
                try:
                    # specify the update pathseed file
                    file_path = os.path.join(package_path, 'pathseeds', 'update_pathseeds', 'pathseed_pick.txt')
                except FileNotFoundError:
                    rospy.logerr("File not found")
                    self.try_count = 0
                    return 'failure'
                # set Stomp parameters
                stomp_params = self.params[env]["StompParams"]['PickPoint'][self.phase]
                self.set_stomp_params(stomp_params)
                rospy.loginfo(f"Stomp parameters set: {stomp_params}")
            else:
                print(f"Phase: {self.phase} is not recognized.")
                return 'failure'
            joint_values = self.compute_ik.inverse_kinematics(
                x=target_pose.position.x, y=target_pose.position.y, z=target_pose.position.z, 
                wx=target_pose.orientation.x, wy=target_pose.orientation.y, wz=target_pose.orientation.z, ww=target_pose.orientation.w
            )
            if joint_values is None:
                rospy.logerr("Failed to compute IK.")
                return 'failure'
            goal_joint_values = joint_values
            start_joint_values = self.xarm.get_current_joint_values()
            # decode the pathseed file
            decoder = Decoder()
            print(f"File path: {file_path}")
            print(f"Start joint values: {start_joint_values}")
            print(f"Goal joint values: {goal_joint_values}")
            generated_path = decoder.generate_path(file_path, start_joint_values, goal_joint_values)
            print(f"Generated path: {generated_path}")
            # specify the pathseed file
            pathseed_params = rospy.set_param('/pathseed_param', {
                'path_data': generated_path,
                'reverse': False  # デフォルト値
            })

            # set the target joint values
            self.xarm.set_start_state_to_current_state()
            self.xarm.set_joint_value_target(joint_values)
        # default STOMPの場合
        elif self.pipeline == "stomp":
            print("|====================================|")
            print("|========= Using STOMP method =======|")
            print("|====================================|")

            # set the Stomp parameters (defaultSTOMP: always Initial_Phase parameters)
            stomp_params = self.params[env]["StompParams"]['PickPoint']["Initial_Phase"]
            self.set_stomp_params(stomp_params)
            rospy.loginfo(f"Stomp parameters: {stomp_params}")

            # set the target pose
            self.xarm.set_start_state_to_current_state()
            self.xarm.set_pose_target(target_pose)
        # OMPL (RRT-connect)の場合
        else:
            print("|====================================|")
            print("|========= Using OMPL method =========|")
            print("|====================================|")
            self.xarm.set_start_state_to_current_state()
            self.xarm.set_pose_target(target_pose)

        rospy.loginfo("Planning...")
        # プランニング
        try:
            success, plan, _, _ = self.xarm.plan()
            print(self.xarm.get_planning_frame())
            if success:
                success_exe = self.xarm.execute(plan)
                if success_exe:
                    # create the directories if they do not exist
                    if not os.path.exists(os.path.join(package_path, 'pathseeds', 'update_pathseeds', 'trajectories')):
                        os.makedirs(os.path.join(package_path, 'pathseeds', 'update_pathseeds', 'trajectories'), exist_ok=True)
                    with open(os.path.join(package_path, 'pathseeds', 'update_pathseeds', 'trajectories', 'traj_pick.txt'), 'w') as file:
                        file.write(str(self.msg))
                    rospy.loginfo("Trajectory data written to traj_pick.txt")
            else:
                print("Planning failed.")
                if self.try_count < 3:
                    self.try_count += 1
                    return 'loop'
                return 'failure'
        except Exception as e:
            print(e)
            return 'failure'
        
        # debug
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

        # グリッパーを閉じる
        self.gripper.close()

        return 'success'
```

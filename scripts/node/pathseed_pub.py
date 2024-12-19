#!/usr/bin/env python3

import rospy
from xarm6_pick_and_place_pkg.msg import PathSeed
from std_msgs.msg import Bool
import numpy as np
import os
import rospkg

# rospkgを使ってパッケージのパスを取得
rospack = rospkg.RosPack()
package_path = rospack.get_path('xarm6_pick_and_place_pkg')  # パッケージ名を指定

class PathSeedPublisher:
    def __init__(self):
        self.pub = rospy.Publisher('move_group/path_seed', PathSeed, queue_size=100000)
        self.is_publishing = False  # 初期状態では送信しない
        rospy.Subscriber('pathseed_control', Bool, self.control_callback)  # ON/OFF制御
        self.rate = rospy.Rate(50)  # 50 Hz
        self.count = 0

    def read_data_from_file(self, pathseed_params):
        """Read data from the specified file and return it as a NumPy array."""
        # Read the file path from the parameter server
        path_data = pathseed_params["path_data"]
        reverse = pathseed_params["reverse"]
        if path_data == "":
            rospy.logerr("File path is empty")
            return np.array([])

        data = []
        try:
            lines = path_data.strip().split("\n")
            for line in lines:
                line_data = line.strip().rstrip(",").rstrip(";").split(",")
                for value in line_data:
                    value = value.strip()
                    if value:
                        try:
                            data.append(float(value))
                        except ValueError:
                            rospy.logwarn("Could not convert value to float: '%s'", value)
            if pathseed_params["reverse"] is True:
                # 逆順にしたいとき
                data_reverse = []
                while len(data) > 0:
                    data_reverse += data[-6:]
                    data = data[:-6]
                data = data_reverse
                print("data_reverse: ", data)
        except IOError as e:
            rospy.logerr("Failed to read file: '%s'. Error: %s", pathseed_params["filename"], e)

        return np.array(data)


    def control_callback(self, msg):
        """Callback function for the pathseed_control topic."""
        self.is_publishing = msg.data
        rospy.loginfo("PathSeed publishing is %s", "enabled" if self.is_publishing else "disabled")

    def publish_pathseed(self):
        """Continuously publish PathSeed messages."""
        
        
        # Initialize file path and read initial data
        # filename = rospy.get_param('/pathseed_file', os.path.join(package_path, 'pathseeds', 'pathseed1.txt'))
        pathseed_params = rospy.get_param('/pathseed_param', {
            'path_data': "",
            'reverse': False
        })

        data = self.read_data_from_file(pathseed_params)
        num_cols = 6
        if len(data) % num_cols != 0:
            rospy.logerr("Initial data length is not divisible by the number of columns")
            return
        num_rows = len(data) // num_cols

        # rospy.loginfo("Publishing PathSeed message #%d", count)
        matrix_msg = PathSeed()
        matrix_msg.rows = num_rows
        matrix_msg.cols = num_cols
        matrix_msg.data = data.tolist()

        rospy.loginfo("  Rows: %d", matrix_msg.rows)
        rospy.loginfo("  Cols: %d", matrix_msg.cols)
        rospy.loginfo("  First row of data: %s", str(data[:num_cols]))
        rospy.loginfo("  Total number of data elements: %d", len(data))
        self.count += 1
        # rospy.loginfo("Published PathSeed message #%d", self.count)
        self.pub.publish(matrix_msg)
        self.rate.sleep()

    def run(self):
        """Run the main loop."""
        while not rospy.is_shutdown():
            if self.is_publishing:
                self.publish_pathseed()
            else:
                self.rate.sleep()

if __name__ == '__main__':
    # ROSノードを初期化
    rospy.init_node('pathseed_publisher', anonymous=True)  # ノード名を指定
    pub = PathSeedPublisher()
    pub.run()

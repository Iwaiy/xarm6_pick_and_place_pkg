import os
import ast
import datetime
import numpy as np
import rospy
import rospkg


class Decoder:
    def __init__(self):
        """
        Decodeクラスの初期化
        directory: データファイルが存在するディレクトリ
        """
        # self.directory = directory
        self.pathseed_data = None

    def load_data(self, pathseed_file):
        """
        データの読み込み
        Args:
            pathseed_file: 読み込むファイル
        Returns:
            pathseed_data: 読み込んだデータ
        """
        with open(pathseed_file, "r") as f:
            pathseed_data = f.read()
        return pathseed_data
    
    def normalize_vector(self, v): # ベクトルを正規化する関数
        '''
        ベクトルを正規化する関数
        Args:
            v: 正規化したいベクトル
        Returns:
            v / norm: 正規化されたベクトル
        '''
        norm = np.linalg.norm(v)
        if norm == 0:
            return v
        return v / norm
    
    def cross_three_dim(self, vector1, vector2): # 3次元の外積演算を行う関数
        '''
        3次元の外積演算を行う関数
        Args:
            vector1: ベクトル1
            vector2: ベクトル2
        Returns:
            result: 外積の結果
        '''
        result = np.array([
            [vector1[1]*vector2[2] - vector1[2]*vector2[1]],
            [vector1[2]*vector2[0] - vector1[0]*vector2[2]],
            [vector1[0]*vector2[1] - vector1[1]*vector2[0]]
        ])
        return result
    
    def convert_data(self, matrix): # decordにおけるファイル読み込みを容易にするためのに変換する関数
        '''
        decordにおけるファイル読み込みを容易にするためのに変換する関数
        Args:
            matrix: 変換したい行列
        Returns:
            output: 変換された行列
        '''
        list = matrix.tolist()
        str_list = ['[' + ', '.join(map(str, sublist)) + ']' for sublist in list]
        output = '\n'.join(str_list)
        return output
    
    def formated_data(self, data):
        """
        データをフォーマットする関数
        Args:
            data: フォーマットしたいデータ
        Returns:
            formated_data: フォーマットされたデータ
        """
        formatted_data = data.replace('[', '').replace(']', ',')[:-1]
        formatted_data = formatted_data.rstrip(',') + ';'
        return formatted_data

    def decode(self, pathseed_data, start_joint_values, goal_joint_values):
        """
        データのデコード処理を行う関数
        Args:
            pathseed_data: デコードしたいデータ
        """
        # データの読み込み
        w_num = pathseed_data['w_num']
        r1_array = np.array(pathseed_data['r1_array'])  # NumPy配列に再変換
        r2_array = np.array(pathseed_data['r2_array'])  # NumPy配列に再変換
        tau1_array = np.array(pathseed_data['tau1_array'])  # NumPy配列に再変換
        tau2_array = np.array(pathseed_data['tau2_array'])  # NumPy配列に再変換
        delta_array = np.array(pathseed_data['delta_array'])  # NumPy配列に再変換
        
        n = delta_array.shape[1] // 2  # delta_arrayの半分の列数を取得
        # スライスしてdelta1_arrayとdelta2_arrayを取り出す
        delta1_array = delta_array[:, :n]  # 最初の半分の列がdelta1_array
        delta2_array = delta_array[:, n:]  # 後ろの半分の列がdelta2_array

        s_new = start_joint_values
        g_new = goal_joint_values
        rospy.loginfo(f"s_new: {s_new}")
        rospy.loginfo(f"g_new: {g_new}")
        s = np.array(s_new).reshape(-1,1)
        g = np.array(g_new).reshape(-1,1)
        s1,s2 = np.split(s, 2, axis=0)
        g1,g2 = np.split(g, 2, axis=0)
        v_array = g - s
        v1_array = g1 - s1
        v2_array = g2 - s2
        e_array = np.array(self.normalize_vector(v_array))
        e1 = np.array(self.normalize_vector(v1_array))
        e2 = np.array(self.normalize_vector(v2_array))

        r1_array = np.squeeze(r1_array) # 不要な次元を削除
        r2_array = np.squeeze(r2_array)
        tau1_array = np.squeeze(tau1_array)
        tau2_array = np.squeeze(tau2_array)
        delta_array = np.squeeze(delta_array)

        l3_array = np.empty((3,0), float)
        l4_array = np.empty((3,0), float)
        for i in range(w_num):
            l3 = s1 + (r1_array[i] * v1_array)
            l4 = s2 + (r2_array[i] * v2_array)
            l3_array = np.hstack([l3_array, l3])
            l4_array = np.hstack([l4_array, l4])
        l_array_d = np.concatenate((l3_array, l4_array), axis=0)

        w1_array = np.empty((0,3), float)
        w2_array = np.empty((0,3), float)
        for i in range(w_num):
            cross1 = self.cross_three_dim(delta1_array.T[:,i], e1) # 外積を計算
            cross2 = self.cross_three_dim(delta2_array.T[:,i], e2)
            cross1 = np.squeeze(cross1) # 不要な次元を削除
            cross2 = np.squeeze(cross2)
            w1 = l3_array[:,i] + tau1_array[i] * (cross1)
            w2 = l4_array[:,i] + tau2_array[i] * (cross2)
            w1_array = np.vstack([w1_array, w1])
            w2_array = np.vstack([w2_array, w2])
        w = np.concatenate((w1_array, w2_array), axis=1)
        w_plot_array = w.T
        converted_data = self.convert_data(w)
        # print("w:\n",converted_data) # 完成したウェイポイント

        # waypoint = read_matrix_from_file(waypoint_file)
        # error = waypoint - w # 誤差の計算
        # print("Error for each element:")
        # for row in error:
        #     print(row)
        
        return converted_data
    
    def write_data(self, write_data, write_file):
        """
        データの書き込み
        Args:
            write_data: 書き込むデータ
            write_file: 書き込むファイル
        """
        try:
            # パッケージのパスを取得
            rospack = rospkg.RosPack()
            package_path = rospack.get_path('xarm6_pick_and_place_pkg')
            
            # 相対パスを使ってディレクトリを設定
            directory_name = os.path.join(package_path, 'decoded_data')

            # ディレクトリが存在しない場合は作成
            if not os.path.exists(directory_name):
                rospy.loginfo(f"Creating directory: {directory_name}")
                os.makedirs(directory_name)

            # 確認: ディレクトリが作成されたかチェック
            if os.path.exists(directory_name):
                rospy.loginfo(f"Directory {directory_name} created successfully.")
            else:
                rospy.logwarn(f"Failed to create directory: {directory_name}")
            
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            write_file = os.path.join(directory_name, write_file + "_" + timestamp + ".txt")
            
            # print("write_data:", write_data)
            # データをファイルに書き込む
            with open(write_file, "w") as f:
                f.write(write_data)
            rospy.loginfo(f"Data has been written to {write_file}")
        except Exception as e:
            rospy.logerr(f"Failed to write data to {write_file}: {e}")
    
    def generate_path(self, pathseed_file, start_joint_values, goal_joint_values):
        """
        デコードのメイン関数
        Args:
            pathseed_file: デコードしたいファイル
        """
        with open(pathseed_file, "r") as f: # ファイルを読み込む
            data = f.read()
            # print("data:", data)
        # 文字列を辞書に変換
        pathseed_data = ast.literal_eval(data)

        rospy.loginfo("Decoding the pathseed file...")
        generate_path = self.decode(pathseed_data, start_joint_values, goal_joint_values)

        generate_path = self.formated_data(generate_path)

        rospy.loginfo("Data has been decoded.")
        # rospy.loginfo(f"Decoded data:\n{generate_path}")
        self.write_data(generate_path, "generated_path") # データをファイルに書き込む
        return generate_path
    

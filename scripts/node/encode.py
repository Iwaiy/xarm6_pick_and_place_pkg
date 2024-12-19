import os
import ast
import numpy as np
import rospy


class Encoder:
    def __init__(self):
        """
        Encodeクラスの初期化
        directory: データファイルが存在するディレクトリ
        """
        # self.directory = directory
        self.pathseed_data = None
    
    def normalize_vector(self, v): # ベクトルを正規化する関数
        '''
        ベクトルを正規化する関数
        Args:
            v: 正規化したいベクトル
        Returns:
            v / norm: 正規化されたベクトル
        '''
        norm = np.linalg.norm(v)
        if norm == 0: # ノルムがゼロのときは正規化できない
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
        print(f"matrix: {matrix}, output: {output}")
        return output

    def encode(self, waypoint_file):
        """
        データのエンコード処理を行う関数
        Arg: waypoint_file: ウェイポイントのファイル
        Return: pathseed: パスシード
        """
        with open(waypoint_file, "r") as f: # ファイルを読み込む
            lines = f.readlines() # ファイルの各行をリストに格納
            
            # sとgの初期点とゴール点を取得する
            s = np.array(ast.literal_eval(lines[0])).reshape(-1,1)  # sをファイルの最初の行から取得
            g = np.array(ast.literal_eval(lines[-1])).reshape(-1,1) # gをファイルの最後の行から取得
            
            # sとgを2つのベクトル（s1, s2 / g1, g2）に分割
            s1,s2 = np.split(s, 2, axis=0)
            g1,g2 = np.split(g, 2, axis=0)

            # ウェイポイント数を計算（sとgを除いた数）
            w_num = len(lines) - 2

            # sとgの差ベクトルを計算し、正規化
            v_array = g - s
            v1_array = g1 - s1
            v2_array = g2 - s2
            e_array = np.array(self.normalize_vector(v_array))
            e1 = np.array(self.normalize_vector(v1_array))
            e2 = np.array(self.normalize_vector(v2_array))

            # ウェイポイントwの配列を初期化し、データを格納
            w_array = np.empty((6,0), float)
            for i in range(1, len(lines) - 1):
                w = np.array(ast.literal_eval(lines[i])).reshape(-1,1) # 各ウェイポイントを格納
                w_array = np.hstack([w_array, w])

            # w1, w2に分割
            w1, w2 = np.split(w_array, 2, axis=0)

            # 各l1, l2を計算
            l1_array = np.empty((3,0), float)
            l2_array = np.empty((3,0), float)
            for i in range(w_num):
                l1 = s1 + np.dot((w_array[:3,i].reshape(-1,1) - s1).T, e1) * e1
                l2 = s2 + np.dot((w_array[3:6,i].reshape(-1,1) - s2).T, e2) * e2
                l1_array = np.hstack([l1_array, l1])
                l2_array = np.hstack([l2_array, l2])
            
            # l1とl2を連結
            l_array = np.concatenate((l1_array, l2_array), axis=0)

            # 1. 各ウェイポイントに対して正規化と外積を使用し，相対位置を計算
            r1_array = np.empty((1,0), float)
            r2_array = np.empty((1,0), float)
            for i in range(w_num):
                r1 = ((np.linalg.norm(l1_array[:,i] - s1.T))/((np.linalg.norm(v1_array)))).reshape(-1,1)
                r1_array = np.hstack([r1_array, r1])
                r2 = ((np.linalg.norm(l2_array[:,i] - s2.T))/((np.linalg.norm(v2_array)))).reshape(-1,1)
                r2_array = np.hstack([r2_array, r2])

            # 2. タウ値を計算
            tau1_array = np.empty((1,0), float)
            tau2_array = np.empty((1,0), float)
            for i in range(w_num):
                tau1 = np.linalg.norm(w1[:,i] - l1_array[:,i]).reshape(-1,1)
                tau2 = np.linalg.norm(w2[:,i] - l2_array[:,i]).reshape(-1,1)
                tau1_array = np.hstack([tau1_array, tau1])
                tau2_array = np.hstack([tau2_array, tau2])
            tau_array = np.concatenate((tau1_array, tau2_array), axis=0)

            # 3. 外積と正規化を使ってデルタ値を計算
            delta1_array = np.empty((0,3), float)
            delta2_array = np.empty((0,3), float)
            wl1 = w1 - l1_array
            wl2 = w2 - l2_array
            for i in range(w_num):
                delta1 = self.cross_three_dim(e1, wl1[:,i])
                delta2 = self.cross_three_dim(e2, wl2[:,i])
                delta1 = np.squeeze(delta1)
                delta2 = np.squeeze(delta2)
                normlaize_delta1 = np.array(self.normalize_vector(delta1))
                normlaize_delta2 = np.array(self.normalize_vector(delta2))
                delta1_array = np.vstack([delta1_array, normlaize_delta1])
                delta2_array = np.vstack([delta2_array, normlaize_delta2])
            
            # 結果を連結
            delta_array = np.concatenate((delta1_array, delta2_array), axis=1)
            delta_plot_array = delta_array.T

            # パスシード情報を格納
            self.pathseed_data = {
                # 'waypoint_file': waypoint_file,
                'w_num': w_num,
                # 'l1': l1,
                # 'l2': l2,
                # 'l1_array': l1_array,
                # 'l2_array': l2_array,
                'r1_array': r1_array,
                'r2_array': r2_array,
                'tau1_array': tau1_array,
                'tau2_array': tau2_array,
                # 'delta1_array': delta1_array,
                # 'delta2_array': delta2_array,
                'delta_array': delta_array,
            }

            return self.pathseed_data
        
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
                pathseed_data['r1_array'] = pathseed_data['r1_array'].tolist()
                pathseed_data['r2_array'] = pathseed_data['r2_array'].tolist()
                pathseed_data['tau1_array'] = pathseed_data['tau1_array'].tolist()
                pathseed_data['tau2_array'] = pathseed_data['tau2_array'].tolist()
                pathseed_data['delta_array'] = pathseed_data['delta_array'].tolist()
                file.write(str(pathseed_data))

            print(f"データが {file_name} に書き込まれました。")
        except IOError as e:
            print(f"ファイルの書き込み中にエラーが発生しました: {e}")

    def generate_pathseed(self, waypoint_file=None, directory=None) -> dict:
        """
        パスシードを生成する関数
        Args:
            waypoint_file (str): 読み込むウェイポイントファイル
            directory (str): ウェイポイントファイルが存在するディレクトリ
        Returns:
            dict: パスシードデータ
        """ 
        # waypoint_file = os.path.join(directory, "angle.txt")
        rospy.loginfo(f"Generating pathseed from {directory}/{waypoint_file}")

        pathseed_data = None  # デフォルト値を設定
        
        try:
            pathseed_data = self.encode(waypoint_file)
            # print(pathseed_data)
        except FileNotFoundError:
            rospy.logerr("File not found")

        if pathseed_data is not None:
            # # エンコード結果をファイルに書き込む
            # file_name = os.path.join(directory, "pathseed.txt")
            # NumPy配列をリストに変換して保存
            pathseed_data['r1_array'] = pathseed_data['r1_array'].tolist()
            pathseed_data['r2_array'] = pathseed_data['r2_array'].tolist()
            pathseed_data['tau1_array'] = pathseed_data['tau1_array'].tolist()
            pathseed_data['tau2_array'] = pathseed_data['tau2_array'].tolist()
            pathseed_data['delta_array'] = pathseed_data['delta_array'].tolist()

            # self.save_pathseed(pathseed_data, file_name)

            return pathseed_data
        else:
            print("パスシードの生成に失敗しました。")
            return None
        
# debug
if __name__ == "__main__":
    directory = os.path.dirname(os.path.abspath(__file__))
    encoder = Encoder()
    pathseed_data = encoder.generate_pathseed("angle.txt", directory)
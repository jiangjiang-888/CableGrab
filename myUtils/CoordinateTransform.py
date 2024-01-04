import cv2
import numpy as np
import transforms3d as tfs
#相机坐标系转换为机器人基坐标系
import math

class CoordinateTransform:
    def __init__(self):
        self.x, self.y, self.z = -0.125796, 0.0305877, 0.546227  # 平移
        self.rx, self.ry, self.rz = -3.09545, -0.0643993, -2.8378  # 旋转
        self.qw, self.qx, self.qy, self.qz =0.02832577970432306, 0.15192925503660906, -0.9875980375216189, 0.027663476853261607   #四元数
        
    def get_matrix_eular_radu(self):
        mat = tfs.euler.euler2mat(self.rx, self.ry, self.rz)
        mat = tfs.affines.compose(np.squeeze(np.asarray((self.x, self.y, self.z))), mat, [1, 1, 1])
        return mat
    
    def rot2quat_minimal(m):    
        quat =  tfs.quaternions.mat2quat(m[0:3,0:3])    
        return  quat

    def handXYZ2EyeXYZ(self, hand_x, hand_y, hand_z):
        mat = self.get_matrix_eular_radu()
        handMat = [hand_x, hand_y, hand_z, 1]
        eye_x, eye_y, eye_z, _ = np.dot(mat, handMat)
        return eye_x, eye_y, eye_z

    def eyeXYZ2HandXYZ(self, eye_x, eye_y, eye_z):
        mat = self.get_matrix_eular_radu()
        rmat = np.linalg.inv(mat)
        eyeMat = [eye_x, eye_y, eye_z, 1]
        hand_x, hand_y, hand_z, _ = np.dot(rmat, eyeMat)
        return hand_x, hand_y, hand_z



if __name__ == "__main__":

    # hand = [-0.09214994723594513, -0.02118715313623601, 0.08244428882092639]
    # eye= [-0.058688387274742126, -0.023314662277698517, 0.4467763602733612]
    eye= [0, 0, 0.43447]
    coordinate_transform = CoordinateTransform()
    # print(coordinate_transform.handXYZ2EyeXYZ(*hand))
    print(coordinate_transform.eyeXYZ2HandXYZ(*eye))
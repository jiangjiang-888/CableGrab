import cv2
import numpy as np
import transforms3d as tfs
#相机坐标系转换为机器人基坐标系
import math

class CoordinateTransform:
    def __init__(self):
        self.x, self.y, self.z = -0.165  ,  -0.05 , 0.584369  # 平移
        self.rx, self.ry, self.rz = 3.14 ,  0  , 0   # 旋转
        
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
    hand = [0.1896, -0.046200000000000005, 0.0723]
    eye= [0.05342663764056446,-0.012832965467842449,0.5143761178699267]
    coordinate_transform = CoordinateTransform()
    print(coordinate_transform.handXYZ2EyeXYZ(*hand))
    print(coordinate_transform.eyeXYZ2HandXYZ(*eye))
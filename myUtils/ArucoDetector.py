#使用opencv自带的aruco
# Python311 + opencv4.8 +numpy 1.24.3
import numpy as np
import time
import cv2
import cv2.aruco as aruco
import math
import sys
import threading
import cv2
import uuid

class ArucoDetector:
    def __init__(self,marketLength=0.030, dictionary = aruco.DICT_ARUCO_ORIGINAL):
        #一般不用修改，适配1920*1080的相机 
        self.dist = np.array(([[0.019367, -0.052131, 0.003435, -0.000312, 0.000000]]))
        self.mtx = np.array([[1136.6399, 0., 959.06707],
                             [0., 1136.88727, 539.12713],
                             [0., 0., 1.]])
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.marketLength = marketLength
        self.aruco_dict = aruco.getPredefinedDictionary(dictionary)
        self.parameters = aruco.DetectorParameters()
        self.res_dict = {}
        self.ori=None
        self.frame=None

    #比如传入摄像头读取的结果，不需要作任何变换
    def detectByImage(self,frame):
        frame,self.result_dict=  self.detect(frame)
        return  frame, self.result_dict

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            print(self.getWorldCoords(x,y,0.595))
            print(f"Mouse clicked at ({x}, {y})")

    def camera_thread_function(self , camera_id=1):
        cap = cv2.VideoCapture(camera_id)
        
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        while True:
            ret, self.ori = cap.read()
            if not ret:
                print("Failed to capture frame from camera.")
                break

            # 在这里添加你的图像处理代码，例如显示图像
            self.frame, self.res_dict= self.detect(self.ori)
            
            cv2.imshow("ori",self.ori )
            cv2.setMouseCallback("ori", self.mouse_callback)
            
            resized_image = cv2.resize(self.frame, (int(1920/2), int(1080/2)))
            
            cv2.imshow("res",resized_image )

            key = cv2.waitKey(50) 
            if key == 27  or key == ord('q'):         # 按esc键退出
                print('esc break...')
                break

            if key == ord(' '):   # 按空格键保存
                
                print(self.res_dict )
            
            if key== ord('s'):
                
                cv2.imwrite(str(uuid.uuid4()) + ".jpg", self.ori)
                cv2.imwrite(str(uuid.uuid4()) + ".jpg", self.frame)
                
            if key == ord('0') or key == ord('1') or key == ord('2') or key == ord('3') :
             
                k=key -ord('0')
                if k in self.res_dict:
                    print(self.res_dict[int(key -ord('0'))])
                else:
                    print("找不到该值")

        cap.release()
        cv2.destroyAllWindows()
        
        
    def detectByCamera(self,camera_id):

        camera_thread = threading.Thread(target=self.camera_thread_function)
        camera_thread.start()
                
        # camera_thread.join()

    def getCoordsById(self, dict,id):
        rvec = dict[id]["rvec"]
        tvec = dict[id]["tvec"]
        
        
        return tvec , rvec
    
    
    def getWorldCoords(self, x,y,high):

        # 假设已知的相机外参矩阵（cameraMatrix）和畸变系数（distCoeffs）
        camera_matrix = self.mtx
        dist_coeffs = self.dist

        # 在已经畸变纠正的图像上获取像素坐标
        pixel_coordinates = np.array([[x, y]], dtype=np.float32).copy()

        # 使用cv2.undistortPoints进行畸变纠正
        # undistorted_image_points = pixel_coordinates
        undistorted_image_points = cv2.undistortPoints(pixel_coordinates, camera_matrix, dist_coeffs)
        print(undistorted_image_points)

        # 将像素坐标和高度值转换为相机坐标系中的坐标
        image_points = np.array([undistorted_image_points[0, 0, 0], undistorted_image_points[0, 0, 1], 1])
        camera_coordinates = high * image_points

        return camera_coordinates

        
    # 纠正畸变 
    def undistort_frame(self, frame):
        h1, w1= frame.shape[:2]  

        new_camera_mtx, roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w1,h1), 1, (w1,h1))
        dst1 = cv2.undistort(frame, self.mtx, self.dist, None, new_camera_mtx)
            
        x, y, w1, h1 = roi
        dst1 = dst1[y:y + h1, x:x + w1]
        return dst1
    
    def detect(self,frame):
        frame = self.undistort_frame(frame)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        result_dict= {}

        #使用aruco.detectMarkers()函数可以检测到marker，返回ID和标志板的4个角点坐标
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,self.aruco_dict,parameters=self.parameters)
    
        # 如果找不到id
        if ids is not None:
            #按照id排序
            # sortedData = sorted(zip(ids, corners, rejectedImgPoints), key=lambda x: x[0])
            # ids, corners, rejectedImgPoints=zip(*sortedData )
            # ids=[int(x) for x in ids]
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, self.marketLength, self.mtx, self.dist)
            # 估计每个标记的姿态并返回值rvet和tvec ---不同
            # from camera coeficcients
            (rvec-tvec).any() # get rid of that nasty numpy value array error

            for i in range(rvec.shape[0]):
                cv2.drawFrameAxes(frame, self.mtx, self.dist, rvec[i, :, :], tvec[i, :, :], 0.03)
                rvec[i, :, :]=rvec[i, :, :]  * (180.0 / math.pi)  #弧度制转为角度制
                aruco.drawDetectedMarkers(frame, corners)
            
            for i, marker_id in enumerate(ids):
                result_dict[int(marker_id)] = {
                    # 'corners': corners[i],                        #不需要
                    # 'rejectedImgPoints': rejectedImgPoints[i],    #不需要
                    'rvec': rvec[i, :, :],
                    'tvec': tvec[i, :, :]
                }
            cv2.putText(frame, "Id: " + str(ids), (0,64), self.font, 1, (0,255,0),2,cv2.LINE_AA)
        else:
        ##### DRAW "NO IDS" #####
            cv2.putText(frame, "No Ids", (0,64), self.font, 1, (0,255,0),2,cv2.LINE_AA)
            
        return frame,result_dict
    



if __name__ == "__main__":
    aruco_detection = ArucoDetector(marketLength=0.030)
    
    #windows外置摄像头
    aruco_detection.detectByCamera(1)
    
    #ubuntu
    # aruco_detection.detectByCamera( '/dev/video4')
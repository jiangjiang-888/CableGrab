from myUtils.ArucoDetector import ArucoDetector
from myUtils.MyRobotController import MyRobotController
from myUtils.CoordinateTransform import CoordinateTransform
import cv2
import threading
import time


def thread_func_2(arucoDetector,coordinateTransform,myRobotController): 
    left_aruco_id = 1
    right_aruco_id = 2
    while True:
        time.sleep(0.5)
        if right_aruco_id in arucoDetector.res_dict:
            [ex,ey,ez] = arucoDetector.res_dict[right_aruco_id]["tvec"][0]
            hx,hy,hz = coordinateTransform.eyeXYZ2HandXYZ(ex,ey,ez)
            print("眼部坐标",ex,ey,ez, "手部坐标",hx*1000,hy*1000,hz*1000)
            myRobotController.printBaseCorrds()
            
            

def thread_func_1(aruco_detector):  
    aruco_detector.detectByCamera()
      
if __name__ == '__main__':
    arucoDetector = ArucoDetector()
    
    myRobotController = MyRobotController()
    # time.sleep(3)
    myRobotController.init()
    
    coordinateTransform =  CoordinateTransform()
    
    
    cap = cv2.VideoCapture(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    
    # 创建并启动两个线程
    thread1 = threading.Thread(target=arucoDetector.detectByCamera, args=(cap,))
    thread2 = threading.Thread(target=thread_func_2,args=(arucoDetector,coordinateTransform,myRobotController,))
    
    thread2.daemon = True
    
    # thread2 = threading.Thread(target=coordinateTransform.eyeXYZ2HandXYZ())
    thread1.start()
    thread2.start()
    
    # 等待线程结束
    thread1.join()
    
    # myRobotController.release()
    
    cap.release()
    cv2.destroyAllWindows()
    
    
    
    
    
    
    
    
    
    
    
    











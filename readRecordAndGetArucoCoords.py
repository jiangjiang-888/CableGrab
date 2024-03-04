#record.txt是mybuddy拖动时的轨迹，用来做手眼标定
from myUtils.ArucoDetector import ArucoDetector
from myUtils.MyRobotController import MyRobotController
import cv2
import ast
import time
import xmlrpc.client

import encodings.idna
import encodings.idna

class MyRecorder():
    def __init__(self,record_path='record.txt',   #大象机器人通过拖动生成的轨迹，用angles表示左右手臂的轨迹点，因为标定时aruco码粘贴在右臂上，所以这个文件中只有右臂的角度值发生变化，并且标定时也只使用右臂的angles，发送给右臂，并记录标定点
                 hand_eye_coords_path='hand_eye_record.txt',
                 cam = 1   #win cam 外智摄像头
                 
                 ) -> None:
        
        self.record_path=record_path
        self.hand_eye_coords_path = hand_eye_coords_path

        
        # self.robot = MyRobotController(mode="remote")   #远程连接   获取坐标有点问题，
        
        self.robot = xmlrpc.client.ServerProxy('http://10.3.51.195:15000')  #换成远程调用

        self.arucoDetector =  ArucoDetector(marketLength=0.030)    
        
        self.camera = cam
        
    
    #1、读取轨迹文件中的右臂角度值
    #2、按照一定间隔采样右臂角度并远程发送给机器人
    #3、记录此时的右臂末端坐标值，以及使用aruco来读取码的坐标值，默认都是右手坐标系
    #4、写到文件中
    def sendAngleAndRecord(self):
        with open(self.record_path, 'r') as file:
            content = file.read()
        
        # 将字符串转换为Python对象
        data_array = ast.literal_eval(content)

        # 输出结果
        print("读取的动作个数：", len(data_array))
        
        #将右臂移动到相对安全的位置避免碰撞到
        self.arucoDetector.detectByCamera(self.camera)
        

        self.robot.rightArmInit()
        
        time.sleep(20)
        
        
        # 打开文件，如果文件不存在则创建
        self.file =  open(self.hand_eye_coords_path, mode='a')

        
        for i in range(0,len(data_array),10):
            right_angle=data_array[i][1]  #用来标定的右手坐标
            
            print("轨迹记录点",i,right_angle)
            
            self.robot.sendAngles(2,right_angle,20)    
            time.sleep(15)
            
            
            angles= ', '.join(map(str, self.robot.getAllAngles()[1]))
            coords = self.robot.getAllBaseCoords()[1]
            for j in range(3):
                coords[j] /= 1000

            coords= ', '.join(map(str, coords)) 
            
            print("轨迹记录点angle",angles)
            print("轨迹记录点coord",coords)
            
            if 995 in self.arucoDetector.res_dict:
                tvec_str = ','.join(map(str, self.arucoDetector.res_dict[995]['tvec'][0].flatten()))
                rvec_str = ','.join(map(str, self.arucoDetector.res_dict[995]['rvec'][0].flatten()))

                eye =  tvec_str + ',' + rvec_str
                print("aruco检测结果",eye)
                
                
                self.file.write('hand,'+coords+'\n')
                self.file.write('eye,'+eye+'\n')
                self.file.flush()
            else :
                print("未检测到995标签")
            # break
        


    def  __del__(self):
        self.robot.safeReleaseRight()
        self.camera.camera_thread.join()
        self.file.close()
        




if __name__ == "__main__":
    
    myRecorder=MyRecorder()
    myRecorder.sendAngleAndRecord()




    
    
    
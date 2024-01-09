import time
import math
from pymycobot import MyBuddySocket
from pymycobot import MyBuddy
import functools    
class MyRobotController:
    def __init__(self, mode = 'remote', ip="10.3.51.195",port=9000,serialport="/dev/ttyACM0",baud="115200"):
        self.zeroAngles = [[0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0]]
        #过度位置，避免碰撞到操纵台
        self.tempAngles=[
            [-90, 56, 66, 0, 0, 0],
            [90, -29, -82, 0, 0, 0]
        ]
        self.initAngles=[[-90, 30, 60, 45, -90, -120],[90, -30, -60, -45, -90, 120]]
        self.ip=ip
        self.port =port
        self.serialport=serialport
        self.baud = baud
        self.mode = mode
        
        if mode=='remote':
            self.mc = MyBuddySocket(self.ip,self.port) #双臂机器人ip及端口
            self.mc.connect(self.serialport, self.baud)  
        else :
            self.mc = MyBuddy(self.serialport, self.baud)
            
        self.mc.set_fresh_mode(1,0)
        time.sleep(0.5)
        self.mc.set_fresh_mode(2,0)
        time.sleep(0.5)
    
    def __del__(self):
        self.mc.close()

    def retry_on_failure(retries=3):
        def decorator(func):
            @functools.wraps(func)
            def wrapper(self, *args, **kwargs):
                for _ in range(retries):
                    try:
                        return func(self, *args, **kwargs)
                    except Exception as e:
                        print(f"Function {func.__name__} failed with error: {e}")
                        print("Reconnecting...")
                        self.reConnect()
                raise Exception(f"Function {func.__name__} failed after {retries} retries.")
            return wrapper
        return decorator
    
    def reConnect(self):
        self.mc = MyBuddySocket(self.ip,self.port) #双臂机器人ip及端口
        self.mc.connect(self.serialport, self.baud)   
        self.mc.set_fresh_mode(1,0)
        time.sleep(0.5)
        self.mc.set_fresh_mode(2,0)
        time.sleep(0.5)    
    
    def init(self)   :
        #将机械臂init到预抓取状态
        self.moveByAngles(self.zeroAngles)    #所有关节调到0
        self.moveByAngles(self.tempAngles)   #过渡状态，避免碰撞到板子
        self.moveByAngles(self.initAngles)   #预抓取状态
        
    
    def printBaseCorrds(self):
        print("获取关节位姿...")
        coordL,coordR = self.getAllBaseCoords()
        print("left coords:", coordL)
        print("right coords:", coordR)
        time.sleep(2)
        
    def printAngles(self):
        print("获取关节角度...")
        anglesL,anglesR = self.getAllAngles()
        print("left angles:",anglesL)
        print("right angles:",anglesR)
        time.sleep(2)

    #获取两个臂的所有关节角度
    # @retry_on_failure()
    def getAllAngles(self):
        time.sleep(2)
        anglesL=self.mc.get_angles(1)
        while anglesL is None or len(anglesL)==0:
            time.sleep(1)
            anglesL=self.mc.get_angles(1)
        time.sleep(1)
        anglesR=self.mc.get_angles(2)
        time.sleep(1)
        while anglesR is None or len(anglesR)==0:
            time.sleep(1)
            anglesR=self.mc.get_angles(2)
        print(anglesL,anglesR)
        return [anglesL,anglesR]


    #获取两个臂的所有基坐标位置
    # @retry_on_failure()
    def getAllBaseCoords(self):
        time.sleep(2)
        '''
        coordsL=self.mc.get_base_coord(1)
        while coordsL is None or len(coordsL)==0:
            print(coordsL)
            time.sleep(1)
            coordsL=self.mc.get_base_coord(1)
        print(coordsL)
        time.sleep(1)
        coordsR=self.mc.get_base_coord(2)
        while coordsR is None or  len(coordsR)==0:
            time.sleep(1)
            coordsR=self.mc.get_base_coord(2)
        print(coordsR)
        '''
        coords= self.mc.get_base_coords()
        while coords is None or  len(coords)==0:
            time.sleep(1)
            print(coords)
            coords = self.mc.get_base_coords()
        return coords
        # return [coordsL,coordsR]

    def moveByCoords(self,coords):
        #移动左臂
        self.mc.write_base_coords(1, coords[0], 5)
        time.sleep(0.10)
        #移动右臂
        self.mc.write_base_coords(2, coords[1], 5)
        time.sleep(5)
    
    def moveByAngles(self,angles):
        #移动左臂
        self.mc.send_angles(1, angles[0], 50)
        time.sleep(0.50)
        #移动右臂
        self.mc.send_angles(2, angles[1], 50)
        time.sleep(5)
        

    #absShift=[x1,y1,x2,y2]，用于线缆控制的项目
    def addAbsoluteShift(self,nowCoords,absShift):
        newCoordsL=nowCoords[0]
        newCoordsL[0]+=absShift[0]
        newCoordsL[1]+=absShift[1]

        newCoordsR=nowCoords[1]
        newCoordsR[0]+=absShift[2]
        newCoordsR[1]+=absShift[3]
        
        return [newCoordsL,newCoordsR]


    def moveAbsShift(self,nowCoords,absShift):
        self.mc.write_base_coord(1, 1, nowCoords[0][0]+3, 100)
        time.sleep(0.5)
        self.mc.write_base_coord(2, 1, nowCoords[1][0]-3, 100)
        time.sleep(0.5)
        
        
    ##任务完成以后，将双臂移动到较低的地方，避免释放的时候碰撞
    def release(self):
        #将夹爪张开到最大
        self.mc.set_gripper_value(1, 100,50)
        time.sleep(1)
        self.mc.set_gripper_value(2, 100,50)
        time.sleep(1)
        
        self.moveByAngles([
        [-90, 56, 66, 0, 0, 0],
        [90, -29, -82, 0, 0, 0]
            ])
        self.moveByAngles( self.zeroAngles)
        #移动左臂到释放附近的位置
        self.mc.send_angles(1, [0, 90, 0, 0,0,0], 50)
        time.sleep(0.10)
        ##移动右臂到释放附近的位置
        self.mc.send_angles(2, [0, -90, 0, 0, -90,0], 50)
        time.sleep(3)
        #释放所有舵机
        self.mc.release_all_servos(0)
        
        
if __name__ == '__main__':
    #测试:初始化连接-移动到抓取状态-输出角度和位姿-释放手臂
    mRC  = MyRobotController()
    # mRC.init()
    # mRC.printAngles()
    # mRC.printBaseCorrds()
    mRC.release()
    


    

    
    
    
        
        
        
import time
import math
from pymycobot import MyBuddySocket
from pymycobot import MyBuddy
import functools    
from xmlrpc.server import SimpleXMLRPCServer
from xmlrpc.server import SimpleXMLRPCRequestHandler
# Restrict to a particular path.

class MyRobotController:
    _rpc_methods_=['init', 'printBaseCoords','printAngles','getAllAngles','safeRelease','moveByCoords','moveAbsShift','getAllBaseCoords','rightArmInit','sendAngles','safeReleaseRight']
    def __init__(self, mode = 'remote', ip="10.3.51.195",port=9000,serialport="/dev/ttyACM0",baud="115200"):
        self._data = {}
        self._serv = SimpleXMLRPCServer(('', 15000), allow_none=True)
        for name in self._rpc_methods_:
            self._serv.register_function(getattr(self, name))

        self.zeroAngles = [[0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0]]
        #过度位置，避免碰撞到操纵台
        self.initAnglesList=[
            [[0, 80, 0, 0, -170, 0],[0,-80,0,0,170,0]],
            [[0, 0, 0, 0, -170, 0],[0,0,0,0,170,0]],
            [[0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0]],
            [[-90,0,0,0,0,0],[90, 0, 0, 0, 0, 0]],
            [[-90,60,0,30,0,60],[90,-60,0,-30,0,60]],
            [[-90,65,0,30,90,45],[90,-65,0,-30,-90,-45]]
        ]
        
        self.gripAngles=[[-84,65,0,35,90,40],[85,-65,0,-35,-90,-40]]
        self.gripUpAngles=[[-90,70,0,35,90,40],[90,-70,0,-35,-90,-40]]
        self.ip=ip
        self.port =port
        self.serialport=serialport
        self.baud = baud
        self.mode = mode
        
        if mode=='remote':
            self.mc = MyBuddySocket(self.ip,self.port) #双臂机器人ip及端口
            self.mc.connect(self.serialport, self.baud)  
        elif mode=='local':
            self.mc = MyBuddy(self.serialport, self.baud)
            # self.mc = MyBuddy("/dev/ttyACM0", "115200")
        else:
            print("模式错误")
            
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
        self.mc.power_on(0)
        time.sleep(1)

        for angles in self.initAnglesList:
            self.moveByAngles(angles)
            
        #夹爪张开到最大
        self.mc.set_gripper_value(1, 100,20)
        time.sleep(1)
        self.mc.set_gripper_value(2, 100,20)
        time.sleep(1)
        
    def rightArmInit(self):
        self.mc.power_on()
        time.sleep(1)
        
        self.mc.release_all_servos(1)  #将左臂的舵机释放
        time.sleep(1)
        
        self.mc.send_angles(2, self.zeroAngles[1], 20)
        time.sleep(2)
        self.mc.send_angles(2, self.tempAngles[1], 20)
        time.sleep(2)
        self.mc.send_angles(2, self.initAngles[1], 20)
        time.sleep(2)
    
    
    def sendAngles(self,id,degrees,speed):
        self.mc.send_angles(id,degrees,speed)
        time.sleep(2)  #睡眠时间要长一点避免动作没做完就读取了坐标
        
        
    def printBaseCoords(self):
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
        coords= self.mc.get_base_coords()
        while coords is None or  len(coords)==0:
            time.sleep(1)
            print(coords)
            coords = self.mc.get_base_coords()
        return coords

    def moveByCoords(self,coords):
        #移动左臂
        self.mc.write_base_coords(1, coords[0], 5)
        time.sleep(0.10)
        #移动右臂
        self.mc.write_base_coords(2, coords[1], 5)
        time.sleep(5)
    
    def moveByAngles(self,angles):
        #移动左臂
        self.mc.send_angles(1, angles[0], 20)
        time.sleep(1)
        #移动右臂
        self.mc.send_angles(2, angles[1], 20)
        time.sleep(3)
        

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
    #标定使用
    def safeReleaseRight(self):
        self.mc.set_gripper_value(2, 100,20)
        time.sleep(1)
        
        self.sendAngles(2,[90, -29, 66, 0, 0, 0],20)
        self.sendAngles(2,self.zeroAngles[1],20)
        self.sendAngles(2,[0, -90, 0, 0, -90,0],20)
        self.mc.release_all_servos(0)
        

    def safeRelease(self):
        #将夹爪张开到最大
        self.mc.set_gripper_value(1, 100,20)
        time.sleep(1)
        self.mc.set_gripper_value(2, 100,20)
        time.sleep(1)
        
        
        for angles in reversed(self.initAnglesList):
            self.moveByAngles(angles)
        
        self.mc.release_all_servos(0)

    def serve_forever(self):
        self._serv.serve_forever()
        
    
        
if __name__ == '__main__':
    #测试:初始化连接-移动到抓取状态-输出角度和位姿-释放手臂
    
    mRC  = MyRobotController(mode="local")
    mRC.mc.set_reference_frame(0,0)
    # mRC.init()
    # for i in range(1):
    #     mRC.printAngles()
    #     mRC.printBaseCoords()
    #     time.sleep(2)

    mRC.safeRelease()
    # '
    
    #作为远程服务器
    # kvserv = MyRobotController(mode="local")
    # kvserv.serve_forever()
    


    
'''
from pymycobot import MyBuddy
mc = MyBuddy("/dev/ttyACM0", "115200")
mc.set_gripper_value(1, 100,20)
mc.set_gripper_value(2, 100,20)
mc.send_angles(1,[-90,65,0,30,90,45],20)


'''
    
        
        
        
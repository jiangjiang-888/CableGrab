import cv2
import threading
import time


import rospy
import tf
import transforms3d as tfs
import geometry_msgs.msg
import sys
import numpy as np
import math
from tf2_msgs.msg import TFMessage
import json

#aruco_ros single 单个结果获取 对应aruco_start_usb_cam.launch
def getRobotCoords():
    trans2,rot2 = [-0.125796  ,   0.0305877 ,  0.546227     ],[ 0.15192925503660906, -0.9875980375216189, 0.027663476853261607,0.02832577970432306]
    camera_link="/camera_frame"
    marker_link= "/aruco_marker_frame"
    base_link="/base"
    
    rospy.init_node('test_hand_to_eye')  
    rate = rospy.Rate(1.0) 
    br = tf.TransformBroadcaster()  
    listener = tf.TransformListener()
    while not rospy.is_shutdown(): 
        try:
            br.sendTransform(trans2,rot2,rospy.Time.now(),camera_link,base_link)
            (trans1,rot1) = listener.lookupTransform(base_link,marker_link, rospy.Time(0))
            print("result:%s->%s, %s,%s" % (base_link,marker_link,trans1,rot1))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()

#启动两个aruco节点，对应启动aruco_start_usb_cam_duble_marker.launch,对应的id需要去这个文件中设置
def getDoubleRobotCoords():
    trans2,rot2 = [-0.125796  ,   0.0305877 ,  0.546227     ],[ 0.15192925503660906, -0.9875980375216189, 0.027663476853261607,0.02832577970432306]
    camera_link1="/camera_frame1"
    camera_link2="/camera_frame2"
    marker_link1= "/aruco_marker_frame1"
    marker_link2= "/aruco_marker_frame2"
    base_link="/base"
    
    rospy.init_node('test_hand_to_eye')  
    rate = rospy.Rate(1.0) 
    br = tf.TransformBroadcaster()  
    listener = tf.TransformListener()
    global transID1,transID2
    while not rospy.is_shutdown(): 
        try:
            br.sendTransform(trans2,rot2,rospy.Time.now(),camera_link1,base_link)
            (transID1,rotID1) = listener.lookupTransform(base_link,marker_link1, rospy.Time(0))
            transID1[0]=transID1[0]*(-1.0) 
            print("result:%s->%s, %s,%s" % (base_link,marker_link1,transID1,rotID1))


            br.sendTransform(trans2,rot2,rospy.Time.now(),camera_link2,base_link)
            (transID2,rotID2) = listener.lookupTransform(base_link,marker_link2, rospy.Time(0))
            transID2[0]=transID2[0]*(-1.0)
            print("result:%s->%s, %s,%s" % (base_link,marker_link2,transID2,rotID2))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        rate.sleep()
            

def thread_func_1(aruco_detector):  
    aruco_detector.detectByCamera()
      
if __name__ == '__main__':

    getDoubleRobotCoords()

    '''
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
    '''
    
    
    
    
    
    
    
    
    
    
    











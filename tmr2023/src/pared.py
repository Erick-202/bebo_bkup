#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from cv_bridge import CvBridge
import cv_bridge
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged
from nav_msgs.msg import Odometry
import cv2
from cmath import sqrt
import math
from rospy import Time
import numpy as np
from _ast import Exec





class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._integral = 0
        #self.tiempo_anterior = Time.now().to_sec()
        self.tiempo_anterior = 0
        self.error_anterior = 0
        
    def calcular(self, setpoint, val_actual):
        #Diferencial de tiempo
        tiempo = Time.now().to_sec()
        dt = tiempo - self.tiempo_anterior
        #Calculo de P
        error = setpoint - val_actual
        proporcional = error * self.kp
        #Calculo de I
        self._integral = self._integral + (error * dt)
        integral = self._integral * self.ki
        #Calculo D
        de = error - self.error_anterior
        derivativo = (de/dt) * self.kd
        
        self.tiempo_anterior = tiempo
        self.error_anterior = error
        
        return proporcional + integral + derivativo


bridge = CvBridge()
#Configuracion de OpenCV
#arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
#arucoParams = cv2.aruco.DetectorParameters_create()
#Matrices de la camara
mtx=np.array([[537.292878, 0.000000, 427.331854], [0.000000, 527.000348, 240.226888], [0.000000, 0.000000, 1.000000]])
dist=np.array([0.004974, -0.000130, -0.001212, 0.002192, 0.000000])
global cam_w, cam_h, landing, teclas_control, estado, kernel, avance_at,dron_x, dron_y, dron_th
avance_at = [0,0]
cam_w = 856
cam_h = 480
landing = 0
estado = -2
teclas_control = {ord("i"),ord("j"), ord("k"), ord("l")}
kernel = np.ones((5,5), np.uint8)

#inicio de PIDs 
pos_x = PID(0.001, 0.000, 0.0006)
pos_y = PID(0.001, 0.000, 0.0006)
pos_z = PID(0.001, 0.000, 0.0006)
ang_z = PID(0.005, 0.000, 0.0)

#Configuracion de ROS Y variables de ROS
cmd_vel_pub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=1)
cmd_vel = Twist()
cmd_vel_cam_pub = rospy.Publisher("/bebop/camera_control", Twist, queue_size=1)
cmd_vel_cam = Twist()
takeoff_pub = rospy.Publisher("/bebop/takeoff", Empty, queue_size=1)
land_pub = rospy.Publisher("/bebop/land", Empty, queue_size=100)
land = Empty()
takeoff = Empty()


tecla_opencv = 0
global sistema_armado, camara_armada
camara_armada = 0
sistema_armado = 0

def altitudeCallback(msg):
    global altura
    altura = msg.altitude

def odometryCallback(msg):
    global dron_x, dron_y, dron_th
    dron_x = msg.pose.pose.position.x
    dron_y = msg.pose.pose.position.y
    dron_th = msg.pose.pose.orientation.z * math.pi


def camaraCallback(msg):
    global sistema_armado, cam_w, cam_h, landing, camara_armada, estado, kernel, avance_at, x_real, y_real, dron_x, dron_y, dron_th
    #Convierte la imagen proveniente de ROS a una imagen que OpenCV puede usar
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    
    x_real = (math.cos(dron_th) * dron_x) + (math.sin(dron_th) * dron_y)
    y_real = (-math.sin(dron_th) * dron_x) + (math.cos(dron_th) * dron_y)
    print "x_real ", x_real
    print "y_real ", y_real
    print ""
    #ewcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx,dist,(cam_h,cam_w),0,(cam_h,cam_w))
    #dst1 = cv2.undistort(frame,mtx,dist,None,newcameramtx)
    #x, y, cam_w, cam_h = roi
    #dst1 = dst1[y:y+cam_h, x:x+cam_w]
    #frame = dst1
    ret, thresh = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
    blur = cv2.medianBlur(thresh, 9)
    #dil = cv2.dilate(blur, kernel, iterations = 1)
    dil = cv2.erode(blur, kernel, iterations = 1)
    #kernel = np.ones((7,7), np.uint8)
    #erosion = cv2.erode(thresh,kernel,iterations=1)

    edges = cv2.Canny(dil, 50, 150, apertureSize = 3)
    #contornos = cv2.findContours(blur,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    
    
    #rho = 1  # distance precision in pixel, i.e. 1 pixel
    #angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
    #min_threshold = 10  # minimal of votes
    #line_segments = cv2.HoughLinesP(frame, rho, angle, min_threshold,
    #                                np.array([]), minLineLength=8, maxLineGap=4)
    
    
    #cv2.imshow("filtrado", thresh)
    
    
    if sistema_armado == 1:
        if estado == -2:
            avance_at[0] = x_real
            avance_at[1] = y_real
            estado = -1
        elif estado == -1:
            if x_real < avance_at[0] + 0.9 and x_real > avance_at[0] + 0.7:
                    cmd_vel.linear.x = 0
                    cmd_vel.linear.y = 0
                    cmd_vel.linear.z = 0
                    estado = 0
                    avance_at[0] = x_real
                    pos_x.kp = 0.001 
                    #avance_at[1] = y_real
            else:
                #Aqui se le ordena avanzar una distancia de 0.5 en x
                print "actual ", x_real, " hasta ", avance_at[0] + 0.5
                pos_x.kp = 0.065
                cmd_vel.linear.x = pos_x.calcular(avance_at[0] + 0.8, x_real)
                #cmd_vel.linear.x = 0.01
                cmd_vel.linear.y = 0 # pos_y.calcular(avance_at[1], y_real)
                print('Estoy avanzando hacia adelante')
        
        elif estado == 0:
            pos_x.kp = 0.01
            lines = cv2.HoughLinesP(edges,1,np.pi/180,threshold = 100, minLineLength = 2, maxLineGap = 10)
            if lines != None:
                for line in lines:
                    x1,y1,x2,y2 = line[0]
                    cv2.line(frame, (x1,y1), (x2,y2), (0,0,255), 2)
                    if y1 < 200 and y2 <200 and x2 != x1:
                        try:
                            angulo = math.atan((y2-y1)/(x2-x1))
                        except:
                            angulo = math.pi/2
                        angulo = angulo * (180/math.pi)
                        print('ANGULOS : ',angulo)
                        
                        if angulo == 180:
                            #cX = ((x2 - x1) / 2) + x1
                            cY = ((y2 - y1) / 2) + y1
                        #if (y2 - y1) <= 20 and y2 > 400:
                        #    estado = 1
                        if (angulo >= 0 and angulo <= 15) or (angulo <= 180 and angulo >= 165) and (cY < (cam_h - (cam_h/3)+40) and (cY > cam_h - (cam_h/3))-30):
                            print('Vi una linea horizontal')
                            estado = 1
                            cmd_vel.linear.x = 0
                            cmd_vel.linear.y = 0
                            cmd_vel.linear.z = 0
                            cmd_vel.angular.z = 0
                            land_pub.publish(land)
                            print('aterrice')
                            
                        
                            
                        
                    else:
                        cmd_vel.linear.x = 0.01
                        cmd_vel.linear.y = 0
                        cmd_vel.linear.z = 0
                        cmd_vel.angular.z = 0
                        print('Puro para delante')
            else:
                cmd_vel.linear.x = 0.01
                cmd_vel.linear.y = 0
                cmd_vel.linear.z = 0
                cmd_vel.angular.z = 0
                print('Puro para delante')
                
        elif estado == 1:
            cmd_vel.linear.x = 0
            cmd_vel.linear.y = 0
            cmd_vel.linear.z = 0
            cmd_vel.angular.z = 0
            land_pub.publish(land)
            print('aterrice')
            #cmd_vel.linear.y = pos_y.calcular(cam_w/2, cX)
            #cmd_vel.linear.z = pos_z.calcular(cam_h/2, cY)
            #cmd_vel.angular.z = -ang_z.calcular(0.0, angulox)
        else:
            print "si perro"
                                               
            cmd_vel.linear.x = 0
            cmd_vel.linear.y = 0
            cmd_vel.linear.z = 0
            cmd_vel.angular.z = 0
            land_pub.publish(land)
                    
                    
    else:
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.linear.z = 0;
        cmd_vel.angular.z = 0;
                    
    
    if sistema_armado == 1:
        cv2.putText(frame, "ARMADO", 
        (10, 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5, (0, 0, 255), 2)
    
    #cv2.imshow("thresh", thresh)
    #cv2.imshow("edgy", edges)
    #cv2.imshow("blur", blur)
    #cv2.imshow('DIlate', dil)
    cv2.imshow("Bebop", frame)
    cmd_vel_pub.publish(cmd_vel)
    
    
    tecla_opencv = cv2.waitKey(1)
    
    
    if tecla_opencv in teclas_control:
        sistema_armado = 0
        cmd_vel.linear.x = 0
        cmd_vel.linear.y = 0
        cmd_vel.linear.z = 0
        cmd_vel.angular.z = 0
        if tecla_opencv == ord("i"):
            cmd_vel.linear.x = 0.2
        elif tecla_opencv == ord("j"):
            cmd_vel.linear.y = 0.2
        elif tecla_opencv == ord("k"):
            cmd_vel.linear.x = -0.2
        elif tecla_opencv == ord("l"):
            cmd_vel.linear.y = -0.2
        cmd_vel_pub.publish(cmd_vel)
    else:
        cmd_vel.linear.x = 0
        cmd_vel.linear.y = 0
        cmd_vel.linear.z = 0
        cmd_vel.angular.z = 0
        if tecla_opencv == ord(" ") :
            land_pub.publish(land)

    
    if tecla_opencv == ord(" ") :
        land_pub.publish(land)
        sistema_armado = 0
    
    elif tecla_opencv == ord("d"):
        sistema_armado = 0
        takeoff_pub.publish(takeoff)
    
    elif tecla_opencv == ord("a"):
        if sistema_armado == 1:
            sistema_armado = 0
        else:
            sistema_armado = 1
    elif tecla_opencv == ord("o"):
            camara_armada = 1 - camara_armada
            if camara_armada == 1:
                #Camara gira hacia abajo 
                cmd_vel_cam.angular.y = -75
                print('EStoy volteando hacia abajo perro, ANgulo camara: ')
            else:
                cmd_vel_cam.angular.y = 0
            cmd_vel_cam_pub.publish(cmd_vel_cam)
        
        #cmd_vel.cam.ang.z
            
        



def bebop():
    #cam = cv2.VideoCapture(0)
    #Configuracion de ROS
    rospy.init_node('objeto', anonymous=False)
    cmd_vel_cam.angular.y = -60
    cmd_vel_cam_pub.publish(cmd_vel_cam)
    odometry_sub = rospy.Subscriber("/bebop/odom", Odometry, odometryCallback)
    #camara_pub = rospy.Publisher('/camara', Image, queue_size=10)
    camara_sub = rospy.Subscriber("/bebop/image_raw", Image, camaraCallback)
    altitude_sub = rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AltitudeChanged", Ardrone3PilotingStateAltitudeChanged, altitudeCallback)
    
    pos_x.tiempo_anterior = Time.now().to_sec()
    pos_y.tiempo_anterior = Time.now().to_sec()
    pos_z.tiempo_anterior = Time.now().to_sec()
    ang_z.tiempo_anterior = Time.now().to_sec()
    
    rate = rospy.Rate(24) # 10hz
    while not rospy.is_shutdown():
        #_, frame = cam.read()
        #cv_image = bridge.cv2_to_imgmsg(frame, encoding="passthrough")
        #camara_pub.publish(cv_image)
        rate.sleep()
        #rospy.spin()



if __name__ == '__main__':
    try:
        bebop()
    except rospy.ROSInterruptException:
        #cam.release()
        pass




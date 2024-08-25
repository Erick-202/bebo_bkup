#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged
from cv_bridge import CvBridge
import cv_bridge
import cv2
from cmath import sqrt
import math
from rospy import Time
import numpy as np


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
global cam_w, cam_h, landing
cam_w = 856
cam_h = 480
landing = 0

sistema_armado = 1
tecla_opencv = 0

#Configuracion de ROS Y variables de ROS
cmd_vel_pub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=1)
cmd_vel = Twist()
cmd_vel_cam_pub = rospy.Publisher("/bebop/camera_control", Twist, queue_size=1)
cmd_vel_cam = Twist()
takeoff_pub = rospy.Publisher("/bebop/takeoff", Empty, queue_size=1)
land_pub = rospy.Publisher("/bebop/land", Empty, queue_size=100)
land = Empty()
takeoff = Empty()

#Definir kernel
kernel = np.ones((5,5), np.uint8)


def camaraCallback(msg):
    global sistema_armado, cam_w, cam_h, camara_armada, i, arreglo, estado, teclas_control
    #Convierte la imagen proveniente de ROS a una imagen que OpenCV puede usar
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    #Convierte a escala de grises
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    ret, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY_INV)
	# 40 jala bien
    	
    # Convertir el frame a HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    #---Rangos de color--#
    lower_black = (49, 34, 46)
    upper_black = (152, 159, 152)

    maskBlack = cv2.inRange(hsv, lower_black, upper_black)


    eroded_maskBlack = cv2.erode(maskBlack, kernel, iterations=1)

    #--Final Amarillo--#
   # maskBlackFinal = cv2.bitwise_and(frame, frame, mask=eroded_maskBlack)

    cv2.imshow("Yellow Mask", thresh) 

        
    #------------------------------------------------------Contornos
    
    contornos = cv2.findContours(eroded_maskBlack, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    
    tecla_opencv = cv2.waitKey(1)
    cv2.imshow("Bebop", frame)
    cmd_vel_pub.publish(cmd_vel)
    cmd_vel_cam_pub.publish(cmd_vel_cam)

    estado = 0

    if(tecla_opencv == ord("e")):
        estado == 0

    #Estado para centrar
    if(estado == 0):
        #if cY != 0 and cX != 0:
            #---Aqui se agrega el PID
            #cmd_vel.linear.x = pos_x.calcular(cam_h - cam_h/3, cy)
            #cmd_vel.linear.y = pos_y.calcular(cam_w/2, cx)
        
            #if cY < cam_h - (cam_h/3) + 50 and cY > cam_h - (cam_h/3) - 30:
                    # if  cX < cam_w/2 + 50 and cX > cam_w/2 - 50:
     	print ("Centrado")
            

        
    if(tecla_opencv == ord("l")):
         land_pub.publish(land)

    elif tecla_opencv == ord("d"):
        sistema_armado = 0
        takeoff_pub.publish(takeoff)

    elif tecla_opencv == ord("a"):
        if sistema_armado == 1:
                sistema_armado = 0
        else:
                sistema_armado = 1

    elif tecla_opencv == ord("o"):
        cmd_vel_cam.angular.y = -50;    

def bebop():
    #Configuracion de ROS
    rospy.init_node('objeto', anonymous=False)
    print('bebop')
    camara_sub = rospy.Subscriber("/bebop/image_raw", Image, camaraCallback)
    #altitude_sub = rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AltitudeChanged", Ardrone3PilotingStateAltitudeChanged, altitudeCallback)

    rate = rospy.Rate(24) # 10hz
    while not rospy.is_shutdown():  
        #rint ('sleeep')
        rate.sleep()
        #rospy.spin()
    

if __name__ == '__main__':
    try:
	#print('si jala')
        bebop()
    except rospy.ROSInterruptException:
        #cam.release()
        pass


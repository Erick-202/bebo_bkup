#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_msgs.msg import String
from std_msgs.msg import Bool
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
import time
import serial

import pyzbar.pyzbar as pyzbar


# TECH Master:  este es para tirar la BOTELLA




class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._integral = 0
        # self.tiempo_anterior = Time.now().to_sec()
        self.tiempo_anterior = 0
        self.error_anterior = 0

    def calcular(self, setpoint, val_actual):
        # Diferencial de tiempo
        tiempo = Time.now().to_sec()
        dt = tiempo - self.tiempo_anterior
        # Calculo de P
        error = setpoint - val_actual
        proporcional = error * self.kp
        # Calculo de I
        self._integral = self._integral + (error * dt)
        integral = self._integral * self.ki
        # Calculo D
        de = error - self.error_anterior
        derivativo = (de / dt) * self.kd

        self.tiempo_anterior = tiempo
        self.error_anterior = error

        return proporcional + integral + derivativo


bridge = CvBridge()
# Configuracion de OpenCV

# Matrices de la camara
mtx = np.array([[537.292878, 0.000000, 427.331854], [0.000000, 527.000348, 240.226888], [0.000000, 0.000000, 1.000000]])
dist = np.array([0.004974, -0.000130, -0.001212, 0.002192, 0.000000])
global cam_w, cam_h, landing, teclas_control, estado, kernel, avance_at, dron_x, dron_y, dron_th
avance_at = [0, 0]
cam_w = 856
cam_h = 480
landing = 0
estado = 0
teclas_control = {ord("i"), ord("j"), ord("k"), ord("l"), ord("u"), ord("o"), ord("y"), ord("h")}
kernel = np.ones((5, 5), np.uint8)

global pos_x, pos_y, pos_z
# inicio de PIDs
pos_x = PID(0.0005, 0.000, 0.0006)
pos_y = PID(0.001, 0.000, 0.0006)
pos_z = PID(0.0, 0.000, 0.0006)
ang_z = PID(0.001, 0.000, 0.000)

# Configuracion de ROS Y variables de ROS
cmd_vel_pub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=1)
cmd_vel = Twist()
cmd_vel_cam_pub = rospy.Publisher("/bebop/camera_control", Twist, queue_size=1)
cmd_vel_cam = Twist()
takeoff_pub = rospy.Publisher("/bebop/takeoff", Empty, queue_size=1)
land_pub = rospy.Publisher("/bebop/land", Empty, queue_size=100)
land = Empty()
takeoff = Empty()

global hz
hz = 200

tecla_opencv = 0
global sistema_armado, camara_armada
camara_armada = 0
sistema_armado = 0

global bottle_x,bottle_z, bottle_area, distancia, bool_centrado, contador, time_inicial, time_actual, bool_avanzar
bottle_x = 0
bottle_z = 0
bottle_area = 0
distancia = 0
bool_centrado = False
contador = 0
time_inicial = 0
time_actual = 0
bool_avanzar = False


def bottle_x_callback(msg):
    global bottle_x
    bottle_x = int(msg.data)
    #print "angle  = " , blue_angle

def bottle_z_callback(msg):
    global bottle_z
    bottle_z = int(msg.data)
    #print "X = " , str(blue_x)

def bottle_area_callback(msg):
    global bottle_area
    bottle_area = float(msg.data)
   # print "ARea = " , str(bottle_area)


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
    global frame, thresh
    global bottle_x, bottle_z, bottle_area, distancia, bool_centrado, contador, bool_avanzar, time_inicial, time_actual


    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    # Convert ROS Image message to OpenCV image
    current_frame = br.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    # Process image
    frame = current_frame




    if sistema_armado == 1:

        cv2.putText(frame, "ARMADO",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1, (0, 0, 255), 3)


        if bottle_x != 0 and bottle_z != 0 : #Si ve la botella

            #cv2.line(frame, (0,blue_y),(cam_w,blue_y),(255,0,0),3 )

            #regresion = 2162.9 * x  ^0.445         donde x es el area
            print "area ", str(bottle_area)

            distancia = 2162.9 * pow(bottle_area,- 0.445)
            print str(distancia)

            cv2.putText(frame, str(distancia), (150,50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 1, cv2.LINE_AA)

            cv2.line(frame, (0, cam_h // 2 - 50), (cam_w,cam_h // 2 - 50), (255, 0, 255), 3)
            cv2.line(frame, (0, cam_h//2 +50), (cam_w, cam_h//2 +50), (255, 0, 255), 3)
            cv2.line(frame, (cam_w // 2 -50, 0), (cam_w // 2 -50, cam_h), (0, 255, 255), 3)
            cv2.line(frame, (cam_w // 2 + 50, 0), (cam_w // 2 + 50, cam_h), (0, 255, 255), 3)

            if int(bottle_z) >   cam_h // 2 - 50 and int(bottle_z) < cam_h//2 +50 and \
                    int(bottle_x) > cam_w // 2 -50  and int(bottle_x) < cam_w // 2 + 50:

                print("  / \\__")
                print(" (    @\\___")
                print(" /         O       ")
                print("/   (_____/")
                print("/_____/   U")

                print ("Centrado en BOtella PERRO")

                bool_centrado = True
                if contador == 0:
                    time_inicial = Time.now().to_sec()
                    contador = contador + 1
                    print "TIME INICIAL " + str(time_inicial)

            else:
                cmd_vel.linear.x =0
                cmd_vel.linear.y =- pos_y.calcular(cam_w//2,bottle_x)
                cmd_vel.linear.z = pos_z.calcular(cam_h//2,bottle_z)
                cmd_vel.angular.z = 0

                bool_centrado = False
                contador = 0
                time_inicial = 0
                time_actual = 0

            if bool_centrado == True:
                time_actual = Time.now().to_sec()
                print "Time ACTUAL " + str(time_actual)

            if time_actual - time_inicial >= 3:
                bool_avanzar = True

            else:
                bool_avanzar = False

                print("Aun no puede rotar")

            if bool_avanzar == True:

                if distancia >= 15 and distancia <= 20:
                    print "Ya esta en el rango  -->     AVANZAR, tirar botella"
                    cmd_vel.linear.x = 0.01
                    cmd_vel.linear.y =0
                    cmd_vel.linear.z = 0
                    cmd_vel.angular.z = 0

                else:
                    print "ponerse a 20 cm"
                    cmd_vel.linear.x = pos_x.calcular(distancia, 20)
                    cmd_vel.linear.y = 0
                    cmd_vel.linear.z =0
                    cmd_vel.angular.z = 0




        else:

            cmd_vel.linear.x = 0.01;
            cmd_vel.linear.y = 0;
            cmd_vel.linear.z = 0;
            cmd_vel.angular.z = 0;


    else:
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.linear.z = 0;
        cmd_vel.angular.z = 0;

    cv2.imshow('tech_MASTER2', frame)
    #cv2.moveWindow("tech_MASTER", 0, 0)

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


        elif tecla_opencv == ord("u"):
            cmd_vel.angular.z = 1
            print ("1")


        elif tecla_opencv == ord("o"):
            cmd_vel.angular.z = -1
            print ("-1")

        elif tecla_opencv == ord("y"):
            cmd_vel.linear.z = 0.2

        elif tecla_opencv == ord("h"):
            cmd_vel.linear.z = -0.2

        cmd_vel_pub.publish(cmd_vel)
    else:
        cmd_vel.linear.x = 0
        cmd_vel.linear.y = 0
        cmd_vel.linear.z = 0
        cmd_vel.angular.z = 0
        if tecla_opencv == ord(" "):
            land_pub.publish(land)

    if tecla_opencv == ord(" "):
        land_pub.publish(land)
        sistema_armado = 0

    elif tecla_opencv == ord("d"):
        sistema_armado = 0
        takeoff_pub.publish(takeoff)


        """EXPERIMENTAL: Se debe armar automaticamente  despues de despegar """
        for i in range(1,5):
            print "Esperando a ARMARSE"
            time.sleep(1)

        sistema_armado = 1


    elif tecla_opencv == ord("a"):
        if sistema_armado == 1:
            sistema_armado = 0
        else:
            sistema_armado = 1
    elif tecla_opencv == ord("c"):
        camara_armada = 1 - camara_armada
        if camara_armada == 1:
            # Camara gira hacia abajo
            cmd_vel_cam.angular.y = -75
            print('EStoy volteando hacia abajo perro')
        else:
            print('NO agaches la cabeza rey por que se te cae la corona')
            cmd_vel_cam.angular.y = -20
        cmd_vel_cam_pub.publish(cmd_vel_cam)

    # cmd_vel.cam.ang.z


def bebop():
    # cam = cv2.VideoCapture(0)
    # Configuracion de ROS
    rospy.init_node('objeto', anonymous=False)
    cmd_vel_cam.angular.y = -60
    cmd_vel_cam_pub.publish(cmd_vel_cam)

    odometry_sub = rospy.Subscriber("/bebop/odom", Odometry, odometryCallback)
    # camara_pub = rospy.Publisher('/camara', Image, queue_size=10)
    camara_sub = rospy.Subscriber("/bebop/image_raw", Image, camaraCallback)
    altitude_sub = rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AltitudeChanged",
                                    Ardrone3PilotingStateAltitudeChanged, altitudeCallback)


    bottle_sub_py = rospy.Subscriber("bottle_x", String, bottle_x_callback)
    bottle_sub_py = rospy.Subscriber("bottle_z", String, bottle_z_callback)
    bottle_sub_py = rospy.Subscriber("bottle_area", String, bottle_area_callback)



    print(" _ 	__     __   _   _     __  __	 ____       ____     _____    ____ 	")
    print("| |	\ \   / /  | \ | |    \ \/ /	|  _ \     / __ \   |_   _|  / ___|	")
    print("| |	 \ \_/ /   |  \| |     \  /	| |_) |   | |  | |    | |   | |___ 	")
    print("| |	  \   /    | |\  |     >  <	|  _ <    | |  | |    | |    \___ \ 	")
    print("| |_____   | |     | | \ |    / /\ \	| |_) |   | |__| |    | |    ___) |	")
    print("|_______|  |_|     |_|  \|   /_/  \_\	|____/     \____/     |_|   |____/ 	")

    pos_x.tiempo_anterior = Time.now().to_sec()
    pos_y.tiempo_anterior = Time.now().to_sec()
    pos_z.tiempo_anterior = Time.now().to_sec()
    ang_z.tiempo_anterior = Time.now().to_sec()

    rate = rospy.Rate(hz)  # 10hz 24
    while not rospy.is_shutdown():
        # _, frame = cam.read()
        # cv_image = bridge.cv2_to_imgmsg(frame, encoding="passthrough")
        # camara_pub.publish(cv_image)
        rate.sleep()
        # rospy.spin()


if __name__ == '__main__':
    try:
        bebop()
    except rospy.ROSInterruptException:
        # cam.release()
        pass


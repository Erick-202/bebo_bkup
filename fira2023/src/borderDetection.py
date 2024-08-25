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
# Configuracion de OpenCv
# Matrices de la camara
mtx = np.array([[537.292878, 0.000000, 427.331854], [0.000000, 527.000348, 240.226888], [0.000000, 0.000000, 1.000000]])
dist = np.array([0.004974, -0.000130, -0.001212, 0.002192, 0.000000])
global cam_w, cam_h
cam_w = 856
cam_h = 480

# Definir kernel
kernel = np.ones((5, 5), np.uint8)


def Hminimo(val):
    pass


def Hmaximo(val):
    pass


# Descomentar para modificar la mascara y los parametros de los circulos
"""
    cv2.namedWindow("Trackbars")
    cv2.createTrackbar("Bmax", "Trackbars", 121, 255, Hminimo)
    cv2.createTrackbar("Bmin", "Trackbars", 0, 255, Hmaximo)
    cv2.createTrackbar("Gmax", "Trackbars", 115, 255, Hminimo)
    cv2.createTrackbar("Gmin", "Trackbars", 0, 255, Hmaximo)
    cv2.createTrackbar("Rmax", "Trackbars", 68, 255, Hminimo)
    cv2.createTrackbar("Rmin", "Trackbars", 0, 255, Hmaximo)

    cv2.createTrackbar("Area", "Trackbars", 0, 10000, Hminimo)
    cv2.createTrackbar("Circu", "Trackbars", 0, 1000, Hmaximo)
    cv2.createTrackbar("Convex", "Trackbars", 0, 1000, Hminimo)
    cv2.createTrackbar("Inercia", "Trackbars", 0, 1000, Hmaximo)
    cv2.namedWindow("Trackbars")
    tecla_opencv = cv2.waitKey(1)
"""

# inicio de PIDs
pos_x = PID(0.001, 0.000, 0.0006)
pos_y = PID(0.001, 0.000, 0.0006)
pos_z = PID(0.001, 0.000, 0.0006)
ang_z = PID(0.0005, 0.000, 0.0)

# Configuracion de ROS Y variables de ROS
cmd_vel_pub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=1)
cmd_vel = Twist()
cmd_vel_cam_pub = rospy.Publisher("/bebop/camera_control", Twist, queue_size=1)
cmd_vel_cam = Twist()
takeoff_pub = rospy.Publisher("/bebop/takeoff", Empty, queue_size=1)
land_pub = rospy.Publisher("/bebop/land", Empty, queue_size=100)
land = Empty()
takeoff = Empty()

tecla_opencv = 0
global sistema_armado, camara_armada, estado, altura, dron_x, dron_y, dron_th, th_ini, teclas_control, avance_at
avance_at = [0, 0]
teclas_control = {ord("i"), ord("j"), ord("k"), ord("l")}
altura = 0
estado = 0
sistema_armado = 0
camara_armada = 0


# Variables de odometria
# global dron_x, dron_y, dron_th


def altitudeCallback(msg):
    global altura
    altura = msg.altitude


def odometryCallback(msg):
    global dron_x, dron_y, dron_th
    dron_x = msg.pose.pose.position.x
    dron_y = msg.pose.pose.position.y
    dron_th = msg.pose.pose.orientation.z * math.pi


def camaraCallback(msg):
    global sistema_armado, camara_armada, estado, altura, th_ini, dron_th, teclas_control, avance_at, cX, cY
    global frame
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    cX = 0
    cY = 0


    # Bordes 4-------------------------------------------------------------------------
    # Convertir a HSV
    #cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Se aplica una mascara de negros a la imagen
    negroClaro = np.array([0, 0, 0], np.uint8)
    negroOscuro = np.array([121, 115, 68], np.uint8)


    # Se unen ambos rangos en una mascara
    maskCom = cv2.inRange(frame, negroClaro, negroOscuro)

    # -----EROSION-----#
    eroded_maskBlack = cv2.erode(maskCom, kernel, iterations=1)

    _, contornos ,_= cv2.findContours(eroded_maskBlack, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for c in contornos:
       # print (c.dtype)
        area = cv2.contourArea(c)
        if area > 15000 and area < 50000:
            M = cv2.moments(c)
            if (M["m00"] == 0): M["m00"] = 1
            cX = int(M["m10"] / M["m00"])
            cY = int(M['m01'] / M['m00'])



            cv2.circle(frame, (cX, cY), 7, (0, 255, 0), 3)

            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(frame, '{},{}'.format(cX, cY), (cX + 10, cY), font, 0.75, (0, 255, 0), 1, cv2.LINE_AA)
            cv2.putText(frame, str(area), (cX, cY+30), font, 0.75, (0, 0, 255), 1, cv2.LINE_AA)

            nuevoContorno = cv2.convexHull(c)
            cv2.drawContours(frame, [nuevoContorno], 0, (255, 0, 0), 3)



    cv2.line(frame, (cam_w // 2 - 50, 0), (cam_w // 2 - 50, cam_h), (255, 0, 0), 3)
    cv2.line(frame, (cam_w // 2 + 50, 0), (cam_w // 2 + 50, cam_h), (255, 0, 0), 3)

    if (cX > 0 and cY > 0):
            print("Centrando")
            # cmd_vel.linear.x = pos_x.calcular(cam_h - cam_h/3, cY)
            #cmd_vel.linear.x = 0.1
            #cmd_vel.linear.y = pos_y.calcular(cam_w / 2, cX)

            # Se centra la camara en el punto central de la pista y se acerca poco a poco.
            if cX < cam_w / 2 + 50 and cX > cam_w / 2 - 50:

                # if cY < cam_h - (cam_h/3) + 50 and cY > cam_h - (cam_h/3) - 30:
                print ("Centrado papa")

            # No utiliza como tal Y para centrarse, pero funciona para saber si ya esta llegando a un QR
            if cY < cam_h - (cam_h / 4) + 50 and cY > cam_h - (cam_h / 4) - 30:
                print ("Llegando a un QR")

    cv2.imshow("Bebop", frame)

    # Comando para iniciar el programa.
    if sistema_armado == 1:
        cmd_vel_pub.publish(cmd_vel)
    else:
        cmd_vel.linear.x = 0
        cmd_vel.linear.y = 0
        cmd_vel.linear.z = 0
        cmd_vel.angular.z = 0
        cmd_vel_pub.publish(cmd_vel)

    tecla_opencv = cv2.waitKey(1)

    # MUY IMPORTANTE
    '''
     LAS TECLAS I, J, K, L SIRVEN PARA MOVER MANUALMENTE EL DRON, AL PRESIONAR ESTAS TECLAS EL DRON DEJA DE SER AUTONOMO
     ESTO SE PUSO EN CASO DE QUE EL DRON HAGA UNA ACCION NO DESEADA PODER CONTROLARLO Y EVITAR QUE COLISIONE CON ALGO O ATERRICE
     EN UN LUGAR POCO ADECUADO, POR LO QUE ES IMPORTANTE ESTAR MONITOREANDOLO Y ESTAR PREPARADO PARA MOVERLO MANUALMENTE.
    '''
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



        if tecla_opencv == ord(" "):
            land_pub.publish(land)

        elif tecla_opencv == ord("d"):
            sistema_armado = 0
            th_ini = dron_th
            takeoff_pub.publish(takeoff)

        elif tecla_opencv == ord("a"):
            if sistema_armado == 1:
                sistema_armado = 0
            else:
                sistema_armado = 1

        elif tecla_opencv == ord("o"):
            camara_armada = 1 - camara_armada
            if camara_armada == 1:
                # Camara gira hacia abajo
                cmd_vel_cam.angular.y = -60
            else:
                cmd_vel_cam.angular.y = 0
            cmd_vel_cam_pub.publish(cmd_vel_cam)




def bebop():
    global dron_x, dron_y, dron_th, tiempo_anterior
    # cam = cv2.VideoCapture(0)
    # Configuracion de ROS
    rospy.init_node('objeto', anonymous=False)
    # camara_pub = rospy.Publisher('/camara', Image, queue_size=10)
    camara_sub = rospy.Subscriber("/bebop/image_raw", Image, camaraCallback)
    altitude_sub = rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AltitudeChanged",
                                    Ardrone3PilotingStateAltitudeChanged, altitudeCallback)
    odometry_sub = rospy.Subscriber("/bebop/odom", Odometry, odometryCallback)

    pos_x.tiempo_anterior = Time.now().to_sec()
    pos_y.tiempo_anterior = Time.now().to_sec()
    pos_z.tiempo_anterior = Time.now().to_sec()
    ang_z.tiempo_anterior = Time.now().to_sec()
    tiempo_anterior = Time.now().to_sec()

    rate = rospy.Rate(24)  # 10hz
    while not rospy.is_shutdown():
        rate.sleep()
        # rospy.spin()


if __name__ == '__main__':
    try:
        bebop()
    except rospy.ROSInterruptException:
        # cam.release()
        pass


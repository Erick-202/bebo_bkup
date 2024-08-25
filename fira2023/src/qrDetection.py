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
# arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
# arucoParams = cv2.aruco.DetectorParameters_create()
# Matrices de la camara
mtx = np.array([[537.292878, 0.000000, 427.331854], [0.000000, 527.000348, 240.226888], [0.000000, 0.000000, 1.000000]])
dist = np.array([0.004974, -0.000130, -0.001212, 0.002192, 0.000000])
global cam_w, cam_h, landing, teclas_control, estado, kernel, avance_at, dron_x, dron_y, dron_th
avance_at = [0, 0]
cam_w = 856*2
cam_h = 480*2
landing = 0
estado = -4
teclas_control = {ord("i"), ord("j"), ord("k"), ord("l"), ord("u"), ord("o")}
kernel = np.ones((5, 5), np.uint8)

# inicio de PIDs
pos_x = PID(0.001, 0.000, 0.0006)
pos_y = PID(0.001, 0.000, 0.0006)
pos_z = PID(0.001, 0.000, 0.0006)
ang_z = PID(0.005, 0.000, 0.0)

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
    # Convierte la imagen proveniente de ROS a una imagen que OpenCV puede usar
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    if sistema_armado == 1:
        cv2.putText(frame, "ARMADO",
                    (10, 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 0, 255), 2)








    # cv2.imshow("thresh", thresh)
    # cv2.imshow("edgy", edges)
    # cv2.imshow("blur", blur)
    # cv2.imshow('DIlate', dil)
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
        elif tecla_opencv == ord("u"):
            cmd_vel.angular.z = -0.2
        elif tecla_opencv == ord("o"):
            cmd_vel.angular.z = 0.2
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

    elif tecla_opencv == ord("a"):
        if sistema_armado == 1:
            sistema_armado = 0
        else:
            sistema_armado = 1
    elif tecla_opencv == ord("o"):
        camara_armada = 1 - camara_armada
        if camara_armada == 1:
            # Camara gira hacia abajo
            cmd_vel_cam.angular.y = -75
            print('EStoy volteando hacia abajo perro, ANgulo camara: ')
        else:
            cmd_vel_cam.angular.y = 0
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

    pos_x.tiempo_anterior = Time.now().to_sec()
    pos_y.tiempo_anterior = Time.now().to_sec()
    pos_z.tiempo_anterior = Time.now().to_sec()
    ang_z.tiempo_anterior = Time.now().to_sec()

    rate = rospy.Rate(24)  # 10hz
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



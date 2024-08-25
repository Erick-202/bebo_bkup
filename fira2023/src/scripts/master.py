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


#FALTA:
#2.-    Establecer la altura a 0.75m
#3.-    Hacer las misiones 2,3,4
#4.-    Que haga las rotaciones mas suaves
#5.- Checar lo de que AVANZA antes de ver un QR


"""ACTUALIZAR CUANDO SE CAMBIA la realMIsion en QR.py"""
global misionFinalizada #mision Finalizada es una variable que cuenta la mision que acaba de terminar para que no la vuelva a leer
misionFinalizada = 0

global contadorQR
contadorQR =1 #para MABE nomas

#---------  MASTER : Este codigo recibe la info de los otros NODOS y la procesa para tomar las decisiones
#                       ->  MAQUINA DE ESTADOS

global estado
estado = 0

#->  Working (falta mucho)


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
cam_w = 856
cam_h = 480
landing = 0
estado = 0
teclas_control = {ord("i"), ord("j"), ord("k"), ord("l"), ord("u"), ord("o")}
kernel = np.ones((5, 5), np.uint8)


global pos_x, pos_y, pos_z
# inicio de PIDs
pos_x = PID(0.0005, 0.000, 0.0006)
pos_y = PID(0.0001, 0.000, 0.0006)
pos_z = PID(0.1, 0.000, 0.0006)
ang_z = PID(0.001, 0.000, 0.006)

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




global priority
priority = 0

global bool_rotation, rotation, bool_QRcentrado  #el bool_rotation es pa que no se cicle
rotation = 0
bool_rotation = False
bool_QRcentrado = False

global bridge_area
bridge_area = 0

global mision, bool_mision
mision = 0
bool_mision = False

global angular, horiz, bool_horizCentrado
angular = 0
horiz = cam_w//2
bool_horizCentrado = False

global qr_centroX, qr_centroY, bool_QRdetectado
bool_QRdetectado = False
qr_centroX = 0
qr_centroY = 0

#ANtes de que el dron  detecte el 1er QR va a avanzar para adelante por que tenemos problemas con la alineacion horizontal
global new_qr, contadorQR, qr_ant
new_qr = False
contadorQR = 0
qr_ant = 0

global contador, time_inicial, time_actual
contador = 0
time_inicial = 0
time_actual = 0

global angulo, centrado2
angulo = 90 #Inicia en 90
centrado2 = False


global m1_x, m1_y
m1_x = 0
m1_y = 0

global stopAll  #stopAll detiene lo de detectar la calle y los Qr ' s para hacer una mision
stopAll = False

global cruzCentrado, paqSoltado
cruzCentrado = False
paqSoltado = False




#CONFIGURAR EL PUERTO SERIAL

global ser
#Creacion de una instancia del objeto serial

#Configuracion del puerto serie
# Configura el puerto serie

"""
ser = serial.Serial()
ser.baudrate = 9600
ser.port = '/dev/ttyUSB0'
"""

global tower_bool
tower_bool = False

global blue_x, blue_y
blue_x = 0
blue_y = 0

global altura
altura = 0


global apiFrameOff, apiFrameOn
#Mensajes a enviar
apiFrameOn = bytearray.fromhex("7E 00 10 17 01 00 13 A2 00 41 24 23 C3 9A 71 02 44 33 05 5E")
apiFrameOff = bytearray.fromhex("7E 00 10 17 01 00 13 A2 00 41 24 23 C3 9A 71 02 44 33 04 5F")

def altitudeCallback(msg):
    global altura
    altura = float(msg.altitude)


def odometryCallback(msg):
    global dron_x, dron_y, dron_th
    dron_x = msg.pose.pose.position.x
    dron_y = msg.pose.pose.position.y
    dron_th = msg.pose.pose.orientation.z * math.pi

#-----  PUENTES-------------
def area_callback(msg):
    global bridge_area
    bridge_area = int(msg.data)
    print (msg.data)
    #rospy.loginfo("area detectada: %.2f" % bridge_area)


#  ------   QR detection -----------------
def rotation_callback(msg):
    global rotation, bool_rotation
    rotation = int(msg.data)
    #print (msg.data)
    rospy.loginfo("rotar : %.2f" % rotation)
    #bool_rotation = True

def mision_callback(msg):
    global mision, bool_mision
    mision = int(msg.data)
    print ""
    print ""
    print ""
    print ""
    print ""
    rospy.loginfo("mision : %.2f" % mision)
    print (mision)
    bool_mision = True

def qr_centroX_callback(msg):
    global qr_centroX, bool_QRdetectado
    qr_centroX = int(msg.data)
    #rospy.loginfo("centro X: %.2f" % qr_centroX)
    bool_QRdetectado = True

def qr_centroY_callback(msg):
    global qr_centroY
    qr_centroY = int(msg.data)
    #rospy.loginfo("centro Y: %.2f" % qr_centroY)

def new_qr_callback(msg):
    global new_qr  #Para corregir lo de que se centra en el mismo QR una vez que ya roto y no avanza
    new_qr = bool(msg.data)
    print ""
    print "   NUEVO QR detectado "
    print("                __                ")
    print("     ___( o)>       ")
    print("      \ <_. )                ")
    print("         `---'-'                    ")

    print ""
    print ""
   # rospy.loginfo("Nuevo QR Detectado Y: %.2f" % new_qr)


def angulo_callback(msg):
    global angulo
    angulo = float(msg.data)
    rospy.loginfo("Angulo : %.2f" % angulo)




#-------- street -------------------
def angular_callback(msg):
    global angular

    angular = int(msg.data)
    rospy.loginfo("angular : %.2f" % angular)


def horizontal_callback(msg):
    global  horiz

    horiz = int(msg.data)
    #rospy.loginfo("horizontal:%2f" % horiz)




#------- MISION 1 ---------------

def m1_x_callback(msg):
    global m1_x

    m1_x = int(msg.data)

    #print str(m1_x)

def m1_y_callback(msg):
    global m1_y

    m1_y = int(msg.data)
    #print str(m1_y)



#-----------------------------------------

def tower_bool_callback(msg):
    global tower_bool

    tower_bool = bool(msg.data)
    print "TOWER, qr detected",  tower_bool


def blue_x_callback(msg):
    global blue_x
    blue_x = int(msg.data)

def blue_y_callback(msg):
    global blue_y
    blue_y = int(msg.data)


def maq_estados(mision):
    global hz, m1_x, m1_y, stopAll, contador, cruzCentrado, paqSoltado, time_inicial, time_actual, new_qr, qr_centroX, qr_centroY, misionFinalizada
    global bool_rotation, bool_QRdetectado, bool_QRcentrado, bool_horizCentrado, horiz
    global apiFrameOff, apiFrameOn, ser, tower_bool, altura, estado, pos_z, pos_x, pos_y, ang_z

    if mision == 1 and paqSoltado == False:
        pub = rospy.Publisher('reset', Bool, queue_size=10) #cerrar programa de QR'S
        pub.publish(True)

        print ("Haciendo mision 1")
        if m1_x != 0 and m1_y != 0:
            print str(m1_x)
            print str(m1_y)
            cmd_vel.linear.x = pos_x.calcular(3 * cam_h // 4, int(m1_y))  # 3 cuchareado
            cmd_vel.linear.y = pos_y.calcular(cam_w // 2, int(m1_x))
            cmd_vel.linear.z = 0
            cmd_vel.angular.z = 0


            cv2.line(frame, (0, 2 * cam_h // 3), (cam_w, 2 * cam_h // 3), (255, 0, 255), 3)
            cv2.line(frame, (0, cam_h - 1), (cam_w, cam_h - 1), (255, 0, 255), 3)
            cv2.line(frame, (cam_w // 3, 0), (cam_w // 3, cam_h), (0, 255, 255), 3)
            cv2.line(frame, (2 * cam_w // 3, 0), (2 * cam_w // 3, cam_h), (0, 255, 255), 3)


            if int(m1_y) > 2 * cam_h // 3 and int(m1_y) < cam_h and \
                    int(m1_x) > cam_w // 3 and int(m1_x) < 2 * cam_w // 3:

                print('''
                   X      
                   X      
                XXXXXXX   
                   X      
                   X      
                ''')

                print ("Centrado en CRUZ roja")
                cruzCentrado = True

                if contador == 0:
                    time_inicial = Time.now().to_sec()
                    contador = contador + 1
                    print "TIME INICIAL " + str(time_inicial)

            else:
                contador = 0
                time_inicial = 0
                time_actual = 0
                cruzCentrado = False

            if cruzCentrado == True:
                time_actual = Time.now().to_sec()
                print "Time ACTUAL " + str(time_actual)

            if time_actual - time_inicial >= 3:
                print "SOLTAR PAQUETE"

                land_pub.publish(land)

                """
                ser.open()

                print(ser.is_open)
                print ("Estoy prendido")
                ser.write(apiFrameOn)
                time.sleep(4)   #Esperar 4 segundos mientras abre el servo
                ser.write(apiFrameOff)
                ser.close()
                
                """





                #paqSoltado = True


            else:
                paqSoltado = False

                print("Aun no puede regresar")
                stopAll = True

            if paqSoltado == True:
                print "Rotar 180"
                for i in range(0, 8):
                    cmd_vel.angular.z = - 1
                    print ('Gire der')
                    cmd_vel_pub.publish(cmd_vel)
                    time.sleep(1)




                #new_qr = False


                qr_centroX = 0
                qr_centroY = 0

                pub = rospy.Publisher('reset', Bool, queue_size=10)
                pub.publish(False)

                misionFinalizada = mision    #TERMINE LA MISION 1

                print "MISION FINALIZADA"
                print "                                     "
                print "                   1                  "
                print "                                     "
                print "--------------"

                #Regresar a como estaba

                bool_rotation = False
                bool_QRcentrado = False
                bool_QRdetectado = False
                bool_horizCentrado = False
                horiz = cam_w // 2

                stopAll = False #Regresar a la navegacion

        else:
            cmd_vel.linear.x = 0.1
            print ("Avanzar")





    elif mision == 2:
        print "HACIENDO LA MISION 2"


        print('NO agaches la cabeza rey por que se te cae la corona')
        cmd_vel_cam.angular.y = 0
        cmd_vel_cam_pub.publish(cmd_vel_cam)


        if estado == 0:
            if float(altura) > 1.7 and float(altura) < 1.8:
                estado = 1
            else:
                print ("Ir pa arriba")
                print "                                                         ALTURA     ", str(altura)
                cmd_vel.linear.x = 0;
                cmd_vel.linear.y = 0;
                cmd_vel.angular.z = 0;
                cmd_vel.linear.z = pos_z.calcular(1.75, altura)

        elif estado == 1:
            print "Mover Camara a  Izq: "
            cmd_vel_cam.angular.z = 50
            cmd_vel_cam_pub.publish(cmd_vel_cam)

            if tower_bool == False:
                print "Mover Camara a  Der: "
                cmd_vel_cam.angular.z = 0
                cmd_vel_cam_pub.publish(cmd_vel_cam)

            else:
                print "Ya lo vi"
                estado = 2


        elif estado == 2:
            if float(altura) > 0.6 and float(altura) < 0.8:
                print "MISION FINALIZADA"

            else:
                print ("Ir pa ABAJO")
                print "                                                         ALTURA     ", str(altura)
                cmd_vel.linear.x = 0;
                cmd_vel.linear.y = 0;
                cmd_vel.angular.z = 0;
                cmd_vel.linear.z = pos_z.calcular(0.7, altura)



        """
        if tower_bool == False:
            print ("Ir pa arriba")
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            cmd_vel.angular.z = 0;
            cmd_vel.linear.z = pos_z.calcular(1.75,altura)

        else:
            print "Ya lo vi"
            land_pub.publish(land)
        
        """





    elif mision == 3:
        print "HACIENDO LA MISION 3"
        estado = 0
        tower_bool = False

        print('NO agaches la cabeza rey por que se te cae la corona')
        cmd_vel_cam.angular.y = 0
        cmd_vel_cam_pub.publish(cmd_vel_cam)


        if estado == 0:
            if float(altura) > 1.7 and float(altura) < 1.8:
                estado = 1
            else:
                print ("Ir pa arriba")
                print "                                                         ALTURA     ", str(altura)
                cmd_vel.linear.x = 0;
                cmd_vel.linear.y = 0;
                cmd_vel.angular.z = 0;
                cmd_vel.linear.z = pos_z.calcular(1.75, altura)

        elif estado == 1:
            print "Mover Camara a  Izq: "
            cmd_vel_cam.angular.z = 50
            cmd_vel_cam_pub.publish(cmd_vel_cam)

            if tower_bool == False:
                print "Mover Camara a  Der: "
                cmd_vel_cam.angular.z = 0
                cmd_vel_cam_pub.publish(cmd_vel_cam)

            else:
                print "Ya lo vi"
                estado = 2


        elif estado == 2:
            if float(altura) > 0.6 and float(altura) < 0.8:
                print "MISION FINALIZADA"

            else:
                print ("Ir pa ABAJO")
                print "                                                         ALTURA     ", str(altura)
                cmd_vel.linear.x = 0;
                cmd_vel.linear.y = 0;
                cmd_vel.angular.z = 0;
                cmd_vel.linear.z = pos_z.calcular(0.7, altura)



        """
        
                print('NO agaches la cabeza rey por que se te cae la corona')
        cmd_vel_cam.angular.y = 0
        cmd_vel_cam_pub.publish(cmd_vel_cam)

        if tower_bool == False:
            print ("Ir pa arriba")
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            cmd_vel.angular.z = 0;
            cmd_vel.linear.z = pos_z.calcular(1.75,altura)

        else:
            print "Ya lo vi"
            land_pub.publish(land)
        """





    elif mision == 4:
        print "HACIENDO LA MISION 4"




        if blue_x != 0 and blue_y != 0:

            cmd_vel.linear.x = pos_x.calcular(1 * cam_h // 2, blue_y)
            cmd_vel.linear.y = pos_y.calcular(cam_w // 2, blue_x)
            cmd_vel.linear.z = 0
            cmd_vel.angular.z = 0

            cv2.line(frame, (0, 2 * cam_h // 3), (cam_w, 2 * cam_h // 3), (255, 0, 255), 3)
            cv2.line(frame, (0, cam_h - 1), (cam_w, cam_h - 1), (255, 0, 255), 3)
            cv2.line(frame, (cam_w // 3, 0), (cam_w // 3, cam_h), (0, 255, 255), 3)
            cv2.line(frame, (2 * cam_w // 3, 0), (2 * cam_w // 3, cam_h), (0, 255, 255), 3)

            if int(blue_y) > 1 * cam_h // 2 and int(blue_y) < cam_h and \
                    int(blue_x) > cam_w // 3 and int(blue_x) < 2 * cam_w // 3:

                print("  / \\__")
                print(" (    @\\___")
                print(" /         O       ")
                print("/   (_____/")
                print("/_____/   U")

                print ("Centrado en H   ->      PERRO")

                bool_centrado = True
                if contador == 0:
                    time_inicial = Time.now().to_sec()
                    contador = contador + 1
                    print "TIME INICIAL " + str(time_inicial)

            else:
                bool_centrado = False
                contador = 0
                time_inicial = 0
                time_actual = 0

            if bool_centrado == True:
                time_actual = Time.now().to_sec()
                print "Time ACTUAL " + str(time_actual)

            if time_actual - time_inicial >= 5:
                print "Bajese mi REY"
                land_pub.publish(land)

            else:

                print("Aun no puede ATERRIZAR")





        else:
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            cmd_vel.linear.z = 0;
            cmd_vel.angular.z = 0;


def camaraCallback(msg):
    global sistema_armado, cam_w, cam_h, landing, camara_armada, estado, kernel, avance_at, x_real, y_real, dron_x, dron_y, dron_th, altura
    global frame, thresh
    global rotation, bool_rotation, bridge_area, mision, bool_mision, angular, horiz, qr_centroX, qr_centroY, bool_QRdetectado, bool_horizCentrado, bool_QRcentrado
    global new_qr, contadorQR, contador, time_inicial, time_actual, angulo, centrado2, pos_x , pos_y, pos_z, ang_z
    global stopAll, misionFinalizada, tower_bool

    #Used to convert between ROS and OpenCV images
    br = CvBridge()

    # Convert ROS Image message to OpenCV image
    current_frame = br.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    # Process image
    frame = current_frame



    #print str(frame.shape[0])   #480    height
    #print  str(frame.shape[1])  #856    width

    if sistema_armado == 1:

        cv2.putText(frame, "ARMADO",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1, (0, 0, 255), 3)

        """
        1.-     Si ve un numero de mision, hacer mision
        2.-     Si ve un QR, detenerse, rotar y  levantar la camara
        3.-     Si ve un puente, esquivar
        4.-     Si ve una calle, centrarse horizontalmente
        5.-     SI esta centrada HRZ, centrarse verticalmente 
        
        

        if int(mision) != 0 and bool_mision == True:
            print ('m = ' , str(mision))
            
            maq_estados(mision)

        """



        if paqSoltado:
            pub = rospy.Publisher('paqSoltado', Bool, queue_size=10)
            pub.publish(True)


        ##AQUI LE MOVI DE 1 A 0
        if int(mision)> 0 and bool_mision == True and stopAll == True and mision >misionFinalizada :

            print ('m = ', str(mision))
            maq_estados(mision)
            #stopAll = True


        if stopAll == False:#sI ESTA HACIENDO UNA MISION, STOP ALL


             if float(altura) < 0.4 or float(altura) > 0.65:
                cmd_vel.linear.x = 0
                cmd_vel.linear.y = 0
                cmd_vel.linear.z = pos_z.calcular(0.5, altura)
                cmd_vel.angular.z = 0
                print "Poniendome a la ALTURA perro  ------------------------------------"
                print altura

             else:

                #Centrase en un QR nuevo
                if bool_QRdetectado and new_qr == True :#and contadorQR >= 1 :
                    #bool_horizCentrado = True


                    #MOdificar PID para que se centre mas despacio (regresarlo al final al valor original)
                    pos_x = PID(0.0002, 0.000, 0.0007)
                    pos_y = PID(0.0001, 0.000, 0.0007)
                    pos_z = PID(0.1, 0.000, 0.0006)
                    ang_z = PID(0.001, 0.000, 0.006)

                    print ('cX = ', str(qr_centroX))
                    print ('cY = ', str(qr_centroY))
                    cmd_vel.linear.x =  pos_x.calcular(cam_h // 2 + 50, int(qr_centroY)) #3 cuchareado
                    cmd_vel.linear.y = pos_y.calcular(cam_w // 2, int(qr_centroX))
                    cmd_vel.linear.z = 0
                    cmd_vel.angular.z = 0

                    print "######################3##"
                    print  " CMDVEL Linear X", cmd_vel.linear.x
                    print "######################3##"


                    #contador = 0    # para que solo tome el tiempo inicial una vez

                    #time_inicial = 0
                    #time_actual = 0

                   # bool_QRdetectado = False
                    #print ("Centrandose en QR papa")

                    cv2.line(frame, (0, 1* cam_h // 2), (cam_w, 1 * cam_h // 2), (255, 0, 255), 3)
                    cv2.line(frame, (0,  5 * cam_h//6 ), (cam_w,   5 * cam_h//6), (255, 0, 255), 3)
                    cv2.line(frame, (cam_w // 3 + 50, 0), (cam_w // 3 + 50, cam_h), (0, 255, 255), 3)
                    cv2.line(frame, (2 * cam_w // 3 - 50, 0), (2 * cam_w // 3 - 50, cam_h), (0, 255, 255), 3)

                    if int(qr_centroY) >1* cam_h // 2 and int(qr_centroY) <  5 * cam_h//6  and \
                            int(qr_centroX) > cam_w // 3 + 50 and int(qr_centroX) < 2 * cam_w // 3 - 50:


                        print("  / \\__")
                        print(" (    @\\___")
                        print(" /         O       ")
                        print("/   (_____/")
                        print("/_____/   U")

                        print ("Centrado en QR PERRO")



                        bool_QRcentrado = True
                        if contador == 0:
                            time_inicial = Time.now().to_sec()
                            contador = contador + 1
                            print "TIME INICIAL " + str(time_inicial)

                    else:
                        bool_QRcentrado = False
                        contador = 0
                        time_inicial = 0
                        time_actual = 0


                    if bool_QRcentrado == True:
                        time_actual = Time.now().to_sec()
                        print "Time ACTUAL " + str(time_actual)


                    if time_actual - time_inicial >= 3:
                            new_qr = False
                            contadorQR = int(contadorQR + 1)
                            bool_QRdetectado = False
                            bool_rotation = True    #bool_rotation es cuando ya pueda rotar

                            contador = 0
                            time_inicial = 0
                            time_actual = 0


                             #Ya que se centro, regresar PID
                            pos_x = PID(0.0005, 0.000, 0.0006)
                            pos_y = PID(0.0001, 0.000, 0.0006)
                            pos_z = PID(0.1, 0.000, 0.0006)
                            ang_z = PID(0.001, 0.000, 0.006)



                    else :
                        bool_rotation = False


                        print("Aun no puede rotar")




                #   Las rotaciones estas todas cuchareadas, pero jala bien
                #   1 rotacion = 1/3 de 90 deg
                elif int(rotation) != 0 and bool_rotation == True and bool_QRcentrado == True:
                    cmd_vel.linear.x = 0
                    cmd_vel.linear.y = 0
                    cmd_vel.linear.z = 0
                    cmd_vel.angular.z = 0

                    print (' R  =' , str(rotation))

                    print ""
                    print ""
                    print ""
                    print ""
                    print "   VOY A ROTAR"
                    print ""
                    print ""
                    print ""
                    print ""
                    print ""


                #Ajustes Finales
                    #corregir las rotaciones
                    #aventarse para adelante usando un for despues de rotar con la opcion de aterrizar
                    #ver si en la X y la H no detecta 2 centroides, si si, subir la altura



                    #i = 1
                    if int(rotation) > 0:      #6      # izquierda
                        for i in range(0,45 *int(rotation)):    #Se requieren mas o menos 3 pulsaciones para rotar 90grados
                                                                                            #eso se multiplica por el numero de veces que se va a rotar

                            cmd_vel.linear.x = 0
                            cmd_vel.linear.y = 0
                            cmd_vel.linear.z = 0
                            cmd_vel.angular.z = 1   #0.7


                            print ('Gire izq')
                            cmd_vel_pub.publish(cmd_vel)
                            time.sleep(0.02)
                        #time.sleep(2)

                #tenia un 4
                    elif int(rotation) <  0:     #derecha
                            for i in range(0,-45*int(rotation)):

                                cmd_vel.linear.x = 0
                                cmd_vel.linear.y = 0
                                cmd_vel.linear.z = 0
                                cmd_vel.angular.z = - 1
                                print ('Gire der')
                                cmd_vel_pub.publish(cmd_vel)
                                time.sleep(0.02)
                            #time.sleep(2)




                    print "############"
                    print ""
                    print "YA ROTE JEFE"
                    print ""
                    print(" /\_/\ ")
                    print("( o.o )")
                    print(" > ^ < ")

                    print "############"



                    
                    for i in range(1,10):

                        cmd_vel.linear.x = 0.3
                        cmd_vel.linear.y = 0
                        cmd_vel.linear.z = 0
                        cmd_vel.angular.z = 0
                        print ('Tengo que avanzar pa adelante')
                        cmd_vel_pub.publish(cmd_vel)
                        time.sleep(0.8)

                        print "entre en el for"

                        if cv2.waitKey(1) == ord("i") or ord("j") or ord("k") or ord("l") or ord(" ") :
                             sistema_armado = 0
                             cmd_vel.linear.x = 0
                             cmd_vel.linear.y = 0
                             cmd_vel.linear.z = 0
                             cmd_vel.angular.z = 0









                    """
                                    if int(mision)> 0 and bool_mision == True and mision >misionFinalizada:
                        stopAll = True
                    """


                    bool_rotation = False
                    bool_QRcentrado = False
                    bool_QRdetectado = False
                    bool_horizCentrado = False
                    horiz = cam_w // 2
                    #time.sleep(5)



                    """CENTRARSE Horizontalmetne mientras se centra angular usando street_align y moment_ align"""
                    #NOTA: Si el angulo es negativo nuestro calcular debe tener el signo cambiado para que llegue a 90 y no se de la vuelta por el otro lado



                elif int(mision)> 0 and bool_mision == True and mision >misionFinalizada:
                        stopAll = True
                        print "STOP ALL "


                elif  contadorQR >= 1 :

                    if int(horiz) > 0 :  # and new_qr == False:



                        print ('Horiz = ', str(horiz))
                        cv2.line(frame, (horiz, 0), (horiz, cam_h), (0, 255, 255), 3)  # trayectoria del dron
                        cmd_vel.linear.x = 0.03
                        cmd_vel.linear.y = pos_y.calcular(cam_w // 2, int(horiz))
                        cmd_vel.linear.z = 0
                        # bool_horizCentrado = True
                        print ("Centrandome Horizontal ")






                        if float(angulo) < 0:
                            cmd_vel.angular.z = -ang_z.calcular(90, -float(angulo))    #-88   ->  88  -> 90 , como es negativo, girar a la derecha o sea - angular.z
                        else:
                            cmd_vel.angular.z = ang_z.calcular(90, float(angulo))

                        print cmd_vel.angular.z



                    """
                            #va a avanzar hacia adelante antes de ver el 1er QR
                    elif contadorQR == 0 and bool_QRdetectado == False:
                            print ('Contador de QR ', contadorQR)
                            print ('Avanzar -->   1')   #antes del primer QR
                            cmd_vel.linear.x = 0.01
                            cmd_vel.linear.y = 0
                            cmd_vel.linear.z = 0
                            cmd_vel.angular.z = 0
                    
                    """



                else:
                        print ('Avanzar')
                        cmd_vel.linear.x = 0.03
                        cmd_vel.linear.y = 0
                        cmd_vel.linear.z = 0
                        cmd_vel.angular.z = 0





    else:
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.linear.z = 0;
        cmd_vel.angular.z = 0;



    cv2.imshow('MASTER', frame)
    cv2.moveWindow("MASTER" , 0,0)

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
            cmd_vel.linear.z = 0.2


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
        for i in range(1, 5):
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

    bridge_sub_py = rospy.Subscriber('bridge_area', String, area_callback)

    pub = rospy.Publisher('reset', Bool, queue_size=10)
    pub.publish(False)

    #pub = rospy.Publisher('paqSoltado', Bool, queue_size=10)
    #pub.publish(False)

    qr_sub_py = rospy.Subscriber('num_rotations', String, rotation_callback )
    qr_sub_py = rospy.Subscriber('num_mision', String, mision_callback)
    qr_sub_py = rospy.Subscriber('qr_centro_x' , String, qr_centroX_callback)
    qr_sub_py = rospy.Subscriber('qr_centro_y' , String, qr_centroY_callback)
    qr_sub_py = rospy.Subscriber('new_qr',Bool, new_qr_callback)


    #street_sub_py = rospy.Subscriber('align_angular1', String, angular_callback )
    street_sub_py = rospy.Subscriber('angulo', String, angulo_callback)
    moment_sub_py = rospy.Subscriber('align_horiz', String, horizontal_callback)

    mision1_sub_py = rospy.Subscriber('m1_x', String, m1_x_callback)
    mision1_sub_py = rospy.Subscriber('m1_y', String, m1_y_callback)

    tower_sub_py = rospy.Subscriber('tower_bool', Bool, tower_bool_callback)

    h_sub_py = rospy.Subscriber('blue_x', String, blue_x_callback)
    h_sub_py = rospy.Subscriber('blue_y', String, blue_y_callback)


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





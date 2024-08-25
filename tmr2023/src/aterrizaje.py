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
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
arucoParams = cv2.aruco.DetectorParameters_create()
#Matrices de la camara
mtx=np.array([[537.292878, 0.000000, 427.331854], [0.000000, 527.000348, 240.226888], [0.000000, 0.000000, 1.000000]])
dist=np.array([0.004974, -0.000130, -0.001212, 0.002192, 0.000000])
global cam_w, cam_h
cam_w = 856
cam_h = 480

def Hminimo(val):
    pass
    
def Hmaximo(val):
    pass


#Descomentar para modificar la mascara y los parametros de los circulos
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


#inicio de PIDs 
pos_x = PID(0.0005, 0.000, 0.0006)
pos_y = PID(0.001, 0.000, 0.0006)
pos_z = PID(0.5, 0.000, 0.0006)
ang_z = PID(0.001, 0.000, 0.0006)


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
global sistema_armado, camara_armada, estado, altura, dron_x, dron_y, dron_th, th_ini, teclas_control, avance_at
avance_at = [0, 0]
teclas_control = {ord("i"),ord("j"), ord("k"), ord("l")}
altura = 0
estado = 0
sistema_armado = 0
camara_armada = 0

#Variables de odometria
global dron_x, dron_y, dron_th


def altitudeCallback(msg):
    global altura
    altura = msg.altitude
    

def odometryCallback(msg):
    global dron_x, dron_y, dron_th
    dron_x = msg.pose.pose.position.x
    dron_y = msg.pose.pose.position.y
    dron_th = msg.pose.pose.orientation.z * math.pi

def camaraCallback(msg):
    global sistema_armado, camara_armada, estado, altura, th_ini, dron_th, teclas_control, avance_at
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    #Se le coloca un filtro HSV a la imagen
    cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    #Se aplica una mascara de negros a la imagen
    negroClaro = np.array([0, 0, 0], np.uint8)
    negroOscuro = np.array([121, 115, 68], np.uint8)
    bin = cv2.inRange(frame,negroClaro,negroOscuro)
    #Se usa la funcion SimpleBlobDetector para identificar los cuadrados
    #Se dibujaran dos circulos sobre los cuadrados
    params = cv2.SimpleBlobDetector_Params()

    #Los circulos se dibujaran sobre las formas que cumplan los siguientes parametros:
    
    # Set Area filtering parameters
    params.filterByArea = True
    params.minArea = 3000
    params.maxArea = 17000
             
    # Set Circularity filtering parameters
    params.filterByCircularity = True
    params.minCircularity = 0
    params.maxCircularity = 1
             
    # Set Convexity filtering parameters
    params.filterByConvexity = False
    params.minConvexity = 0
                 
    # Set inertia filtering parameters
    params.filterByInertia = False
    params.minInertiaRatio = 0
             
    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)
                 
         
        # Reverting back to the original image,
        # with optimal threshold value
    #frame[bin > 0.01 * bin.max()]=[0, 0, 255]
    
            #if pixel > 0:
                #cv2.circle(frame, pixel, 4, (0, 0, 255), -1)
    
    
    #bin =cv2.erode(bin, kernel)

    # Detect blobs
    keypoints = detector.detect(frame)

    #diametros = list()
    #coord = list()
    puntos = list()
    
    cx = cam_w / 2
    cy = cam_h / 2
    detectado = False
    
    
    #Se extraen datos de los keypoints encontros para su uso posterior
    for point in keypoints:
        puntos.append([point.size, point.pt])
    #Si el arreglo de puntos tiene elementos y el estado es diferente al 0, se detecta el centro que hay entre los dos circulos, el cual es el centro de la pista de aterrizaje 
    if len(puntos) > 0 and estado != 0:
        puntos = sorted(puntos, key = lambda puntos: puntos[0])
        cuad_marcador = [0, [0, 0]]
        cuad_marcador[0] = 0
        ""
        for punto in puntos:
            print punto[0]
            if (punto[0] < cuad_marcador[0] + cuad_marcador[0]*(0.2 + ((2-altura)/10)) and (punto[0] > cuad_marcador[0] - cuad_marcador[0]*(0.2+ ((2-altura)/10)))):
                if avance_at[0] == 0: 
                    estado = 2
                cx = (cuad_marcador[1][0] + punto[1][0])/2
                cy = (cuad_marcador[1][1] + punto[1][1])/2
                cv2.circle(frame, (int(cx),int(cy)), 1, (0, 255, 0), 4)
                detectado = True
                break
            else:
                cuad_marcador = punto
        
    # Draw blobs on our image as red circles
    blank = np.zeros((1, 1))
    frame = cv2.drawKeypoints(frame, keypoints, blank, (0, 0, 255),
            cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    cv2.imshow('bin', bin)
    
    #Aqui se hace un calculo para generarle un nuevo sistema de ejes al dron dependiendo de la direccion en la que este mirando el dron.
    if estado != 3:
        cmd_vel.linear.z = pos_z.calcular(2, altura)
    x_real = (math.cos(dron_th) * dron_x) + (math.sin(dron_th) * dron_y)

    y_real = (-math.sin(dron_th) * dron_x) + (math.cos(dron_th) * dron_y)
    #if dron_th > math.pi/2 or dron_th < -math.pi/2:
    
    
    print "x_real ", x_real
    print "y_real ", y_real
    print ""
    
    #MAQUINA DE ESTADOS
    
    print "                                   estado ", estado
    
    #En el estado 0 el dron se eleva a una altura de dos metros.
    if estado == 0:
        if altura > 1.9 and altura < 2.1:
            estado = 1
        else:
            #Al llegar a los dos metros se le ordena que se detenga en ese punto.
            print "             altura:", altura
            cmd_vel.linear.x = 0
            cmd_vel.linear.y = 0
            cmd_vel.angular.z = 0
            
    #Una vez alcanzada la altura de dos metros, el dron avanza en linea recta hacia adelante.
    elif estado == 1:
        cmd_vel.linear.x = 0.01
        
    #Pasa al estado 2 cuando el dron detecta los circulos en la pista de aterrizaje.
    elif estado == 2:
        print "               Detectado: ", detectado
        if detectado == True:
            #Se calcula el PID.
            cmd_vel.linear.x = pos_x.calcular(cam_h - cam_h/3, cy)
            cmd_vel.linear.y = pos_y.calcular(cam_w/2, cx)
        
            #Se centra la camara en el punto central de la pista y se acerca poco a poco.
            if cy < cam_h - (cam_h/3) + 50 and cy > cam_h - (cam_h/3) - 30:
                if  cx < cam_w/2 + 50 and cx > cam_w/2 - 50:
                    print "Centrado papa"
                    print altura
                    avance_at[0] = x_real
                    avance_at[1] = y_real
                    estado = 3
                    pos_x.kp = 0.1
                    pos_y.kp = 0.1
                    
    #Una vez centrado el dron avanzara una distancia de 0.9 y se le ordena detenerce y aterrizar.
    elif estado == 3:
        if x_real < avance_at[0] + 1 and x_real > avance_at[0] + 0.7:
            cmd_vel.linear.x = 0
            cmd_vel.linear.y = 0
            cmd_vel.linear.z = 0
            land_pub.publish(land)
        else:
            #Aqui se le ordena avanzar una distancia de 0.9 en x.
            print "actual ", x_real, " hasta ", avance_at[0] + 0.7, "x real ", x_real
            cmd_vel.linear.x = pos_x.calcular(avance_at[0] + 0.9, x_real)  
            cmd_vel.linear.y = pos_y.calcular(avance_at[1], y_real)
            
        
    cv2.imshow("Bebop", frame)
    
    #Comando para iniciar el programa.
    if sistema_armado == 1:
        cmd_vel_pub.publish(cmd_vel)
    else:
        cmd_vel.linear.x = 0
        cmd_vel.linear.y = 0
        cmd_vel.linear.z = 0
        cmd_vel.angular.z = 0
        cmd_vel_pub.publish(cmd_vel)
        
    tecla_opencv = cv2.waitKey(1)
 
 
 #MUY IMPORTANTE
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
        if tecla_opencv == ord(" ") :
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
                #Camara gira hacia abajo 
                cmd_vel_cam.angular.y = -60
            else:
                cmd_vel_cam.angular.y = 0
            cmd_vel_cam_pub.publish(cmd_vel_cam)
                
def bebop():
    global dron_x, dron_y, dron_th, tiempo_anterior
    #cam = cv2.VideoCapture(0)
    #Configuracion de ROS
    rospy.init_node('objeto', anonymous=False)
    #camara_pub = rospy.Publisher('/camara', Image, queue_size=10)
    camara_sub = rospy.Subscriber("/bebop/image_raw", Image, camaraCallback)
    altitude_sub = rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AltitudeChanged", Ardrone3PilotingStateAltitudeChanged, altitudeCallback)
    odometry_sub = rospy.Subscriber("/bebop/odom", Odometry, odometryCallback)
    
    
    pos_x.tiempo_anterior = Time.now().to_sec()
    pos_y.tiempo_anterior = Time.now().to_sec()
    pos_z.tiempo_anterior = Time.now().to_sec()
    ang_z.tiempo_anterior = Time.now().to_sec()
    tiempo_anterior = Time.now().to_sec()
    

    
    rate = rospy.Rate(24) # 10hz
    while not rospy.is_shutdown():
        #_, frame = cam.read()
        #cv_image = bridge.cv2_to_imgmsg(frame, encoding="passthrough")
        #camara_pub.publish(cv_image)
        #print("No mamo")
        
        
        
        rate.sleep()
        #rospy.spin()
        

if __name__ == '__main__':
    try:
        bebop()
    except rospy.ROSInterruptException:
        #cam.release()
        pass

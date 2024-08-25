#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
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


#inicio de PIDs 
pos_x = PID(0.001, 0.000, 0.0006)
pos_y = PID(0.001, 0.000, 0.0006)
pos_z = PID(0.001, 0.000, 0.0006)
ang_z = PID(0.005, 0.000, 0.0)

#Configuracion de ROS Y variables de ROS
cmd_vel_pub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=1)
cmd_vel = Twist()
takeoff_pub = rospy.Publisher("/bebop/takeoff", Empty, queue_size=1)
land_pub = rospy.Publisher("/bebop/land", Empty, queue_size=100)
land = Empty()
takeoff = Empty()


tecla_opencv = 0
global sistema_armado
sistema_armado = 0


def camaraCallback(msg):
    global sistema_armado, cam_w, cam_h
    #Convierte la imagen proveniente de ROS a una imagen que OpenCV puede usar
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    #ewcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx,dist,(cam_h,cam_w),0,(cam_h,cam_w))
    #dst1 = cv2.undistort(frame,mtx,dist,None,newcameramtx)
    #x, y, cam_w, cam_h = roi
    #dst1 = dst1[y:y+cam_h, x:x+cam_w]
    #frame = dst1
    
    #Detecta los marcadores de aruco
    (corners, ids, rejected) = cv2.aruco.detectMarkers(gray, arucoDict, parameters=arucoParams)
    
    # Checa que exista algun marcador para realizar el proceso
    if len(corners) > 0:
        # flatten the ArUco IDs list
        ids = ids.flatten()
        # loop over the detected ArUCo corners
        for (markerCorner, markerID) in zip(corners, ids):
            if markerID == 37:
                # extract the marker corners (which are always returned
                # in top-left, top-right, bottom-right, and bottom-left
                # order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners

                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorner, 0.05, mtx, dist)
                
                (rvec-tvec).any() 
                
                #Anadir PID A este angulo
                angulox = rvec[0][0][2] * 180/math.pi
                print(angulox)
                
                for i in range(rvec.shape[0]):
                    cv2.aruco.drawAxis(frame, mtx, dist, rvec[i,:,:], tvec[i,:,:], 0.03)
                    #cv2.aruco.drawDetectedMarkers(frame, markerCorner)
    
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
    
                # draw the bounding box of the ArUCo detection
                cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
    
                # compute and draw the center (x, y)-coordinates of the
                # ArUco marker
                cX = (topLeft[0] + bottomRight[0]) / 2.0
                cY = (topLeft[1] + bottomRight[1]) / 2.0
                
                
                intcX = int(cX)
                intcY = int(cY)
                cv2.circle(frame, (intcX, intcY), 4, (0, 0, 255), -1)
    
                # draw the ArUco marker ID on the frame
                cv2.putText(frame, str(markerID),
                    (topLeft[0], topLeft[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)
                
                #calculate the area
                #width
                w = sqrt(pow((topLeft[1]-topRight[1]),2)+pow((topLeft[0]-topRight[0]),2))
                #height
                h = sqrt(pow((topLeft[1]-bottomLeft[1]),2)+pow((topLeft[0]-bottomLeft[0]),2))
                #Regresion por potencias en excel
                distancia = 8038.8 * pow((h.real*w.real), -0.499)
                    
                if sistema_armado == 1:
                    cmd_vel.linear.x = -pos_x.calcular(150, distancia)
                    cmd_vel.linear.y = pos_y.calcular(cam_w/2, cX)
                    cmd_vel.linear.z = pos_z.calcular(cam_h/2, cY)
                    cmd_vel.angular.z = ang_z.calcular(0.0, angulox)
                else:
                    cmd_vel.linear.x = 0;
                    cmd_vel.linear.y = 0;
                    cmd_vel.linear.z = 0;
                    cmd_vel.angular.z = 0;
                
                cv2.putText(frame, str(distancia), 
                    (topRight[0], topRight[1]),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)
            else:
                cmd_vel.linear.x = 0;
                cmd_vel.linear.y = 0;
                cmd_vel.linear.z = 0;
                cmd_vel.angular.z = 0;
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
    
    cv2.imshow("Bebop", frame)
    cmd_vel_pub.publish(cmd_vel)
    
    tecla_opencv = cv2.waitKey(1)
    
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
            
        



def bebop():
    #cam = cv2.VideoCapture(0)
    #Configuracion de ROS
    rospy.init_node('objeto', anonymous=False)
    #camara_pub = rospy.Publisher('/camara', Image, queue_size=10)
    camara_sub = rospy.Subscriber("/bebop/image_raw", Image, camaraCallback)
    
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

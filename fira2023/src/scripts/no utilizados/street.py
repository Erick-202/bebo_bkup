#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np
import math
import time

# -------------------CENRARSE en la carretera usando LINEAS   ->  ya

# Calibrar area y thresh antes de usar

kernel = np.ones((5, 5), np.uint8)

cam_w = 856
cam_h = 480

ang_anterior = 0

estado = 0

threshValue = 60

def image_callback(data):
    global cam_w, cam_h, ang_anterior, estado
    global frame, thresh
    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    # Convert ROS Image message to OpenCV image
    current_frame = br.imgmsg_to_cv2(data, desired_encoding="bgr8")

    # Process image
    frame = current_frame

    # Output debugging information to the terminal
    # rospy.loginfo("receiving video frame")

    cX = cam_h//2
    cY = cam_w//2

    # Bordes 4-------------------------------------------------------------------------
    # Convertir a HSV
    # cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    x1, y1 = cam_w // 2 - 100, 0
    x2, y2 = cam_w // 2 + 100, cam_h

    roi = frame[y1:y2, x1:x2]



    gray = cv2.cvtColor(roi,cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, threshValue, 255, cv2.THRESH_BINARY_INV) #120 en el gymnasio, 50 en el cap

    """
    
    # Se aplica una mascara de negros a la imagen
    negroClaro = np.array([0, 0, 0], np.uint8)
    negroOscuro = np.array([121, 115, 68], np.uint8)

    # Se unen ambos rangos en una mascara
    maskCom = cv2.inRange(frame, negroClaro, negroOscuro)
    
    """




    # -----EROSION-----#
    eroded_maskBlack = cv2.erode(thresh, kernel, iterations=1)

    blur = cv2.medianBlur(eroded_maskBlack, 9)
    # dil = cv2.dilate(blur, kernel, iterations = 1)
    # dil = cv2.erode(blur, kernel, iterations=1)
    # kernel = np.ones((7,7), np.uint8)
    # erosion = cv2.erode(thresh,kernel,iterations=1)

    edges = cv2.Canny(blur, 50, 150, apertureSize=3)

    roi_edges = edges[y1:y2, x1:x2]


    cv2.rectangle(roi, (0, 0), (x2 - x1, y2 - y1), (255, 255, 0), 3)

    angular = 0  # Este dato se va a publicar al master para aliearse angularmente segun la pendiente de la mascara negra

    _, contornos, _ = cv2.findContours(eroded_maskBlack, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)



    for c in contornos:
        # print (c.dtype)
        area = cv2.contourArea(c)
        if area > 15000 and area < 50000:
            M = cv2.moments(c)
            if (M["m00"] == 0): M["m00"] = 1
            cX = int(M["m10"] / M["m00"])
            cY = int(M['m01'] / M['m00'])



            if estado == 0 and cX > cam_w // 2 - 100 and cX < cam_w // 2 + 100 and cY > cam_h//3 or cY < 2*cam_h//3:

                cv2.circle(roi, (cX, cY), 7, (0, 255, 0), 3)

                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(roi, '{},{}'.format(cX, cY), (cX + 10, cY), font, 0.75, (0, 255, 0), 1, cv2.LINE_AA)
                cv2.putText(roi, str(area), (cX, cY + 30), font, 0.75, (0, 0, 255), 1, cv2.LINE_AA)

                print (str(cX))
                # enviando a nodo maestro
                pub = rospy.Publisher('align_horiz', String, queue_size=10)  # 2 para enviar datos mas reales
                rate = rospy.Rate(50)  # 10hz
                # rospy.loginfo(ids)
                pub.publish(str(cX + cam_w//2 - 100))
                rate.sleep()


            else:
                #print ('centrado')
                estado = 1

            nuevoContorno = cv2.convexHull(c)
            cv2.drawContours(roi, [nuevoContorno], 0, (255, 0, 0), 3)


    if estado == 1:


        lines = cv2.HoughLinesP(roi_edges, 1, np.pi / 180, threshold=100, minLineLength=25, maxLineGap=35)

        # imgROI =frame [0:cam_h/2,  ]

        if lines != None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(roi, (x1, y1), (x2, y2), (0, 0, 255), 2)
                if y1 < 200 and y2 < 200 and x2 != x1:
                    try:
                        angulo = math.atan((y2 - y1) / (x2 - x1))
                    except:
                        angulo = math.pi / 2
                    angulo = angulo * (180 / math.pi)
                    angulo = int(angulo)

                    if ang_anterior ==  angulo:
                        print ('igual')
                    else:

                        print('ANGULOS : ', angulo)

                        if (angulo >= 85 or angulo <= -85) and angulo <= 90:
                            angular = 0

                            # print ('0')

                        elif angulo < 85 and angulo > 0:
                            angular = 1
                            # print ('1')


                        elif angulo > -85 and angulo < 0:
                            angular = -1
                            # print ('-1')

                        ang_anterior = angulo
                        print str(angular)
                        angulo = 0

                        # enviando a nodo maestro
                        pub = rospy.Publisher('align_angular', String, queue_size=10)    # 2 para enviar datos mas reales
                        rate = rospy.Rate(50)  # 10hz
                        # rospy.loginfo(ids)
                        pub.publish(str(angulo))
                        rate.sleep()

    if estado == 1 and (cX < cam_w // 2 - 100 or cX > cam_w // 2 + 100):    #SI la bolita se sale de la ROI, volver a alinearse
        estado = 0

    cv2.line(frame, (0, cY), (cam_w, cY), (0, 255, 0), 3)

    # cv2.line(frame, (cam_w // 2 - 50, 0), (cam_w // 2 - 50, cam_h), (255, 0, 0), 3)
    # cv2.line(frame, (cam_w // 2 + 50, 0), (cam_w // 2 + 50, cam_h), (255, 0, 0), 3)

    # if (cX > 0 and cY > 0):
    # print("Centrando")
    # cmd_vel.linear.x = pos_x.calcular(cam_h - cam_h/3, cY)
    # cmd_vel.linear.x = 0.1
    # cmd_vel.linear.y = pos_y.calcular(cam_w / 2, cX)

    """
    # Se centra la camara en el punto central de la pista y se acerca poco a poco.
            if cX < cam_w / 2 + 50 and cX > cam_w / 2 - 50:
                # if cY < cam_h - (cam_h/3) + 50 and cY > cam_h - (cam_h/3) - 30:
                print ("Centrado papa")

            # No utiliza como tal Y para centrarse, pero funciona para saber si ya esta llegando a un QR
            if cY < cam_h - (cam_h / 4) + 50 and cY > cam_h - (cam_h / 4) - 30:
                print ("Llegando a un QR")

    """

    """
                        # enviando a nodo maestro
                        pub = rospy.Publisher('num_rotations', String, queue_size=10)
                        rate = rospy.Rate(50)  # 10hz
                        # rospy.loginfo(ids)
                        pub.publish(str(calcular_NEWS(orientacionInicial, codigo_result[0])))
                        rate.sleep()

    """
    cv2.imshow("Region Of Interest", roi)
    #cv2.imshow("ROI edges", roi_edges)
    # cv2.imshow("Centroid Mask" , eroded_maskBlack)
    cv2.imshow("Street Centering", frame)
    #cv2.imshow("edges" , edges)

    cv2.waitKey(1)


def receive_message():
    # Tells rospy the name of the node.
    # Anonymous = True makes sure the node has a unique name. Random
    # numbers are added to the end of the name.
    print ('receive msg')
    rospy.init_node('street_sub_py', anonymous=True)

#-----  Estas publiaciones se hacen por que el master nunca lee el primer dato por alguna razon
    # enviando a nodo maestro
    pub = rospy.Publisher('align_angular', String, queue_size=10)
    rate = rospy.Rate(50)  # 10hz
    # rospy.loginfo(ids)
    pub.publish("Iniciando Comunicacion con mi Maestro")
    rate.sleep()

    # enviando a nodo maestro
    pub = rospy.Publisher('align_horiz', String, queue_size=10)
    rate = rospy.Rate(50)  # 10hz
    # rospy.loginfo(ids)
    pub.publish("Iniciando Comunicacion con mi Maestro2")
    rate.sleep()

#-------------------------------------------------------------

    # Node is subscribing to the video_frames topic
    rospy.Subscriber('/bebop/image_raw', Image, image_callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    # Close down the video stream when done
    cv2.destroyAllWindows()


if __name__ == '__main__':
    receive_message()
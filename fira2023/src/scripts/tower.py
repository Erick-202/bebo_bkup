#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import cv2

import pyzbar.pyzbar as pyzbar

# ----------------- Metodo QR deteccion ----------------- ->  Working details


QR_ant = ""

# 213 duela, 78 consejo
threshValue = 40

global hz  # rate hertz
hz = 200




def image_callback(data):
    global cam_w, cam_h, QR_ant, threshValue
    global frame, thresh

    # Used to convert between ROS and OpenCV images
    br = CvBridge()


    # Convert ROS Image message to OpenCV image
    current_frame = br.imgmsg_to_cv2(data, desired_encoding="bgr8")

    # Process image
    frame = current_frame

    # Output debugging information to the terminal
    # rospy.loginfo("receiving video frame")

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, threshValue, 255, cv2.THRESH_BINARY)  # 120 en el gymnasio, 50 en el cap

    #cv2.line(frame, (0, frame.shape[0] * 1 / 3), (frame.shape[1], frame.shape[0] * 1 / 3), (0, 255, 0), 3)

        # Buscar codigos QR en el frame
    codigos = pyzbar.decode(thresh)

    # Centrarse en linea
    # pa adelante

    for codigo in codigos:
        # if len(codigos[0]) > 0:
        # codigo = codigos[0]
        # Obtener las coordenadas del rectangulo del codigo QR
        x, y, w, h = codigo.rect

        centro_x = x + w // 2
        centro_y = y + h // 2

        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
        cv2.circle(frame, (centro_x, centro_y), 7, (0, 255, 0), 3)


        if y > 0:
            # Ahi mero
            # print ("DETENGASE prro")

            # print("El codigo QR esta debajo de la linea")
            if codigo.data.decode("utf-8") == QR_ant:
                # print "Dibujar QR"
                #print 'Ya lo habia leido'
                pass

            else:

                contenido = codigo.data.decode("utf-8")

                if len(contenido) != 9 :
                    print ""
                    print "CONTENIDO = " , contenido
                    print ""

                    pub = rospy.Publisher('tower_bool', Bool, queue_size=2)
                    pub.publish(True)


                else:
                    #print "Este es de la navegacion"
                    pass

                QR_ant = contenido


        rate = rospy.Rate(hz)  # 10hz
        rate.sleep()

    cv2.imshow("Tower Reading", frame)
    cv2.moveWindow("Tower Reading", 1200, 1200)
    # cv2.imshow("Thresh", thresh)

    cv2.waitKey(1)


def receive_message():
    global hz
    # Tells rospy the name of the node.
    # Anonymous = True makes sure the node has a unique name. Random
    # numbers are added to the end of the name.
    print ('receive msg')
    rospy.init_node('tower_sub_py', anonymous=True)

    # DATO BASURA---------- Se envia este dato por que por alguna razon, el master nunca recibe el primer dato

    # ------------  SUbscribe

    # Node is subscribing to the video_frames topic
    rospy.Subscriber('/bebop/image_raw', Image, image_callback)
    # spin() simply keeps python from exiting until this node is stopped

    rate = rospy.Rate(hz)  # 10hz
    rate.sleep()

    rospy.spin()

    # Close down the video stream when done
    cv2.destroyAllWindows()


if __name__ == '__main__':
    receive_message()

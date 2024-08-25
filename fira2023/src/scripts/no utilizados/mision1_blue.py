#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np
import math
import time

# -------------------CENRARSE en la cruz roja para dejar el paquete

# NOTA: para calibrar se usa el colorROS_ts.py   -->        por que quiero detectar los rojos pero uso la imagen asi sin convertir a BGR

cam_w = 856
cam_h = 480

kernel = np.ones((3, 3), np.uint8)

areaMin = 15000
areaMax = 40000

hz = 200


"""MASCARAS"""

"""
blue_low = np.array([100,100,20],np.uint8)
blue_high = np.array([125,255,255],np.uint8)

yellow_low = np.array([15,100,20],np.uint8)
yellow_high = np.array([45,255,255],np.uint8)

"""


red_low = np.array([95, 174, 126],np.uint8)
red_high = np.array([151, 255, 216],np.uint8)

"""
red_low2 = np.array([175,100,20],np.uint8)
red_high2 = np.array([179,255,255],np.uint8)

"""


font = cv2.FONT_HERSHEY_SIMPLEX

global x,y
x = 0
y = 0


def image_callback(data):
    global cam_w, cam_h, init_mision, x, y
    global frame, red_low, red_high
    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    # Convert ROS Image message to OpenCV image
    current_frame = br.imgmsg_to_cv2(data)

    # Process image
    frame = current_frame

    cX = cam_h // 2
    cY = cam_w // 2

    """NOTA: Si uso estos rango sin el  desired_encoding="bgr8", detecta rojo en vez de azul"""

    frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(frameHSV, red_low, red_high)


    eroded_Red= cv2.erode(mask, kernel, iterations=2)

    blur_red = cv2.medianBlur(eroded_Red,9 )



    _, contornos, _ = cv2.findContours(blur_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for c in contornos:
        area = cv2.contourArea(c)
        if area >= areaMin and area <= areaMax:

            M = cv2.moments(c)
            if (M["m00"] == 0): M["m00"] = 1
            x = int(M["m10"] / M["m00"])
            y = int(M['m01'] / M['m00'])
            nuevoContorno = cv2.convexHull(c)
            cv2.circle(frame, (x, y), 7, (0, 255, 0), -1)
            cv2.putText(frame, '{},{}'.format(x, y), (x + 10, y), font, 0.75, (0, 255, 0), 1, cv2.LINE_AA)
            cv2.drawContours(frame, [nuevoContorno], 0, (0,0,255), 3)

        rate = rospy.Rate(hz)  # 10hz

        pub = rospy.Publisher('m1_x', String, queue_size=10)
        pub.publish(str(x))

        pub = rospy.Publisher('m1_y', String, queue_size=10)
        pub.publish(str(y))




    cv2.imshow("Mision1", eroded_Red)
    cv2.moveWindow("Mision1", 0, 650)
    cv2.imshow("Frame RED", frame)

    rate.sleep()
    cv2.waitKey(1)


def receive_message():
    # Tells rospy the name of the node.
    # Anonymous = True makes sure the node has a unique name. Random
    # numbers are added to the end of the name.
    print ('receive msg')
    rospy.init_node('mision1_sub_py', anonymous=True)

    # -----  Estas publiaciones se hacen por que el master nunca lee el primer dato por alguna razon
    # enviando a nodo maestro

    rate = rospy.Rate(hz)  # 10hz

    pub = rospy.Publisher('m1_x', String, queue_size=10)
    pub.publish("Iniciando Comunicacion con mi Maestro")

    pub = rospy.Publisher('m1_y', String, queue_size=10)
    pub.publish("Iniciando Comunicacion con mi Maestro")

    rate.sleep()

    # -------------------------------------------------------------

    # Node is subscribing to the video_frames topic
    rospy.Subscriber('/bebop/image_raw', Image, image_callback)
    # spin() simply keeps python from exiting until this node is stopped

    rospy.spin()

    # Close down the video stream when done
    cv2.destroyAllWindows()


if __name__ == '__main__':
    receive_message()
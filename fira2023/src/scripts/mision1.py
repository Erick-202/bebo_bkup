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

# Calibrar area y thresh antes de usar

cam_w = 856
cam_h = 480

kernel = np.ones((3, 3), np.uint8)

areaMin = 1000
areaMax = 30000

hz = 200


"""MASCARAS"""

"""
blue_low = np.array([100,100,20],np.uint8)
blue_high = np.array([125,255,255],np.uint8)

yellow_low = np.array([15,100,20],np.uint8)
yellow_high = np.array([45,255,255],np.uint8)

"""

red_low1 = np.array([  0, 100, 20], np.uint8)
red_high1 = np.array([ 5, 255, 255], np.uint8)

red_low2 = np.array([154, 100, 20], np.uint8)
red_high2 = np.array([ 255, 255, 255], np.uint8)


font = cv2.FONT_HERSHEY_SIMPLEX

global x,y
x = 0
y = 0


def image_callback(data):
    global cam_w, cam_h, x, y
    global frame, blue_low, blue_high, yellow_low, yellow_high, red_low1, red_high1, red_low2, red_high2
    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    # Convert ROS Image message to OpenCV image
    current_frame = br.imgmsg_to_cv2(data, desired_encoding="bgr8")

    # Process image
    frame = current_frame

    cX = cam_h // 2
    cY = cam_w // 2

    frameHSV = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

    #maskAzul = cv2.inRange(frameHSV, blue_low, blue_high)
    #maskAmarillo = cv2.inRange(frameHSV, yellow_low, yellow_high)

    maskRed1 = cv2.inRange(frameHSV, red_low1, red_high1)
    maskRed2 = cv2.inRange(frameHSV, red_low2, red_high2)
    maskRed = cv2.add(maskRed1, maskRed2)

    eroded_Red= cv2.erode(maskRed, kernel, iterations=2)



    _, contornos, _ = cv2.findContours(eroded_Red, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

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
            cv2.putText(frame, str(area), (x, y + 30), font, 0.75, (255, 0, 0), 1, cv2.LINE_AA)
            cv2.drawContours(frame, [nuevoContorno], 0, (0,0,255), 3)

        else:
            x = 0
            y = 0



    pub = rospy.Publisher('m1_x', String, queue_size=2)
    pub.publish(str(x))

    pub = rospy.Publisher('m1_y', String, queue_size=2)
    pub.publish(str(y))



    rate = rospy.Rate(hz)  # 10hz
    rate.sleep()


    #cv2.imshow("Mision1", eroded_Red)
    #cv2.moveWindow("Mision1", 0, 650)
    cv2.imshow("Frame RED", frame)
    cv2.moveWindow("Frame RED", 0, 1200)

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

    pub = rospy.Publisher('m1_x', String, queue_size=2)
    pub.publish("Iniciando Comunicacion con mi Maestro")

    pub = rospy.Publisher('m1_y', String, queue_size=2)
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
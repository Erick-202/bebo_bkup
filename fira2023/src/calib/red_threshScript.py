#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np

# -------------------CALIBRAR   ROJO  ->  Jala bien


contador = 0




def Hminimo(val):
    pass


def Hmaximo(val):
    pass


def Hminimo2(val):
    pass


def Hmaximo2(val):
    pass


def createTrackbars():
    cv2.namedWindow("Trackbars 1")
    cv2.createTrackbar("Hmax", "Trackbars 1", 5, 255, Hminimo)
    cv2.createTrackbar("Hmin", "Trackbars 1", 0, 255, Hmaximo)
    cv2.createTrackbar("Smax", "Trackbars 1", 255, 255, Hminimo)
    cv2.createTrackbar("Smin", "Trackbars 1", 100, 255, Hmaximo)
    cv2.createTrackbar("Vmax", "Trackbars 1", 255, 255, Hminimo)
    cv2.createTrackbar("Vmin", "Trackbars 1", 20, 255, Hmaximo)

    cv2.namedWindow("Trackbars 2")
    cv2.createTrackbar("Hmax", "Trackbars 2", 179, 255, Hminimo2)
    cv2.createTrackbar("Hmin", "Trackbars 2", 175, 255, Hmaximo2)
    cv2.createTrackbar("Smax", "Trackbars 2", 255, 255, Hminimo2)
    cv2.createTrackbar("Smin", "Trackbars 2", 100, 255, Hmaximo2)
    cv2.createTrackbar("Vmax", "Trackbars 2", 255, 255, Hminimo2)
    cv2.createTrackbar("Vmin", "Trackbars 2", 20, 255, Hmaximo2)


def image_callback(data):
    global cam_w, cam_h, contador, MaskLI1, MaskLI2, MaskLS1, MaskLS2
    global frame, thresh

    if contador == 0:
        createTrackbars()

        contador = 1



    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    # Convert ROS Image message to OpenCV image
    current_frame = br.imgmsg_to_cv2(data, desired_encoding="bgr8")

    # Process image
    frame = current_frame

    # Output debugging information to the terminal
    rospy.loginfo("receiving video frame")

    # Frame to HSV
    frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    Hmax = cv2.getTrackbarPos("Hmax", "Trackbars 1")
    Smax = cv2.getTrackbarPos("Smax", "Trackbars 1")
    Vmax = cv2.getTrackbarPos("Vmax", "Trackbars 1")

    Hmin = cv2.getTrackbarPos("Hmin", "Trackbars 1")
    Smin = cv2.getTrackbarPos("Smin", "Trackbars 1")
    Vmin = cv2.getTrackbarPos("Vmin", "Trackbars 1")

    # Ajustar valor 2das trackbars
    Hmax2 = cv2.getTrackbarPos("Hmax", "Trackbars 2")
    Smax2 = cv2.getTrackbarPos("Smax", "Trackbars 2")
    Vmax2 = cv2.getTrackbarPos("Vmax", "Trackbars 2")

    Hmin2 = cv2.getTrackbarPos("Hmin", "Trackbars 2")
    Smin2 = cv2.getTrackbarPos("Smin", "Trackbars 2")
    Vmin2 = cv2.getTrackbarPos("Vmin", "Trackbars 2")

    MaskLI1 = np.array([Hmin, Smin, Vmin], np.uint8)
    MaskLS1 = np.array([Hmax, Smax, Vmax], np.uint8)

    mask1 = cv2.inRange(frameHSV, MaskLI1, MaskLS1)

    #frameFinal = cv2.bitwise_and(frame, frame, mask=mask1)

    print "-----------------------"

    print("red_low1 = np.array([ ", Hmin, Smin, Vmin,"], np.uint8)")
    print("red_high1 = np.array([", Hmax, Smax, Vmax,"], np.uint8)")
    print ""

    print("red_low2 = np.array([ ", Hmin2, Smin2, Vmin2,"], np.uint8)")
    print("red_high2 = np.array([", Hmax2, Smax2, Vmax2,"], np.uint8)")


    print "-----------------------"

    MaskLI2 = np.array([Hmin2, Smin2, Vmin2], np.uint8)
    MaskLS2 = np.array([Hmax2, Smax2, Vmax2], np.uint8)

    mask2 = cv2.inRange(frameHSV, MaskLI2, MaskLS2)

    #frameFinal2 = cv2.bitwise_and(frame, frame, mask=mask2)

    #frameFinalFinal = cv2.bitwise_and(frameFinal, frameFinal2)

    finalMask = cv2.add(mask1,mask2)


    cv2.imshow('Red Mask', finalMask)
    #cv2.imshow('Rango 2', frameFinal2)
    #cv2.imshow('Mask Calibration Color', frameFinalFinal)
    # cv2.imshow('Frame Calib', frame)
    # cv2.imshow('FrameHSV',frameHSV)

    cv2.waitKey(1)


def receive_message():
    # Tells rospy the name of the node.
    # Anonymous = True makes sure the node has a unique name. Random
    # numbers are added to the end of the name.
    # print ('receive msg')
    rospy.init_node('colorCalib_sub_py', anonymous=True)

    # Node is subscribing to the video_frames topic
    rospy.Subscriber('/bebop/image_raw', Image, image_callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    # Close down the video stream when done
    cv2.destroyAllWindows()


if __name__ == '__main__':
    receive_message()

#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np



#-------------------PUENTES  en la carretera    ->  Working
#                           ->  Publicar area y color...






def image_callback(data):
    global frame, mask
    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    # Output debugging information to the terminal
    #rospy.loginfo("receiving video frame")

    # Convert ROS Image message to OpenCV image
    current_frame = br.imgmsg_to_cv2(data)

    # Process image
    frame = current_frame

    azulBajo = np.array([100, 100, 20], np.uint8)
    azulAlto = np.array([125, 255, 255], np.uint8)

    """NOTA: Si uso estos rango sin el  desired_encoding="bgr8", detecta rojo en vez de azul"""


    frameHSV = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(frameHSV,azulBajo,azulAlto)

    blue_pixels = cv2.countNonZero(mask)



    #enviando a nodo maestro
    pub = rospy.Publisher('bridge_area', String, queue_size=10)
    rate = rospy.Rate(60)  # 10hz
    # rospy.loginfo(ids)
    pub.publish(str(blue_pixels))
    rate.sleep()

    #cv2.imshow('maskAzul',mask)





    cv2.imshow('Bridge Detection', mask)
    # print(gray.shape)
    # cv2.imshow("camera", current_frame)

    cv2.waitKey(1)


def receive_message():
    # Tells rospy the name of the node.
    # Anonymous = True makes sure the node has a unique name. Random
    # numbers are added to the end of the name.
    rospy.init_node('bridge_sub_py', anonymous=True)

    # Node is subscribing to the video_frames topic
    rospy.Subscriber('/bebop/image_raw', Image, image_callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    # Close down the video stream when done
    cv2.destroyAllWindows()


if __name__ == '__main__':
    receive_message()

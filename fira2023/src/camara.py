#!/usr/bin/env python

import rospy
import cv2
from cv2 import aruco
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge 
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('Camara', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hola"
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

def opencvNode():
    rospy.init_node('opencvNode', anonymous=True)
    rospy.loginfo("Hi ROS")


def publish_message():
  #https://automaticaddison.com/working-with-ros-and-opencv-in-ros-noetic/
  
  pub = rospy.Publisher('video_frames', Image, queue_size=10)
  rospy.init_node('video_pub_py', anonymous=True)
  rate = rospy.Rate(10) # 10hz
     
  cap = cv2.VideoCapture(0)

  br = CvBridge()
 
  # While ROS is still running.
  while not rospy.is_shutdown():
     
      ret, frame = cap.read()
         
      if ret == True:
        rospy.loginfo('publishing video frame')
        
        pub.publish(br.cv2_to_imgmsg(frame))

      rate.sleep()

if __name__ == '__main__':
    try:
        publish_message()
    except rospy.ROSInterruptException:
        pass

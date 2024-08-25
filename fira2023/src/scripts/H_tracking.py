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

# -------------------CENRARSE en la H azul para dejar el Technical Challange

# Calibrar area y thresh antes de usar

cam_w = 856
cam_h = 480

kernel = np.ones((3, 3), np.uint8)

areaMin = 1000
areaMax = 30000

hz = 200


"""MASCARAS"""

"""
('Max: ', 255, 255, 255)
('Min: ', 90, 128, 72)

"""

blue_low = np.array([53, 163, 0],np.uint8)
blue_high = np.array([255, 255, 255],np.uint8)

yellow_low = np.array([15,100,20],np.uint8)
yellow_high = np.array([45,255,255],np.uint8)




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

    maskRed1 = cv2.inRange(frameHSV, red_low1, red_high1)
    maskRed2 = cv2.inRange(frameHSV, red_low2, red_high2)
    maskRed = cv2.add(maskRed1, maskRed2)

    maskAzul = cv2.inRange(frameHSV, blue_low, blue_high)
    #maskAmarillo = cv2.inRange(frameHSV, yellow_low, yellow_high)

    eroded_Blue= cv2.erode(maskAzul, kernel, iterations=2)



    _, contornos, _ = cv2.findContours(eroded_Blue, cv2.RETR_EXTERNAL,
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
            cv2.drawContours(frame, [nuevoContorno], 0, (255,0,0), 3)

        else:
            x = 0
            y = 0



    pub = rospy.Publisher('blue_x', String, queue_size=2)
    pub.publish(str(x))

    pub = rospy.Publisher('blue_y', String, queue_size=2)
    pub.publish(str(y))



    rate = rospy.Rate(hz)  # 10hz
    rate.sleep()


    #cv2.imshow("Mision1", eroded_Red)
    #cv2.moveWindow("Mision1", 0, 650)
    cv2.imshow("H tracking", frame)
    cv2.moveWindow("H tracking", 2200, 1200)

    cv2.waitKey(1)


def receive_message():
    # Tells rospy the name of the node.
    # Anonymous = True makes sure the node has a unique name. Random
    # numbers are added to the end of the name.
    print ('receive msg')
    rospy.init_node('h_sub_py', anonymous=True)

    # -----  Estas publiaciones se hacen por que el master nunca lee el primer dato por alguna razon
    # enviando a nodo maestro

    rate = rospy.Rate(hz)  # 10hz

    pub = rospy.Publisher('blue_x', String, queue_size=2)
    pub.publish("Iniciando Comunicacion con mi Maestro")

    pub = rospy.Publisher('blue_y', String, queue_size=2)
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




"""   con angular
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

# -------------------CENRARSE en la H azul para dejar el Technical Challange

# Calibrar area y thresh antes de usar

cam_w = 856
cam_h = 480

kernel = np.ones((3, 3), np.uint8)

areaMin = 1000
areaMax = 30000

hz = 200


"""
('Max: ', 255, 255, 255)
('Min: ', 90, 128, 72)

"""

blue_low = np.array([70, 135, 0],np.uint8)
blue_high = np.array([255, 255, 255],np.uint8)

yellow_low = np.array([15,100,20],np.uint8)
yellow_high = np.array([45,255,255],np.uint8)





global font
font = cv2.FONT_HERSHEY_SIMPLEX

global x,y
x = 0
y = 0


global x1,y1,x2,y2,roi1
# Estas son las dimensiones de la ROI 1
x1, y1 = 0, 0
x2, y2 = cam_w , 1 * cam_h //2

global i1,i2,j1,j2,roi2
#Estas son las dimensiones de la ROI 2
i1, j1 = 0, 1 * cam_h // 2
i2, j2 = cam_w , cam_h

global areaMin, areaMax
areaMax = 20000
areaMin = 7000

global comp_j
comp_j = cam_h // 2

global  cX_master, cY_master,  cJ_master,  cI_master
cX_master = 0
cY_master = 0
cI_master = 0
cJ_master = 0




def image_callback(data):
    global cam_w, cam_h, x, y
    global frame, blue_low, blue_high, yellow_low, yellow_high, red_low1, red_high1, red_low2, red_high2
    global x1, y1, x2, y2, roi1
    global i1, i2, j1, j2, roi2
    global cX_master, cY_master, cJ_master, cI_master
    global comp_j
    global areaMin, areaMax, font

    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    # Convert ROS Image message to OpenCV image
    current_frame = br.imgmsg_to_cv2(data, desired_encoding="bgr8")

    # Process image
    frame = current_frame


    frameHSV = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

    roi1 = frameHSV[y1:y2, x1:x2]
    roi2 = frameHSV[j1:j2, i1:i2]

    maskAzul = cv2.inRange(frameHSV, blue_low, blue_high)
    #maskAmarillo = cv2.inRange(frameHSV, yellow_low, yellow_high)

    eroded_Blue= cv2.erode(maskAzul, kernel, iterations=2)


    _, contornos, _ = cv2.findContours(eroded_Blue, cv2.RETR_EXTERNAL,
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
            cv2.drawContours(frame, [nuevoContorno], 0, (255,0,0), 3)

        else:
            x = 0
            y = 0


    pub = rospy.Publisher('blue_x', String, queue_size=2)
    pub.publish(str(x))

    pub = rospy.Publisher('blue_y', String, queue_size=2)
    pub.publish(str(y))





    maskAzul_roi1 = cv2.inRange(roi1, blue_low, blue_high)
    maskAzul_roi2 = cv2.inRange(roi2, blue_low, blue_high)

    eroded_maskBlue1 = cv2.erode(maskAzul_roi1, kernel, iterations=2)

    #blur1 = cv2.medianBlur(eroded_maskBlue1, 9)

    eroded_maskBlue2 = cv2.erode(maskAzul_roi2, kernel, iterations=2)

    #blur2 = cv2.medianBlur(eroded_maskBlue2, 9)



    #cv2.rectangle(roi1, (0, 0), (x2 - x1, y2 - y1), (255, 255, 0), 3)

    _, contornos1, _ = cv2.findContours(eroded_maskBlue1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contornos1 == None:
        cX_master = cam_h // 2
        cY_master = 1 * cam_w // 6



    else:
        for c in contornos1:
            # print (c.dtype)
            area1 = cv2.contourArea(c)
            #print  area
            if area1 > areaMin and area1 < areaMax:
                M1 = cv2.moments(c)
                if (M1["m00"] == 0): M1["m00"] = 1
                cX = int(M1["m10"] / M1["m00"])
                cY = int(M1['m01'] / M1['m00'])

                cv2.circle(roi1, (cX, cY), 7, (0, 255, 0), 3)
                #cv2.line(roi1 (0, cY), (cam_w, cY), (0, 255, 255), 3)

                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(roi1, '{},{}'.format(cX, cY), (cX + 10, cY), font, 0.75, (0, 255, 0), 1, cv2.LINE_AA)
                cv2.putText(roi1, str(area1), (cX, cY + 30), font, 0.75, (0, 0, 255), 1, cv2.LINE_AA)

                nuevoContorno1 = cv2.convexHull(c)
                cv2.drawContours(roi1, [nuevoContorno1], 0, (255, 0, 0), 3)

                cX_master = cX
                cY_master = cY
                print (cX_master)
                print (cY_master)



    _, contornos2, _ = cv2.findContours(eroded_maskBlue2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contornos2 == None:
        cI_master = cam_w // 2
        cJ_master = 5 * cam_w // 6

    else:

        for c2 in contornos2:
            # print (c.dtype)
            area2 = cv2.contourArea(c2)
            # print  area
            if area2 > areaMin and area2 < areaMax:
                M2 = cv2.moments(c2)
                if (M2["m00"] == 0): M2["m00"] = 1
                cI = int(M2["m10"] / M2["m00"])
                cJ = int(M2['m01'] / M2['m00'])

                cv2.circle(roi2, (cI, cJ), 7, (0, 255, 0), 3)
                # cv2.line(roi1 (0, cY), (cam_w, cY), (0, 255, 255), 3)

                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(roi2, '{},{}'.format(cI, cJ), (cI + 10, cJ), font, 0.75, (0, 255, 0), 1, cv2.LINE_AA)
                cv2.putText(roi2, str(area2), (cI, cJ + 30), font, 0.75, (0, 0, 255), 1, cv2.LINE_AA)

                nuevoContorno2 = cv2.convexHull(c2)
                cv2.drawContours(roi2, [nuevoContorno2], 0, (255, 0, 0), 3)

                cI_master = cI
                cJ_master = cJ + comp_j



        try:
            angulo = math.atan((cJ_master - cY_master) / (cI_master - cX_master))
        except:
            angulo = math.pi / 2
        angulo = angulo * (180 / math.pi) * -1


        print('ANGULOS : ', angulo)

        cv2.line(frame, (cX_master, cY_master), (cI_master, cJ_master), (255, 255, 0), thickness=3)
        cv2.putText(frame, str(angulo), (50, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 1, cv2.LINE_AA)
        print (str(angulo))

        pub = rospy.Publisher('blue_angle', String, queue_size=2)
        pub.publish(str(angulo))

        rate = rospy.Rate(hz)  # 10hz
        rate.sleep()





    #cv2.imshow("Mision1", eroded_Red)
    #cv2.moveWindow("Mision1", 0, 650)
    cv2.imshow("Frame Blue", frame)
    cv2.imshow("ROI1", roi1)
    cv2.imshow("ROI2", roi2)
    #cv2.moveWindow("Frame BLue", 900, 0)

    cv2.waitKey(1)


def receive_message():
    # Tells rospy the name of the node.
    # Anonymous = True makes sure the node has a unique name. Random
    # numbers are added to the end of the name.
    print ('receive msg')
    rospy.init_node('h_sub_py', anonymous=True)

    # -----  Estas publiaciones se hacen por que el master nunca lee el primer dato por alguna razon
    # enviando a nodo maestro

    rate = rospy.Rate(hz)  # 10hz

    pub = rospy.Publisher('blue_x', String, queue_size=2)
    pub.publish("Iniciando Comunicacion con mi Maestro")

    pub = rospy.Publisher('blue_y', String, queue_size=2)
    pub.publish("Iniciando Comunicacion con mi Maestro")

    pub = rospy.Publisher('blue_angle', String, queue_size=2)
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

"""

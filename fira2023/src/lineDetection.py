import cv2
import numpy as np
import math

#En este ejemplo uso una imagen estatica pero se puede reemplazar por video
img = cv2.imread('/home/erick/Escritorio/street.jpeg')
kernel = np.ones((5, 5), np.uint8)

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
ret, thresh = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
blur = cv2.medianBlur(thresh, 9)
dil = cv2.erode(blur, kernel, iterations=1)
edges = cv2.Canny(dil, 50, 150, apertureSize=3)

lines = cv2.HoughLinesP(
edges, 1, np.pi/180, threshold=100, minLineLength=2, maxLineGap=10)
if lines is not None and len(lines) > 0:
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
        if y1 < 200 and y2 < 200 and x2 != x1:
            try:
                angulo = math.atan((y2 - y1) / (x2 - x1))
            except:
                angulo = math.pi / 2
            angulo = angulo * (180 / math.pi)
            print('ANGULOS : ', angulo)

            if angulo == 180:
                cY = ((y2 - y1) / 2) + y1

            cam_h = img.shape[0]
            if (angulo >= 0 and angulo <= 15)  or (angulo <= 180 and angulo >= 165) and (cY < (cam_h - (cam_h/3)+40) and (cY > cam_h - (cam_h/3))-30):
                print('Vi una linea horizontal')

cv2.imshow("thresh", thresh)
cv2.imshow("edgy", edges)
cv2.imshow("Bebop", img)
cv2.waitKey(0)

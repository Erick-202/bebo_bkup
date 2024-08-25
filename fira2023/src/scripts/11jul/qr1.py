#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import cv2

import  pyzbar.pyzbar as pyzbar



#----------------- Metodo QR deteccion ----------------- ->  Working details



QR_ant = ""

mision = 1

orientacionAnt = 'N'

#213 duela, 78 consejo
threshValue =100

global hz   #rate hertz
hz = 200


def get_mision(codigo):
    # Obtener el contenido del codigo QR
    contenido = codigo.data.decode("utf-8")

    m = "".join(contenido.split(","))   #elimia las comas de mi cadena de caracteres

    return m[4]


# Me regresa el caracter del movimiento que tiene que hacer el dron en su respectia mision
def procesar_codigo(codigo ,numMision):
    # Obtener el contenido del codigo QR
    contenido = codigo.data.decode("utf-8")



    global QR_ant

    c = "".join(contenido.split(","))   #elimia las comas de mi cadena de caracteres

    return c[mision-1] , contenido


def dibujar_codigo(codigo, numMision):
    # Obtener el contenido del codigo QR
    contenido = codigo.data.decode("utf-8")

    global QR_ant

    c = "".join(contenido.split(","))  # elimina las comas de mi cadena de caracteres

    fontSize = 1.0

    if numMision == 1:
        cv2.putText(frame, contenido, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, fontSize, (0, 255, 0), 2)
        cv2.putText(frame, c[numMision - 1], (10, 30), cv2.FONT_HERSHEY_SIMPLEX, fontSize, (0, 0, 255), 2)

        if int(c[4]) > 0:
            cv2.putText(frame, str(c[4]), (98, 30), cv2.FONT_HERSHEY_SIMPLEX, fontSize, (0, 0, 255), 2)

    elif numMision == 2:
        cv2.putText(frame, contenido, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, fontSize, (0, 255, 0), 2)
        cv2.putText(frame, c[numMision - 1], (40, 30), cv2.FONT_HERSHEY_SIMPLEX, fontSize, (0, 0, 255), 2)

        if int(c[4]) > 0:
            cv2.putText(frame, str(c[4]), (98, 30), cv2.FONT_HERSHEY_SIMPLEX, fontSize, (0, 0, 255), 2)

    elif numMision == 3:
        cv2.putText(frame, contenido, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, fontSize, (0, 255, 0), 2)
        cv2.putText(frame, c[numMision - 1], (55, 30), cv2.FONT_HERSHEY_SIMPLEX, fontSize, (0, 0, 255), 2)

        if int(c[4]) > 0:
            cv2.putText(frame, str(c[4]), (98, 30), cv2.FONT_HERSHEY_SIMPLEX, fontSize, (0, 0, 255), 2)

    elif numMision == 4:
        cv2.putText(frame, contenido, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, fontSize, (0, 255, 0), 2)
        cv2.putText(frame, c[numMision - 1], (75, 30), cv2.FONT_HERSHEY_SIMPLEX, fontSize, (0, 0, 255), 2)

        if int(c[4]) > 0:
            cv2.putText(frame, str(c[4]), (98, 30), cv2.FONT_HERSHEY_SIMPLEX, fontSize, (0, 0, 255), 2)

    else:
        # cv2.putText(frame, contenido, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        # cv2.putText(frame, 'NONE', (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        print('hasta aqui no llega')



#recibe alguno de estos caracteres N ,E , W ,S
def movimiento(char):

    if (char == 'N'):
        print ('Destiny         North - ADELANTE')

    elif (char == 'E'):
        print ('Destiny         East - DERECHA' )

    elif (char == 'W' or char == 'O'):
        print ('Destiny         West - IZQUIERDA' )

    elif (char == 'S'):
        print ('Destiny         South - ATRAS')


    else:
        print ('aqui no deberia llegar')



#NEWS es el North, East, West y South
def calcular_NEWS(anterior, destino):
    global NEWS_array, rotar

    #Numero de veces que va a rotar a la derecha para llegar al destino
    rotar = 0

#   ->  IMPORTANTE : OESTE esta en espanol,  va a haber que cambiarlo a WEST
    NEWS_array = ['N', 'E', 'S' , 'O']
#                                       1 ,  2 ,  3  ,  4

    #print ('Instruccion Anterior: ' + str( anterior))
    # n = 1
    for n  in range( 1,  len(NEWS_array)):
        if anterior ==  NEWS_array[n-1]:
            print ('------>   Calcular NEWS: ')
            print ('ANT : '+anterior)
            print ('DEST : ' + destino)
            #print (" ")
            #print NEWS_array[n-1]

            #print 'n = ' + str(n)  + '     ->    ' +str( NEWS_array[n])

            if destino ==  'N':
                rotar =   1 - n

            elif destino == 'E':
                rotar =  2 - n

            elif destino == 'S':
                rotar = 3 - n

            elif destino == 'O' or destino == 'W':
                rotar = 4 - n

            #Si regresa un valor positivo significa que tiene que girar 90 deg     a la derecha
            # negativo = izquieda

            #NOTA: AL final se determino que estaba al reves, por que cuando uso las teclas
            # 1 es izquierda
            # - 1 es derecha
            #POR ESO lo multiplico por menos 1 al final

    if rotar == 3:
        rotar = -1

    elif rotar == -3:
        rotar = 1


    return rotar * -1



#---------------------------------------------------



def image_callback(data):
    global cam_w, cam_h, QR_ant, mision, orientacionAnt, threshValue
    global frame, thresh
    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    # Convert ROS Image message to OpenCV image
    current_frame = br.imgmsg_to_cv2(data,  desired_encoding="bgr8")

    # Process image
    frame = current_frame

    # Output debugging information to the terminal
    #rospy.loginfo("receiving video frame")


    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, threshValue, 255, cv2.THRESH_BINARY) #120 en el gymnasio, 50 en el cap

    cv2.line(frame, (0, frame.shape[0] * 1 / 3), (frame.shape[1], frame.shape[0] * 1 / 3), (0, 255, 0), 3)


        # Buscar codigos QR en el frame
    codigos = pyzbar.decode(thresh)

    # Centrarse en linea
    # pa adelante

    for codigo in codigos:
        # if len(codigos[0]) > 0:
        # codigo = codigos[0]
        # Obtener las coordenadas del rectangulo del codigo QR
        x, y, w, h = codigo.rect

        centro_x = x+w//2
        centro_y = y+h//2

        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
        cv2.circle(frame,(centro_x,centro_y),7,(0,255,0),3)


        # enviando a nodo maestro
        pub = rospy.Publisher('qr_centro_x', String, queue_size=1)
        #rate = rospy.Rate(hz)  # 10hz
        # rospy.loginfo(ids)
        pub.publish(str(centro_x))
       # rate.sleep()

        # enviando a nodo maestro
        pub = rospy.Publisher('qr_centro_y', String, queue_size=1)
        #rate = rospy.Rate(hz)  # 10hz
        # rospy.loginfo(ids)
        pub.publish(str(centro_y))




       # rate.sleep()





        if y > frame.shape[0] // 3:
            # Ahi mero
           # print ("DETENGASE prro")

            # print("El codigo QR esta debajo de la linea")
            if codigo.data.decode("utf-8") == QR_ant:
               # print "Dibujar QR"
                dibujar_codigo(codigo, mision)
            # print 'Ya lo habia leido'

            else:

                # SI lee el codigo de la mision 1, publica que tiene que hacer la mision 1 y cambia a realizar la mision 2
                tempMision = get_mision(codigo)

                if int(tempMision) == 1:
                    # enviando a nodo maestro
                    print ('entre mision 2')
                    mision = 2

                elif int(tempMision) == 2 and int(mision) ==2:
                    # enviando a nodo maestro

                    mision = 3

                elif int(tempMision) == 3 and int(mision) ==3 :
                    # enviando a nodo maestro

                    mision = 4

                elif int(tempMision) == 4 and int(mision) ==4:

                    print ('hacer mision 4')

                elif int(tempMision) == 4 and int(mision) == 1:
                    mision = 1

                print  ('MISION ', mision)
                codigo_result = procesar_codigo(codigo, mision)
                print('CODIGO = ' + codigo_result[1])
                print('  CHAR     =  ' + str(codigo_result[0]))
                QR_ant = codigo_result[1]
                movimiento(codigo_result[0])
                numRotations  = str(calcular_NEWS(orientacionAnt, codigo_result[0]))
                print(' Girar : ' + numRotations + ' veces  ')
                orientacionAnt =  str(codigo_result[0])

                # enviando a nodo maestro
                pub = rospy.Publisher('num_rotations', String, queue_size=2)
                #rate = rospy.Rate(hz)  # 10hz
                # rospy.loginfo(ids)
                pub.publish(str(numRotations))
               # rate.sleep()

                print ""
                print ("TEMPORAL :  ", str(tempMision))
                print ""

                pub = rospy.Publisher('num_mision', String, queue_size=2)
               # rate = rospy.Rate(hz)  # 10hz
                # rospy.loginfo(ids)
                pub.publish(str(tempMision))
               # rate.sleep()

                print "Vi un QR nuevo"

                pub = rospy.Publisher('new_qr', Bool, queue_size=2)
                #rate = rospy.Rate(hz)  # 10hz
                # rospy.loginfo(ids)
                pub.publish(True)
                #rate.sleep()

                """
                                if mision < int(get_mision(codigo)):
                    mision = int(get_mision(codigo))
                    print  ('MISION ', mision)
                    codigo_result = procesar_codigo(codigo, mision)
                    print('CODIGO = ' + codigo_result[1])
                    print('  CHAR     =  ' + str(codigo_result[0]))
                    QR_ant = codigo_result[1]
                    movimiento(codigo_result[0])
                    print(' Girar : ' + str(
                        calcular_NEWS(orientacionInicial, codigo_result[0])) + ' veces ')

                    # enviando a nodo maestro
                    pub = rospy.Publisher('num_rotations', String, queue_size=10)
                    rate = rospy.Rate(50)  # 10hz
                    # rospy.loginfo(ids)
                    pub.publish(str(calcular_NEWS(orientacionInicial, codigo_result[0])))
                    rate.sleep()


                else:
                    print  ('MISION ', mision)
                    codigo_result = procesar_codigo(codigo, mision)
                    print('CODIGO = ' + codigo_result[1])
                    print('  CHAR     =  ' + str(codigo_result[0]))
                    QR_ant = codigo_result[1]
                    movimiento(codigo_result[0])
                    print(' Girar : ' + str(calcular_NEWS(orientacionInicial, codigo_result[0])) + ' veces  der')

                    # enviando a nodo maestro
                    pub = rospy.Publisher('num_rotations', String, queue_size=10)
                    rate = rospy.Rate(50)  # 10hz
                    # rospy.loginfo(ids)
                    pub.publish(str(calcular_NEWS(orientacionInicial, codigo_result[0])))
                    rate.sleep()
                
                """
        rate = rospy.Rate(hz)  # 10hz
        rate.sleep()


    cv2.imshow("QR Detection", frame)
    cv2.moveWindow("QR Detection" , 1000 ,0 )
    #cv2.imshow("Thresh", thresh)

    cv2.waitKey(1)





def receive_message():
    global hz
    # Tells rospy the name of the node.
    # Anonymous = True makes sure the node has a unique name. Random
    # numbers are added to the end of the name.
    print ('receive msg')
    rospy.init_node('qr_sub_py', anonymous=True)

    #DATO BASURA---------- Se envia este dato por que por alguna razon, el master nunca recibe el primer dato

    # enviando a nodo maestro
    pub = rospy.Publisher('num_rotations', String, queue_size=10)
    rate = rospy.Rate(hz)  # 10hz
    # rospy.loginfo(ids)
    pub.publish("Iniciando Comunicacion con mi Maestro")
    rate.sleep()

    pub = rospy.Publisher('num_mision', String, queue_size=10)
    rate = rospy.Rate(hz)  # 10hz
    # rospy.loginfo(ids)
    pub.publish("Iniciando Comunicacion con mi Maestro2")
    rate.sleep()

    pub = rospy.Publisher('qr_centro_x', String, queue_size=10)
    rate = rospy.Rate(hz)  # 10hz
    # rospy.loginfo(ids)
    pub.publish("Iniciando Comunicacion con mi Maestro3")
    rate.sleep()

    pub = rospy.Publisher('qr_centro_y', String, queue_size=10)
    rate = rospy.Rate(hz)  # 10hz
    # rospy.loginfo(ids)
    pub.publish("Iniciando Comunicacion con mi Maestro4")
    rate.sleep()

    pub = rospy.Publisher('new_qr', Bool, queue_size=10)
    rate = rospy.Rate(hz)  # 10hz
    # rospy.loginfo(ids)
    pub.publish("Iniciando Comunicacion con mi Maestro5")
    rate.sleep()

    # Node is subscribing to the video_frames topic
    rospy.Subscriber('/bebop/image_raw', Image, image_callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    # Close down the video stream when done
    cv2.destroyAllWindows()


if __name__ == '__main__':
    receive_message()

from cvzone.FaceDetectionModule import FaceDetector
from cvzone.SerialModule import SerialObject
import cv2

arduino = SerialObject("COM6")

send_x = 0
send_z = 0




faces = FaceDetector()
cam = cv2.VideoCapture(0)


deltat = 1
prex = 0
prey = 0


acceptance_rate = 10


while True:

    cisei, img = cam.read()

    img,bboxs = faces.findFaces(img)


    if bboxs:
        faccia=bboxs[0]
        print(faccia)
        id = faccia['id']
        bbox = faccia['bbox']
        score = faccia['score']
        center = faccia['center']

        cv2.circle(img,center,5,(255,255,255), thickness=10)

        if prex !=0 and prey !=0:
            deltax = center[0]-prex
            deltay = center[1]-prey
            accl_x = (2*deltax)/deltat
            accl_y = (2*deltay)/deltat
            # print(accl_y)

            if accl_x > acceptance_rate:
                print('sinistra')
                send_x = 2
                # cv2.putText(img,"sinistra",(bbox[2],bboxs[1]-20),cv2.FONT_HERSHEY_PLAIN,fontScale=1.5,color=(255,255,255),thickness=2)
            if accl_x < -acceptance_rate:
                print('destra')
                send_x = 1
            if accl_y > acceptance_rate:
                print('down') #il contrario quindi se maggiore di 4 -> giù
                send_z = 2
            if accl_y < -acceptance_rate:
                print('up') #il contrario quindi se minore di -4 -> su
                send_z = 1



        arduino.sendData([send_x,send_z])

        send_x = 0
        send_z = 0

        prex = center[0]
        prey = center[1]





    cv2.imshow("Immagine", img)

    cv2.waitKeyEx(10)
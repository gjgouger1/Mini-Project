import numpy as np
import cv2
from cv2 import aruco
from picamera import PiCamera
from time import sleep
from PIL import Image
import math


#initializing the dictionary and parameters
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
arucoParams = cv2.aruco.DetectorParameters_create()

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()
while True: #infinte loop capturing the given frame
    # Capture frame-by-frame
    ret, frame = cap.read()
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    # Our operations on the frame come here
    image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #reading in image in grayscale
    

    (corners, ids, rejected) = cv2.aruco.detectMarkers(image, aruco_dict, parameters=arucoParams)

    if (len(corners) > 0): #below is repeated code from Q4
        ids = ids.flatten()

        for (markerCorner, markerID) in zip(corners, ids):
            
            corners = markerCorner.reshape((4,2))
            (topLeft,topRight, bottomRight, bottomLeft) = corners

            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
	    
	    # compute and draw the center (x, y)-coordinates of the Aruco
	    # marker
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)

            

            width = frame.shape[1]
            height = frame.shape[0]

            quadrant = 0

            if (cX < width/2 and cY < height/2): #lower left
                quadrant = math.pi/2
            if (cX > width/2 and cY < height/2): #lower right
                quadrant = 0
            if (cX < width/2 and cY > height/2): #upper left
                quadrant = math.pi
            if (cX > width/2 and cY > height/2): #upper right
                quadrant = 3*math.pi*0.5
            

           
            cv2.putText(image, str(quadrant), #put the angle on the marker, showing delta degrees to center
                (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2)
        
            
    # Display the resulting frame
    cv2.imshow('frame', image)
    if cv2.waitKey(1) == ord('q'):
        break

    
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

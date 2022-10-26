#This file handles the computer vision portion of Demo 1
#Meaning that in this file we can take an image input from the camera mounted on our robot
#and locate an Aruco marker within the field of view of the robot. The result is displayed on the
#LCD screen and the detection of the Aruco marker is also stated on the LCD screen.

import numpy as np
import cv2
from cv2 import aruco
from picamera import PiCamera
from time import sleep
from PIL import Image
import math
import smbus2 as smbus
import time
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd


#initializing the dictionary and parameters
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
arucoParams = cv2.aruco.DetectorParameters_create()

# LCD setup

lcd_columns = 16
lcd_rows = 2

# Initialise I2C bus.
i2c = board.I2C()  # uses board.SCL and board.SDA

# Initialise the LCD class
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

bus = smbus.SMBus(1)
# this is the address we set up in the Arduino program
address = 4




#def readNumber(quadrant, offset):
#    #number = bus.read_byte(address)
#    number = bus.read_byte(address)
#    return number

lcd.clear()
# Set LCD color to green
lcd.color = [0, 100, 0]
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()
starttime = time.time()    
once = 0
frame_bool = 0
printed = 0
while True: #infinte loop capturing the given frame
    once = 0
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
        starttime = time.time()

        marker_bool = 1 #marker is found
        once = 1
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
            #cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
            
            
            
            #calculation of phi
            width_image = frame.shape[1]
            delta_width = ((width_image / 2.0) - cX)
            center_to_edge = (width_image / 2.0)
            phi = (57/2)*(delta_width / center_to_edge)

           
        
            # Print message only once an aruco is initially detected (from frame without to a frame with aruco)
            if (once != 0 and printed == 0):
                lcd.message = 'Aruco Detected'
                once = 0
                printed = 1
                time.sleep(1)
                lcd.clear()
            #writeNumber(phi, 0)
            phi = round(phi,1) #round phi to one decimal
            if (time.time() % 1): #every one second lets print to the lcd to prevent lag
                lcd.message = str(phi)
                #phi_rad = phi * (3.14 / 180) * 100  THIS CAN BE UNCOMMENTED FOR DEMO 2
                #bus.write_byte_data(address,0,int(phi_rad))
            
            cv2.putText(image, str(phi), #put the angle on the marker, showing delta degrees to center
                (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2)
            #thiscan be uncommented to print the quadranto nthe aruco
            
        


    else:
        lcd.clear()
            
        if ((time.time() - starttime) > 2): #this resets the flag to print the aruco if an aruco is not on the screen for more than 2 sec
            printed = 0
            #if (starttime % 0.1):
                #writeNumber(quadrant, 0)
                #lcd.clear()
    # Display the resulting frame
    cv2.imshow('frame', image)
    if cv2.waitKey(1) == ord('q'):
        break

    
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()


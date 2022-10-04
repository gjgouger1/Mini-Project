#This code handles the image processing and video capture. This code also handles transfering data from the camera input to the Arduino, in addition it
#processes data from the arduino in order to print on to the LCD.

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


def writeNumber(quadrant, offset): #function writeNumber to write and read the data over the line, done in the same function so the data does not collide
    #bus.write_byte(address, value)
    try:
        bus.write_byte_data(address,0,quadrant) #writing from pi to arduino
    except:
        print('io error writing')
        
    tpos = ''
    try:
        data= ''
        for i in range(0,4):
            data += chr(bus.read_byte(address, 0)) #reading from arduino one char at a time
        

       
        
    except:
        print('io error')
        
    if (quadrant == 1):
        tpos = '0'
    if (quadrant == 2):
        tpos = 'pi/2'
    if (quadrant == 3):
        tpos = 'pi'
    if (quadrant == 4):
        tpos = '3pi/2'

    if (starttime % 0.1): #print to the lcd every 1/10th of a second so we dont have insane lag
        lcd.message = 'C: ' + str(data) + '\nE:' + str(tpos + '     ') #print to the lcd the given data that we are sending and receiving

        
    return -1



lcd.clear()
# Set LCD color to green
lcd.color = [0, 100, 0]
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()
    
starttime = time.time()
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
            #cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)

            

            width = frame.shape[1]
            height = frame.shape[0]

            quadrant = 0

            if (cX < width/2 and cY < height/2): #upper left
                quadrant = 2 #pi/2
            if (cX > width/2 and cY < height/2): #lower right
                quadrant = 1 #0
            if (cX < width/2 and cY > height/2): #lower left
                quadrant = 3 #pi
            if (cX > width/2 and cY > height/2): #lower right
                quadrant = 4 #3pi/2
            
        
           
            writeNumber(quadrant, 0)

                


            cv2.putText(image, str(quadrant), #put the angle on the marker, showing delta degrees to center
                (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2)
        


        else:
            quadrant = 0;
            if (starttime % 0.1):
                writeNumber(quadrant, 0)
                lcd.clear()
    # Display the resulting frame
    cv2.imshow('frame', image)
    if cv2.waitKey(1) == ord('q'):
        break

    
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

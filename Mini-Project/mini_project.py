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

def writeNumber(quadrant, offset):
    #bus.write_byte(address, value)
    bus.write_byte_data(address, offset, quadrant)
    return -1

def readNumber(quadrant, offset):
    #number = bus.read_byte(address)
    number = bus.read_byte_data(address, offset)
    print(number)
    return number

lcd.clear()
# Set LCD color to green
lcd.color = [0, 100, 0]
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
            #cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)

            

            width = frame.shape[1]
            height = frame.shape[0]

            quadrant = 0

            if (cX < width/2 and cY < height/2): #lower left
                quadrant = 1 #pi/2
            if (cX > width/2 and cY < height/2): #lower right
                quadrant = 0 #0
            if (cX < width/2 and cY > height/2): #upper left
                quadrant = 2 #pi
            if (cX > width/2 and cY > height/2): #upper right
                quadrant = 3 #3pi/2
            
            writeNumber(quadrant, 0)
            number = 0 #hardcoded tozero for now
            #number = readNumber(quadrant, offset)
            
            # Print two line message
            lcd.message = str(quadrant) + "\n" + str(number)
            
            #thiscan be uncommented to print the quadranto nthe aruco
            #cv2.putText(image, str(quadrant), #put the angle on the marker, showing delta degrees to center
                #(topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                #0.5, (0, 255, 0), 2)
        


            
    # Display the resulting frame
    cv2.imshow('frame', image)
    if cv2.waitKey(1) == ord('q'):
        break

    
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

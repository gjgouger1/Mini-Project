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
import pickle
import os
import threading

class CameraBufferCleanerThread(threading.Thread):
    def __init__(self, camera, name='camera-buffer-cleaner-thread'):
        self.camera = camera
        self.last_frame = None
        super(CameraBufferCleanerThread, self).__init__(name=name)
        self.start()

    def run(self):
        while True:
            ret, self.last_frame = self.camera.read()


#initializing the dictionary and parameters
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
arucoParams = cv2.aruco.DetectorParameters_create()

#camera calibration
if not os.path.exists('/home/pi/Desktop/calibration.pckl'):
    print("You need to calibrate the camera you'll be using. See calibration project directory for details.")
    exit()
else:
    f = open('calibration.pckl', 'rb')
    (cameraMatrix, distCoeffs, _, _) = pickle.load(f)
    f.close()
    if cameraMatrix is None or distCoeffs is None:
        print("Calibration issue. Remove ./calibration.pckl and recalibrate your camera with CalibrateCamera.py.")
        exit()

#lcd setup
#lcd_columns = 16
#lcd_rows = 2
#i2c = board.I2C()  # uses board.SCL and board.SDA

# Start the camera
camera = cv2.VideoCapture(0)
#camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1000)
#camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1000)
camera.set(cv2.CAP_PROP_BUFFERSIZE, 2);
camera.set(cv2.CAP_PROP_FPS, 24);
camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75);
camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25);
camera.set(cv2.CAP_PROP_EXPOSURE, 40);

markerSize = 5.3 #in cm
mtx = cameraMatrix
dist = distCoeffs
# Create vectors we'll be using for rotations and translations for postures
rvecs, tvecs = None, None

# Initialise I2C bus.


# Initialise the LCD class
#lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

bus = smbus.SMBus(1)
# this is the address we set up in the Arduino program
address = 4

#lcd.clear()
# Set LCD color to green
#lcd.color = [0, 100, 0]

once = 0
printed = 0
cam_cleaner = CameraBufferCleanerThread(camera)
starttime = 0
while True:
    if cam_cleaner.last_frame is not None:
        img = cv2.cvtColor(cam_cleaner.last_frame, cv2.COLOR_BGR2GRAY)
        # Obtain the image pixel size, and find the horizontal (x-coordinate) center
        imgSize = img.shape
        xCoord = (imgSize[1] / 2)
        yCoord = (imgSize[0] / 2)
        # Import the dictonary of markers, define the parameters, and load values into vars
        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
        arucoParams = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(img, arucoDict, parameters=arucoParams)
        # Verify if an ArUco marker is detected
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, markerSize, mtx, dist)
        
        if len(corners)> 0:
            starttime = time.time()

            marker_bool = 1 #marker is found
            once = 1

            # flatten ID list
            ids = ids.flatten()
            # loop over the marker coners
            for (markerCorner, markerID) in zip(corners, ids):
                # extract marker corners and convert them to integers
                corners = markerCorner.reshape((4,2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                #draw bounding box
                cv2.line(img, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(img, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(img, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(img, bottomLeft, topLeft, (0, 255, 0), 2)
                # compute and draw the center (x, y)-coordinates of the ArUco marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(img, (cX, cY), 4, (0, 0, 255), -1)
                
                
                width_image = img.shape[1]
                delta_width = ((width_image / 2.0) - cX)
                center_to_edge = (width_image / 2.0)
                phi = (57/2)*(delta_width / center_to_edge)
                dist = (tvec[0][0][2]/100) / (math.cos(phi*3.14/ 180))
                
                # Print two line message
                #if (once != 0 and printed == 0):
                    #lcd.message = 'Aruco Detected'
                    #once = 0
                    #printed = 1
                    #time.sleep(1)
                    #lcd.clear()
                #writeNumber(phi, 0)
                phi = round(phi,1)
                if (time.time() % 1):
                    phi_rad = phi * (3.14 / 180) 
                    phi_rad_1 = round(phi_rad, 2)
                    print(phi_rad / 0.003682)
                    if (phi_rad < 0):
                        phi_test = 256 + int(phi_rad / 0.003682)  
                        
                    else:
                        phi_test = phi_rad / 0.003682
                        
#                    print(phi_rad)        
#                    print(phi_test, dist)
#                    print(markerID)
                    #bus.write_byte_data(address,int(phi_test),int(dist*10),int(markerID)) #phi to send
                    
                    bus.write_i2c_block_data(address, int(markerID), [int(dist*10),int(phi_test)])
                    print([int(phi_test), int(dist*10),int(markerID)])
                
                cv2.putText(img, str(markerID), #put the angle on the marker, showing delta degrees to center
                    (topLeft[0], topLeft[1]), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)
                cv2.putText(img, str(phi), #put the angle on the marker, showing delta degrees to center
                    (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)
        
        else:
            bus.write_byte_data(address, int(0),int(0))
        cv2.imshow('Image', img)
        
    if (cv2.waitKey(1) & 0xFF == ord('q')):
        break
camera.release()
cv2.destroyAllWindows()



"""
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Cannot open camera")
    exit()
starttime = time.time()    
once = 0
frame_bool = 0
printed = 0
Line_Pts = None
measure = None
while True: #infinte loop capturing the given frame
    once = 0
    Dist = []
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
            #area = ((topRight[0] - topLeft[0])/3779.527) * ((bottomLeft[1] - topLeft[1])/3779.527)
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
        
        # compute and draw the center (x, y)-coordinates of the Aruco
        # marker
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            #cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
            
            
            

            width_image = frame.shape[1]
            delta_width = ((width_image / 2.0) - cX)
            center_to_edge = (width_image / 2.0)
            phi = (57/2)*(delta_width / center_to_edge)

            #markerSizeInCM = 6
            #pixel^2 
            #area = area * (0.0002645833*0.0002645833)
            #dist = 0.055 * (0.0036/ area)
            #rvec , tvec, _ = aruco.estimatePoseSingleMarkers(corners, markerSizeInCM, image, dist)
            
            
            # Print two line message
            if (once != 0 and printed == 0):
                #lcd.message = 'Aruco Detected'
                once = 0
                printed = 1
                time.sleep(1)
                lcd.clear()
            #writeNumber(phi, 0)
            phi = round(phi,1)
            if (time.time() % 1):
                phi_rad = phi * (3.14 / 180) 
                phi_rad_1 = round(phi_rad, 2)
                
                if (phi_rad < 0):
                    phi_test = 256 + int(phi_rad / 0.003682)  
                
                else:
                    phi_test = phi_rad / 0.003682
                    
                #lcd.message = str(phi_rad_1)
                
                #if (phi < 0):
                    #bus.write_byte_data(address,0,0)
                #if (phi > 0 ):
                    #bus.write_byte_data(address,0,1)
                    
                bus.write_byte_data(address,0,int(phi_test)) #phi to send
                
                
                
                
        
            cv2.putText(image, str(phi), #put the angle on the marker, showing delta degrees to center
                (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2)
            #thiscan be uncommented to print the quadranto nthe aruco
            
            


    else:
        lcd.clear()
        if (time.time() - starttime > 10):
            bus.write_byte_data(address, 0,int(0))
        if ((time.time() - starttime) > 2):
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

"""
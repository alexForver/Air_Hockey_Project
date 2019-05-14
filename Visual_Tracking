from picamera.array import PiRGBArray
from picamera import PiCamera 
import time
import cv2
from collections import deque
import numpy as np
import argparse
import imutils
import serial
                
if __name__ == '__main__':
      
        blue_lower = (90,50,50)
        blue_upper = (160,255,255)
        buffer = 32
        n = 0
        pts = deque(maxlen=buffer)
        
        camera = PiCamera()
        camera.resolution = (640,480)
        camera.framerate = 32
        rawCapture = PiRGBArray(camera, size=(640,480))
        time.sleep(0.1)

        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
                n += 1
                image = frame.array
                blurred = cv2.GaussianBlur(image,(11,11),0)
                hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

                mask = cv2.inRange(hsv, blue_lower, blue_upper)
                mask = cv2.erode(mask, None, iterations=2)
                mask = cv2.dilate(mask, None, iterations=2)

                cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cnts = imutils.grab_contours(cnts)
                center = None

                if len(cnts) > 0:
                        c = max(cnts, key=cv2.contourArea)
                        ((x,y), radius) = cv2.minEnclosingCircle(c)
                        M = cv2.moments(c)
                        center=(int(M["m10"] / M["m00"]),int(M["m01"]/M["m00"]))

                        if radius > 10:
                            cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
                            cv2.circle(image, center, 5, (255, 255, 255), -1)

                        pts.appendleft(center)

                for i in range(1, len(pts)):
                        if pts[i - 1] is None or pts[i] is None:
                                continue
                        thickness = int(np.sqrt( buffer/ float(i + 1)) * 2.5)
                        cv2.line(image, pts[i - 1], pts[i], (0, 0, 255), thickness)
                        
                cv2.imshow("Frame", image)
                cv2.imwrite("tracking{}.jpg".format(n),image)
                
                key = cv2.waitKey(1) & 0xFF
                rawCapture.truncate(0)

                if key == ord("q"):
                        break

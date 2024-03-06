import numpy as np
import cv2
from networktables import NetworkTables
import logging

kernal = np.ones((7,7), "uint8")
camera = cv2.VideoCapture(0) # First webcam (video0)
NetworkTables.initialize()
sd=NetworkTables.getTable("SmartDashboard")
def BestieDetection(frame):
    success, frame = frame
    if not success:
        print("not success frame")
    else:
        hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        cv2.imshow('raw', frame)

    # lower range of red color in HSV
        lower_range = (80, 100, 0)
        upper_range = (90, 245, 245)
        mask = cv2.inRange(hsv_img, lower_range, upper_range)

        lower_range = (90,100,0)
        upper_range = (130,245,245)
        mask1 = cv2.inRange(hsv_img, lower_range, upper_range)

        mask = mask + mask1

        mask = cv2.erode(mask, kernal)
        mask = cv2.erode(mask, kernal)
        mask = cv2.dilate(mask, kernal)
        friendetex = []
        friendetey = []
        friendeteh = []
        friendetew = []

    #frame = cv2.bitwise_and(frame, frame, mask=mask)

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for pic, contour in enumerate (contours):
            area = cv2.contourArea(contour)
            if (area > 1000):
             x,y,w,h = cv2.boundingRect(contour)
             frame = cv2.rectangle(frame, (x,y), (x+w, y+h), (255,255,0),2)

             cv2.putText(frame, "Friendly!", (x,y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,0))
             friendeteh.append(h)
             friendetex.append(x)
             friendetey.append(y)
             friendetew.append(w)
    #print("h", friendeteh)
    #print("w", friendetew) 
    #print("y", friendetey)
    #print("x", friendetex)
        cv2.imshow('Highlighted', frame)
        return(friendetex, friendetey, friendeteh, friendetew)


    # Display the color of the image

        
def NoteDetetction(frame):
        #print("Noted")
        success, frame = frame
        if not success:
            print("not success")
        else:
         hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

         # lower range of red color in HSV
         lower_range = (2.5, 125, 50)
         upper_range = (30, 255, 255)
         mask = cv2.inRange(hsv_img, lower_range, upper_range)

         mask = cv2.erode(mask, kernal)
         mask = cv2.erode(mask, kernal)
         mask = cv2.dilate(mask, kernal)
         #frame = cv2.bitwise_and(frame, frame, mask=mask)

         contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

         notedetex = []
         notedetey = []
         notedeteh = []
         notedetew = []

         for pic, contour in enumerate (contours):
            area = cv2.contourArea(contour)
            if (area > 500):
                x,y,w,h = cv2.boundingRect(contour)
                frame = cv2.rectangle(frame, (x,y), (x+w, y+h), (255,255,0),2)

                cv2.putText(frame, "Note!", (x,y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,0))
                #print(x, x+h, y, y+w)
                notedetex.append(x)
                notedetey.append(y)
                notedeteh.append(h)
                notedetew.append(w)
        

         cv2.imshow("mask", mask)


      #frame = cv2.bitwise_and(frame, frame, mask=mask)

         contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
         cv2.imshow('Highlighted', frame)
         return(notedetex, notedetey, notedeteh, notedetew)
def OppDetection(frame):
    #print("oppd")
    success, frame = frame
    if not success:
        print("not success frame")
    else:
        hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # lower range of red color in HSV
        lower_range = (0, 0, 0)
        upper_range = (5, 255, 255)
        mask = cv2.inRange(hsv_img, lower_range, upper_range)

        lower_range = (130,0,100)
        upper_range = (300,255,255)
        mask1 = cv2.inRange(hsv_img, lower_range, upper_range)

        lower_range = (0,245,100)
        upper_range = (300,255,255 )
        mask2 = cv2.inRange(hsv_img, lower_range, upper_range)
       
        mask = mask + mask1 + mask2

        mask = cv2.erode(mask, kernal)
        mask = cv2.erode(mask, kernal)
        mask = cv2.dilate(mask, kernal)

        #frame = cv2.bitwise_and(frame, frame, mask=mask)
        oppdetex = []
        oppdetey = []
        oppdeteh = []
        oppdetew = []
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for pic, contour in enumerate (contours):
            area = cv2.contourArea(contour)
            if (area > 700):
                x,y,w,h = cv2.boundingRect(contour)
                frame = cv2.rectangle(frame, (x,y), (x+w, y+h), (255,255,0),2)
                cv2.putText(frame, "Enemy!", (x,y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,0))
                #print(x, h, y, w)
                oppdetex.append(x)
                oppdetey.append(y)
                oppdeteh.append(h)
                oppdetew.append(w)


    # Display the color of the image
        cv2.imshow('Highlighted', frame)
        return(oppdetex, oppdetey, oppdeteh, oppdetew)
while camera.isOpened:
    framd = camera.read()
    framfrien = framd
    framopp = framd
    framnot = framd
    Bestiecoord =  BestieDetection(framfrien)
    oppcoord =  OppDetection(framopp)
    notecoord =  NoteDetetction(framnot)
    print("b", Bestiecoord)
    print("o", oppcoord)
    print("n", notecoord)
    sd.putNumberArray("friendly coordinates:", Bestiecoord)
    sd.putNumberArray("opponent coordinates:", oppcoord)
    sd.putNumberArray("note coordinates:", notecoord)
    DOUBLE_ARRAY = [Bestiecoord, oppcoord, notecoord]
    #print(oppcoord,"||", notecoord, "||", Bestiecoord)
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

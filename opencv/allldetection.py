import numpy as np
import cv2
from networktables import NetworkTables

kernal = np.ones((7,7), "uint8")
camera = cv2.VideoCapture(0) # First webcam (video0)
sd = NetworkTables.getTable("SmartDashboard")
NetworkTables.initialize(server="10.25.2.2")

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
        friendetex = ''
        friendetey = ''
        friendeteh = ''
        friendetew = ''

    #frame = cv2.bitwise_and(frame, frame, mask=mask)

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for pic, contour in enumerate (contours):
            area = cv2.contourArea(contour)
            if (area > 1000):
             x,y,w,h = cv2.boundingRect(contour)
             frame = cv2.rectangle(frame, (x,y), (x+w, y+h), (255,255,0),2)

             cv2.putText(frame, "Friendly!", (x,y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,0))
             friendeteh = friendeteh + ' ' + str(h)
             friendetex = friendetex + ' ' + str(x)
             friendetey = friendetey + ' ' + str(y)
             friendetew = friendetew + ' ' + str(w)
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
         cv2.imshow('raw', frame)

         # lower range of red color in HSV
         lower_range = (4.5, 50, 50)
         upper_range = (25, 255, 255)
         mask = cv2.inRange(hsv_img, lower_range, upper_range)

         mask = cv2.erode(mask, kernal)
         mask = cv2.erode(mask, kernal)
         mask = cv2.dilate(mask, kernal)
         #frame = cv2.bitwise_and(frame, frame, mask=mask)

         contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

         notedetex = ''
         notedetey = ''
         notedeteh = ''
         notedetew = ''

         for pic, contour in enumerate (contours):
            area = cv2.contourArea(contour)
            if (area > 500):
                x,y,w,h = cv2.boundingRect(contour)
                frame = cv2.rectangle(frame, (x,y), (x+w, y+h), (255,255,0),2)

                cv2.putText(frame, "Note!", (x,y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,0))
                #print(x, x+h, y, y+w)
                notedetex = notedetex + ' ' + str(x)
                notedetey = notedetey + ' ' + str(y)
                notedeteh = notedeteh + ' ' + str(h)
                notedetew = notedetew + ' ' + str(w)
        

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
        cv2.imshow('raw', frame)

        # lower range of red color in HSV
        lower_range = (1, 5, 5)
        upper_range = (5, 150, 150)
        mask = cv2.inRange(hsv_img, lower_range, upper_range)

        lower_range = (160,5,5)
        upper_range = (255,245,245)
        mask1 = cv2.inRange(hsv_img, lower_range, upper_range)

        mask = mask + mask1

        mask = cv2.erode(mask, kernal)
        mask = cv2.erode(mask, kernal)
        mask = cv2.dilate(mask, kernal)

        #frame = cv2.bitwise_and(frame, frame, mask=mask)
        oppdetex = ''
        oppdetey = ''
        oppdeteh = ''
        oppdetew = ''
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for pic, contour in enumerate (contours):
            area = cv2.contourArea(contour)
            if (area > 700):
                x,y,w,h = cv2.boundingRect(contour)
                frame = cv2.rectangle(frame, (x,y), (x+w, y+h), (255,255,0),2)
                cv2.putText(frame, "Enemy!", (x,y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,0))
                #print(x, h, y, w)
                oppdetex = oppdetex + ' ' + str(x)
                oppdetey = oppdetey + ' ' + str(y)
                oppdeteh = oppdeteh + ' ' + str(h)
                oppdetew = oppdetew + ' ' + str(w)


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
    #print(oppcoord,"||", notecoord, "||", Bestiecoord)
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

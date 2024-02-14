import numpy as np
import cv2
from networktables import NetworkTables

kernal = np.ones((7,7), "uint8")
camera = cv2.VideoCapture(0) # First webcam (video0)
#sd = NetworkTables.getTable("SmartDashboard")
#NetworkTables.initialize(server="10.25.2.2")

    #while camera.isOpened():
    #success, frame = camera.read()
   # if not success:
   #     break
  #  hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
   # cv2.imshow('raw', frame)

    # lower range of red color in HSV
  #  lower_range = (4.5, 130, 0)
   # upper_range = (4.9, 255, 255)
 #   mask = cv2.inRange(hsv_img, lower_range, upper_range)

  #  lower_range = (5.1, 130, 1)
  #  upper_range = (50,255,255)
  #  mask1 = cv2.inRange(hsv_img, lower_range, upper_range)

    #lower_range = (0, 100, 50)
    #upper_range = (2, 245, 245)
    #mask2 = cv2.inRange(hsv_img, lower_range, upper_range)

    #lower_range = (170,100,50)
    #upper_range = (180,245,245)
    #mask3 = cv2.inRange(hsv_img, lower_range, upper_range)

    #lower_range = (5, 100, 50)
    #upper_range = (5, 245, 245)
    #mask4 = cv2.inRange(hsv_img, lower_range, upper_range)

    #lower_range = (160,100,50)
    #upper_range = (170,245,245)
    #mask5 = cv2.inRange(hsv_img, lower_range, upper_range)

    #lower_range = (80, 0, 0)
    #upper_range = (90, 245, 245)
    #mask6 = cv2.inRange(hsv_img, lower_range, upper_range)

    #lower_range = (90,55,0)
    #upper_range = (130,245,245)
    #mask7 = cv2.inRange(hsv_img, lower_range, upper_range)

def NoteDetetction():
        success, frame = camera.read()
        if not success:
            print("not success")
        else:
         hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
         cv2.imshow('raw', frame)

         # lower range of red color in HSV
         lower_range = (3.5, 100, 50)
         upper_range = (4.9, 255, 255)
         mask = cv2.inRange(hsv_img, lower_range, upper_range)

         lower_range = (5.1,100,50)
         upper_range = (17,255,255)
         mask1 = cv2.inRange(hsv_img, lower_range, upper_range)

         mask = mask + mask1

         mask = cv2.erode(mask, kernal)
         mask = cv2.erode(mask, kernal)
         mask = cv2.dilate(mask, kernal)

         #frame = cv2.bitwise_and(frame, frame, mask=mask)

         contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

         for pic, contour in enumerate (contours):
            area = cv2.contourArea(contour)
            if (area > 500):
                x,y,w,h = cv2.boundingRect(contour)
                frame = cv2.rectangle(frame, (x,y), (x+w, y+h), (255,255,0),2)

                cv2.putText(frame, "Note!", (x,y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,0))



         mask = mask + mask1

         mask = cv2.erode(mask, kernal)
         mask = cv2.erode(mask, kernal)
         mask = cv2.dilate(mask, kernal)

         cv2.imshow("mask", mask)


      #frame = cv2.bitwise_and(frame, frame, mask=mask)

         contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

      #for pic, contour in enumerate (contours):
        #    area = cv2.contourArea(contour)
     #   if (area > 2000):
     #       x,y,w,h = cv2.boundingRect(contour)
            #frame = cv2.rectangle(frame, (x,y), (x+w, y+h), (255,255,0),2)

     #       center = (int(x+w/2),int(y+h/2))
     #       frame = cv2.circle(frame, center, 4, (0,0,255), -1)

      #      cv2.putText(frame, "Note!", (x,y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,0))

    
     #   sd.putNumber("andys number", 6)


        # Display the color of the image
        cv2.imshow('Highlighted', frame)
      #   if cv2.waitKey(10) & 0xFF == ord('q'):
       #     camera.release()
       #     cv2.destroyAllWindows()
       #      break   
    
while camera.isOpened:
    NoteDetetction()
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break
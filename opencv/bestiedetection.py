import numpy as np
import cv2

kernal = np.ones((7,7), "uint8")
camera = cv2.VideoCapture(0) # First webcam (video0)
 
while camera.isOpened():
    success, frame = camera.read()
    if not success:
        break
    hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    cv2.imshow('raw', frame)

    # lower range of red color in HSV
    lower_range = (80, 0, 0)
    upper_range = (90, 245, 245)
    mask = cv2.inRange(hsv_img, lower_range, upper_range)

    lower_range = (90,55,0)
    upper_range = (130,245,245)
    mask1 = cv2.inRange(hsv_img, lower_range, upper_range)

    mask = mask + mask1

    mask = cv2.erode(mask, kernal)
    mask = cv2.erode(mask, kernal)
    mask = cv2.dilate(mask, kernal)

    #frame = cv2.bitwise_and(frame, frame, mask=mask)

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate (contours):
        area = cv2.contourArea(contour)
        if (area > 28):
            x,y,w,h = cv2.boundingRect(contour)
            frame = cv2.rectangle(frame, (x,y), (x+w, y+h), (255,255,0),2)

            cv2.putText(frame, "Enemy!", (x,y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,0))


    # Display the color of the image
    cv2.imshow('Highlighted', frame)
    if cv2.waitKey(10) & 0xFF == ord('q'):
        camera.release()
        cv2.destroyAllWindows()
        break
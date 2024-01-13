
import cv2
import numpy as np
img = cv2.imread('new_image.jpg')
cv2.imshow('Original Image', img)
cv2.waitKey(0)
hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# lower range of red color in HSV
lower_range = (0, 100, 50)

# upper range of red color in HSV
upper_range = (10, 255, 255)


mask = cv2.inRange(hsv_img, lower_range, upper_range)


lower_range = (170,100,50)
upper_range = (180,255,255)
mask1 = cv2.inRange(hsv_img, lower_range, upper_range)

mask = mask + mask1

color_image = cv2.bitwise_and(img, img, mask=mask)

# Display the color of the image
cv2.imshow('Coloured Image', color_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
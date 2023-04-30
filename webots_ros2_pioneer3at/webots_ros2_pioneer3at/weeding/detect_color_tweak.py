import cv2
import numpy as np

def getBlobPositionInImage(color, lower_boud, upper_bound):
    color = cv2.cvtColor(color, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(color, lower_boud, upper_bound)
    color = cv2.bitwise_and(color, color, mask=mask)

    kernel = np.ones((5,5), np.uint8)
    floorplan = cv2.dilate(color, kernel, iterations=1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    x = y = 0
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        moments = cv2.moments(largest_contour)
        if moments["m00"] != 0:
            x = int(moments["m10"] / moments["m00"])
            y = int(moments["m01"] / moments["m00"])
    return color

path = "/home/bresilla/webots/webo/images/front_camera_504.png"
img = cv2.imread(path)
# hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
# bound_lower = np.array([28, 17, 20])
# bound_upper = np.array([85, 255, 255])

# mask_green = cv2.inRange(hsv_img, bound_lower, bound_upper)
# kernel = np.ones((7,7),np.uint8)

# mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)
# mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)

# seg_img = cv2.bitwise_and(img, img, mask=mask_green)
# contours, hier = cv2.findContours(mask_green.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# output = cv2.drawContours(seg_img, contours, -1, (0, 0, 255), 3)

# cv2.imshow("opencv", output)
# while True:
#     key = cv2.waitKey(1)
#     if key == ord('q'):
#         break
# cv2.destroyAllWindows()


def nothing(x):
    pass

# Convert the image to the HSV color space
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# Create window
cv2.namedWindow("Result")

# Create sliders for lower and upper bounds of the blue color
cv2.createTrackbar("Lower H", "Result", 0, 179, nothing)
cv2.createTrackbar("Lower S", "Result", 0, 255, nothing)
cv2.createTrackbar("Lower V", "Result", 0, 255, nothing)
cv2.createTrackbar("Upper H", "Result", 179, 179, nothing)
cv2.createTrackbar("Upper S", "Result", 255, 255, nothing)
cv2.createTrackbar("Upper V", "Result", 255, 255, nothing)

while True:
    # Get values from sliders
    lower_h = cv2.getTrackbarPos("Lower H", "Result")
    lower_s = cv2.getTrackbarPos("Lower S", "Result")
    lower_v = cv2.getTrackbarPos("Lower V", "Result")
    upper_h = cv2.getTrackbarPos("Upper H", "Result")
    upper_s = cv2.getTrackbarPos("Upper S", "Result")
    upper_v = cv2.getTrackbarPos("Upper V", "Result")
    
    # Define the lower and upper bounds of the blue color
    lower_bound = np.array([lower_h, lower_s, lower_v])
    upper_bound = np.array([upper_h, upper_s, upper_v])
    
    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    
    # Bitwise-AND of the original image and the mask to get the blue object
    result = cv2.bitwise_and(img, img, mask=mask)
    
    # Show the result
    cv2.imshow("Result", result)
    
    # Break the loop if the 'q' key is pressed
    key = cv2.waitKey(1)
    if key == ord('q'):
        break

# Close all windows
cv2.destroyAllWindows()
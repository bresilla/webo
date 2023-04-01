import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import Float32, Int32
from cv_bridge import CvBridge
import message_filters
import numpy as np
import matplotlib.pyplot as plt

def gauss(img):
    green = img[:, :, 1]
    green_flat = green.T.flatten()
    y_coords = np.arange(green_flat.shape[0])
    hist, bins = np.histogram(y_coords, bins=20, range=(0, img.shape[0]))
    mu, sigma = np.mean(y_coords), np.std(y_coords)
    pdf = 1 / (sigma * np.sqrt(2 * np.pi)) * np.exp(-0.5 * ((bins - mu) / sigma) ** 2)
    fig, ax = plt.subplots()
    ax.hist(y_coords, bins=20, range=(0, img.shape[0]), density=True, alpha=0.5, color='blue')
    ax.plot(bins, pdf, color='red')
    ax.set_xlabel('Y-coordinate')
    ax.set_ylabel('Density')
    plt.show()

camera_front = []
camera_back = []


def blober(img, array):
    y, x = int(img.shape[0]/2), 0
    h, w = 150, img.shape[1]
    roi = img[y:y+h, x:x+w]
    img = roi

    img = cv2.GaussianBlur(img, (9, 9), 0)
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    bound_lower = np.array([30, 30, 0])
    bound_upper = np.array([90, 255, 255])

    mask_green = cv2.inRange(hsv_img, bound_lower, bound_upper)

    kernel = np.ones((7,7),np.uint8)
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
    mask_green = cv2.dilate(mask_green, kernel, iterations=1)
    mask_green = cv2.erode(mask_green, kernel, iterations=1)

    seg_img = cv2.bitwise_and(img, img, mask=mask_green)
    contours, hier = cv2.findContours(mask_green.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    min_area = 500
    large_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]

    for e in large_contours:
        moments = cv2.moments(e)
        if moments["m00"] != 0:
            x = int(moments["m10"] / moments["m00"])
            y = int(moments["m01"] / moments["m00"])
            cv2.circle(seg_img, (x, y), 5, (0, 255, 0), -1)

    output = cv2.drawContours(seg_img, large_contours, -1, (0, 0, 255), 3)
    return output, seg_img


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__("image_subscriber")
        self.cv_bridge = CvBridge()

        self.image_back = message_filters.Subscriber(self, Image, '/camera_back/image_raw')
        self.image_front = message_filters.Subscriber(self, Image, '/camera_front/image_raw')
        self.navsatfix = message_filters.Subscriber(self, NavSatFix, '/gps/gps')
        self.distance = message_filters.Subscriber(self, Float32, '/gps/distance')
        self.pixels_front = self.create_publisher(Int32, "/pixels/front", 10)
        self.pixels_back = self.create_publisher(Int32, "/pixels/back", 10)


        self.back_sub = message_filters.ApproximateTimeSynchronizer([self.image_back, self.navsatfix], 10, slop=10)
        self.back_sub.registerCallback(self.camera_back)
        
        self.front_sub = message_filters.ApproximateTimeSynchronizer([self.image_front, self.navsatfix], 10, slop=10)
        self.front_sub.registerCallback(self.camera_front)

    def camera_front(self, img, gps):
        global camera_front
        # print(gps.latitude)
        # print(gps.longitude)
        camera_front = []
        image = self.cv_bridge.imgmsg_to_cv2(img)
        image, _ = counter(image, self.pixels_front)
        cv2.imshow("CAM_FRONT", image)
        cv2.waitKey(1)

    def camera_back(self, img, gps):
        global camera_back
        # print(gps.latitude)
        # print(gps.longitude)
        camera_back = []
        image = self.cv_bridge.imgmsg_to_cv2(img)
        image = cv2.flip(image, 1)
        image, _ = counter(image, self.pixels_back)
        cv2.imshow("CAM_BACK", image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
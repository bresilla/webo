import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import Float32, Int32
from handy_msgs.msg import Tag, Tags, Float32Stamped
from cv_bridge import CvBridge
import message_filters
import numpy as np
import matplotlib.pyplot as plt


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__("blob_detector")
        self.bridge = CvBridge()
        self.offset = 1.07

        self.distance = message_filters.Subscriber(self, Float32Stamped, '/gps/distance')
        self.image_back = message_filters.Subscriber(self, Image, '/Pioneer3at/camera_back')
        self.image_front = message_filters.Subscriber(self, Image, '/Pioneer3at/camera_front')
        self.navsatfix = message_filters.Subscriber(self, NavSatFix, '/gps/fix')

        self.image_front_pub = self.create_publisher(Image, '/Pioneer3at/camera_front/blobs', 10)
        self.tags_front = self.create_publisher(Tags, '/Pioneer3at/camera_front/tags', 10)
        self.image_back_pub = self.create_publisher(Image, '/Pioneer3at/camera_back/blobs', 10)
        self.tags_back = self.create_publisher(Tags, '/Pioneer3at/camera_back/tags', 10)

        self.back_sub = message_filters.ApproximateTimeSynchronizer([self.image_back, self.distance], 10, slop=10)
        self.back_sub.registerCallback(self.camera_back)
    
        self.front_sub = message_filters.ApproximateTimeSynchronizer([self.image_front, self.distance], 10, slop=10)
        self.front_sub.registerCallback(self.camera_front)

    def camera_front(self, img, gps):
        # print(gps.latitude)
        # print(gps.longitude)
        image = self.bridge.imgmsg_to_cv2(img)
        image, tags = self.blober(image, img.header.stamp)
        self.tags_front.publish(tags)
        labeled_frame = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        labeled_frame.header.stamp = img.header.stamp
        self.image_front_pub.publish(labeled_frame)

    def camera_back(self, img, gps):
        # print(gps.latitude)
        # print(gps.longitude)
        image = self.bridge.imgmsg_to_cv2(img)
        image = cv2.flip(image, 1)
        image, tags = self.blober(image, img.header.stamp)
        self.tags_back.publish(tags)
        labeled_frame = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        labeled_frame.header.stamp = img.header.stamp
        self.image_back_pub.publish(labeled_frame)

    def blober(self, img, img_header_stamp):
        img = cv2.GaussianBlur(img, (9, 9), 0)
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        bound_lower = np.array([50, 70, 0])
        bound_upper = np.array([120, 255, 127])
        # bound_lower = np.array([30, 30, 0])
        # bound_upper = np.array([100, 160, 255 ])

        mask_green = cv2.inRange(hsv_img, bound_lower, bound_upper)

        kernel = np.ones((7,7),np.uint8)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
        mask_green = cv2.dilate(mask_green, kernel, iterations=1)
        mask_green = cv2.erode(mask_green, kernel, iterations=1)

        seg_img = cv2.bitwise_and(img, img, mask=mask_green)
        contours, hier = cv2.findContours(mask_green.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        min_area = 1000
        large_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]

        tags = Tags()
        for e in large_contours:
            tag = Tag()
            tag.x, tag.y, tag.w, tag.h = cv2.boundingRect(e)
            tag.l = "weed"
            tag.d = 0
            tag.p = 0.9
            tags.data.append(tag)
            cv2.rectangle(seg_img, (tag.x, tag.y), (tag.x+tag.w, tag.y+tag.h), (0, 255, 0), 2)

            moments = cv2.moments(e)
            if moments["m00"] != 0:
                xc = int(moments["m10"] / moments["m00"])
                yc = int(moments["m01"] / moments["m00"])
                cv2.circle(seg_img, (xc, yc), 5, (0, 255, 0), -1)
        tags.header.stamp = img_header_stamp
        tags.header.frame_id = "tags"

        output = cv2.drawContours(seg_img, large_contours, -1, (0, 0, 255), 3)
        output = cv2.cvtColor(output, cv2.COLOR_RGBA2BGR)
        return output, tags

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
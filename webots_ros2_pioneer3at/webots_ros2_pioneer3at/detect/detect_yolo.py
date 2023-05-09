import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from handy_msgs.msg import Tag, Tags, Float32Stamped
from cv_bridge import CvBridge
import message_filters
import numpy as np
from ultralytics import YOLO


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__("blob_detector")
        self.bridge = CvBridge()
        self.offset = 1.07

        self.distance = message_filters.Subscriber(self, Float32Stamped, '/gps/distance')
        self.image_back = message_filters.Subscriber(self, Image, '/Pioneer3at/camera_back')
        self.image_front = message_filters.Subscriber(self, Image, '/Pioneer3at/camera_front')
        self.navsatfix = message_filters.Subscriber(self, NavSatFix, '/gps/fix')

        self.image_front_pub = self.create_publisher(Image, '/Pioneer3at/camera_front/yolo', 10)
        self.tags_front = self.create_publisher(Tags, '/Pioneer3at/camera_front/tags', 10)
        self.image_back_pub = self.create_publisher(Image, '/Pioneer3at/camera_back/yolo', 10)
        self.tags_back = self.create_publisher(Tags, '/Pioneer3at/camera_back/tags', 10)

        self.back_sub = message_filters.ApproximateTimeSynchronizer([self.image_back, self.distance], 10, slop=10)
        self.back_sub.registerCallback(self.camera_back)
    
        self.front_sub = message_filters.ApproximateTimeSynchronizer([self.image_front, self.distance], 10, slop=10)
        self.front_sub.registerCallback(self.camera_front)

    def camera_front(self, img, distance):
        image = self.bridge.imgmsg_to_cv2(img)
        image, tags = self.gen_bboxes(image, img.header.stamp)
        self.tags_front.publish(tags)
        labeled_frame = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        self.image_front_pub.publish(labeled_frame)

    def camera_back(self, img, distance):
        image = self.bridge.imgmsg_to_cv2(img)
        image = cv2.flip(image, 1)
        image, tags = self.gen_bboxes(image, img.header.stamp)
        self.tags_back.publish(tags)
        labeled_frame = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        self.image_back_pub.publish(labeled_frame)

    def gen_bboxes(self, img, img_header_stamp):
        weights = "/home/bresilla/webots/webo/best.pt"
        model = YOLO(weights)
        results = model(img)
        tags = Tags()
        for r in results:           
            boxes = r.boxes
            for box in boxes:
                c = box.xywh[0].tolist()
                tag = Tag()
                tag.x, tag.y, tag.w, tag.h = int(c[0]-(c[2]/2)), int(c[1]-(c[3]/2)), int(c[2]), int(c[3])
                tag.l = "weed"
                tag.d = 0
                tag.p = box.conf[0].tolist()
                tags.data.append(tag)
        tags.header.stamp = img_header_stamp
        tags.header.frame_id = "tags"
        img_clone = img.copy()
        for tag in tags.data:
            x1, y1, x2, y2 = tag.x, tag.y, tag.x+tag.w, tag.y+tag.h
            cv2.rectangle(img_clone, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 3)
        return img_clone, tags

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
from typing import List
import numpy as np

import norfair
from norfair import Detection, Tracker, Paths

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters
from handy_msgs.msg import Tag, Tags, Float32Stamped
import cv2


class Trackit(Tracker):
    def __init__(self, trackt):
        super().__init__(
            distance_function=self.iou if trackt == "bbox" else self.euclidean_distance,
            distance_threshold=3.33 if trackt == "bbox" else 30,
            initialization_delay=10,
            )
        self.track_form = trackt
        self.pather = Paths(thickness=2, attenuation=0.1)

    def iou(self, detection, tracked_object):
        box_a = np.concatenate([detection.points[0], detection.points[1]])
        box_b = np.concatenate([tracked_object.estimate[0], tracked_object.estimate[1]])
        x_a = max(box_a[0], box_b[0])
        y_a = max(box_a[1], box_b[1])
        x_b = min(box_a[2], box_b[2])
        y_b = min(box_a[3], box_b[3])
        inter_area = max(0, x_b - x_a + 1) * max(0, y_b - y_a + 1)
        box_a_area = (box_a[2] - box_a[0] + 1) * (box_a[3] - box_a[1] + 1)
        box_b_area = (box_b[2] - box_b[0] + 1) * (box_b[3] - box_b[1] + 1)
        iou = inter_area / float(box_a_area + box_b_area - inter_area)
        return 1 / iou if iou else 10000

    def euclidean_distance(self, detection, tracked_object):
        return np.linalg.norm(detection.points - tracked_object.estimate)


    def gen_detections(self, tags) -> List[Detection]:
        norfair_detections: List[Detection] = []
        for detection in tags.data:
            if self.track_form == "centroid":
                points = np.array([
                    detection.x,
                    detection.y
                ])
                scores = np.array([detection.p])
            elif self.track_form == "bbox":
                points = np.array([
                    [detection.x, detection.y],
                    [detection.x + detection.w, detection.y + detection.h]
                ])
                scores = np.array([detection.p, detection.p])
            norfair_detections.append(Detection(points=points, scores=scores))
        return norfair_detections
    

    def draw(self, frame, tracked_objects):
        if self.track_form == "centroid":
            norfair.draw_points(frame, drawables=tracked_objects, draw_ids=True, text_size=1.5)
        elif self.track_form == "bbox":
            norfair.draw_boxes(frame, drawables=tracked_objects, draw_ids=True, text_size=1.5)
        return frame, tracked_objects


class Tracktor(Node):
    def __init__(self):
        super().__init__('tracker_node')
        self.bridge = CvBridge()
        self.bgr = (0, 255, 0)
        self.tracker_front = Trackit("bbox")
        self.tracker_back = Trackit("bbox")


        self.image_front_pub = self.create_publisher(Image, '/Pioneer3at/camera_front/tracker', 10)
        self.image_back_pub = self.create_publisher(Image, '/Pioneer3at/camera_back/tracker', 10)
        
        self.distance = message_filters.Subscriber(self, Float32Stamped, '/gps/distance')
        self.image_front = message_filters.Subscriber(self, Image, '/Pioneer3at/camera_front')
        self.image_back = message_filters.Subscriber(self, Image, '/Pioneer3at/camera_back')

        self.front_ts = message_filters.ApproximateTimeSynchronizer([self.image_front, self.distance], 10, slop=10)
        self.front_ts.registerCallback(self.callback_front)
        self.back_ts = message_filters.ApproximateTimeSynchronizer([self.image_back, self.distance], 10, slop=10)
        self.back_ts.registerCallback(self.callback_back)


    def callback_front(self, image, distance):
        frame = self.bridge.imgmsg_to_cv2(image, "bgr8")
        img, tags = self.blober(frame)

        detections = self.tracker_front.gen_detections(tags)
        tracked_objects = self.tracker_front.update(detections=detections)
        frame2 = self.tracker_front.pather.draw(frame.copy(), tracked_objects)
        frame = cv2.add(frame, frame2)
        self.tracker_front.draw(frame, tracked_objects)

        labeled_frame = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_front_pub.publish(labeled_frame)


    def callback_back(self, image, distance):
        frame = self.bridge.imgmsg_to_cv2(image, "bgr8")
        img, tags = self.blober(frame)

        detections = self.tracker_back.gen_detections(tags)
        tracked_objects = self.tracker_back.update(detections=detections)
        frame2 = self.tracker_back.pather.draw(frame, tracked_objects)
        frame = cv2.add(frame, frame2)
        self.tracker_back.draw(frame, tracked_objects)

        labeled_frame = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_back_pub.publish(labeled_frame)
        

    def blober(self, img):
        # y, x = int(img.shape[0]/2), 0
        # h, w = 150, img.shape[1]
        # roi = img[y:y+h, x:x+w]
        # img = roi

        img = cv2.GaussianBlur(img, (9, 9), 0)
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        bound_lower = np.array([50, 70, 0])
        bound_upper = np.array([120, 255, 127])

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
        tags.header.stamp = self.get_clock().now().to_msg()
        tags.header.frame_id = "tags"

        output = cv2.drawContours(seg_img, large_contours, -1, (0, 0, 255), 3)
        return output, tags


def main():
    rclpy.init(args=None)
    tracktor = Tracktor()
    rclpy.spin(tracktor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

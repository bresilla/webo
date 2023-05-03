from typing import List
import numpy as np

import torch
import torchvision.ops.boxes as bops

import norfair
from norfair import Detection, Tracker, Paths
from supervision.detection.line_counter import LineZone, LineZoneAnnotator
from supervision.detection.core import Detections
from supervision.geometry.core import Point
from example_interfaces.srv import Trigger



import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters
from handy_msgs.msg import Tag, Tags, Float32Stamped
import cv2

def place_text_with_background(frame, text, position, background_color=(255, 255, 255), text_color=(0, 0, 0), font_scale=0.8, font_thickness=2, padding=6):
    text_size, _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_thickness)
    text_w, text_h = text_size
    text_x, text_y = position
    cv2.rectangle(frame, (text_x - padding, text_y - padding*2), (text_x + text_w + padding, text_y + text_h + padding*2), background_color, cv2.FILLED)
    cv2.putText(frame, text, (text_x, text_y + text_h), cv2.FONT_HERSHEY_SIMPLEX, font_scale, text_color, font_thickness)

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
        return 1 / iou if iou else 10
    
    def iou_pytorch(self, detection, tracked_object):
        detection_points = np.concatenate([detection.points[0], detection.points[1]])
        tracked_object_points = np.concatenate([tracked_object.estimate[0], tracked_object.estimate[1]])
        box_a = torch.tensor([detection_points]) 
        box_b = torch.tensor([tracked_object_points])
        iou = bops.box_iou(box_a, box_b)
        floater = float(1 / iou if iou else 10000)
        return floater
    
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
    def __init__(self, name, sub_name, pub_name):
        super().__init__(name)
        self.bridge = CvBridge()
        self.bgr = (0, 255, 0)
        self.trackit = Trackit("bbox")
        self.offset = 1.1
        self.dimensions = None
        self.annotator = LineZoneAnnotator(thickness=2, text_thickness=2, text_scale=0.8)
        self.counter = None
        self.detections = 0

        distance_reset = self.create_client(Trigger, '/gps/reset_distance')
        distance_reset.call_async(Trigger.Request())

        self.image_sub = message_filters.Subscriber(self, Image, sub_name)
        self.image_pub = self.create_publisher(Image, pub_name, 10)
        self.distance = message_filters.Subscriber(self, Float32Stamped, '/gps/distance')
        self.time_sync = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.distance], 10, slop=10)
        self.time_sync.registerCallback(self.callback)


    def callback(self, image, distance):
        frame = self.bridge.imgmsg_to_cv2(image, "bgr8") 
        tags = self.gen_bboxes(frame)
        detections = self.trackit.gen_detections(tags)
        tracked_objects = self.trackit.update(detections=detections)
        frame2 = self.trackit.pather.draw(frame, tracked_objects)
        frame = cv2.add(frame, frame2)
        self.trackit.draw(frame, tracked_objects)
        if self.counter is None: 
            self.counter = LineZone(Point(20, int(self.dimensions[0]/2)), Point(self.dimensions[1]-20, int(self.dimensions[0]/2)))
        else:
            xyxy = np.empty((0,4))
            tracker_id = np.empty((0,))
            for obj in tracked_objects:
                xyxy = np.append(xyxy, [obj.estimate.flatten()], axis=0)
                tracker_id = np.append(tracker_id, [obj.id], axis=0)
            detections = Detections(xyxy=xyxy, tracker_id=tracker_id)
            self.counter.trigger(detections)
            self.detections = abs(self.counter.in_count-self.counter.out_count)
            self.annotator.annotate(frame=frame, line_counter=self.counter)
            place_text_with_background(frame, f"Weeds: {'{: >3}'.format(self.detections)}", (self.dimensions[1]-200, int(self.dimensions[0]/2)-10))
            place_text_with_background(frame, f"Dist: {'{:.2f}'.format(distance.data)}", (50, int(self.dimensions[0]/2)-10))
        labeled_frame = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_pub.publish(labeled_frame)

    def gen_bboxes(self, img, min_area=1000):
        self.dimensions = img.shape if self.dimensions is None else self.dimensions
        img = cv2.GaussianBlur(img, (9, 9), 0)
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # bound_lower = np.array([50, 70, 0])
        # bound_upper = np.array([120, 255, 127])
        bound_lower = np.array([30, 30, 0])
        bound_upper = np.array([100, 160, 255 ])
        mask_green = cv2.inRange(hsv_img, bound_lower, bound_upper)
        kernel = np.ones((7,7),np.uint8)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
        mask_green = cv2.dilate(mask_green, kernel, iterations=1)
        mask_green = cv2.erode(mask_green, kernel, iterations=1)
        contours, _ = cv2.findContours(mask_green.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        large_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]
        tags = Tags()
        for e in large_contours:
            tag = Tag()
            tag.x, tag.y, tag.w, tag.h = cv2.boundingRect(e)
            tag.l = "weed"
            tag.d = 0
            tag.p = 0.9
            tags.data.append(tag)
        tags.header.stamp = self.get_clock().now().to_msg()
        tags.header.frame_id = "tags"
        return tags

def main(args=None):
    rclpy.init(args=args)
    try:
        tracktor_front = Tracktor('tracktor_front', '/Pioneer3at/camera_front', '/Pioneer3at/camera_front/tracker')
        tracktor_back = Tracktor('tracktor_back', '/Pioneer3at/camera_back', '/Pioneer3at/camera_back/tracker')
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(tracktor_front)
        executor.add_node(tracktor_back)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            tracktor_front.destroy_node()
            tracktor_back.destroy_node()
    finally:
        rclpy.shutdown()

def main2():
    rclpy.init(args=None)
    tracktor = Tracktor('tracktor_front', '/Pioneer3at/camera_front', '/Pioneer3at/camera_front/tracker')
    rclpy.spin(tracktor)
    rclpy.shutdown()

if __name__ == '__main__':
    main2()
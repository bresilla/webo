from deepsort.tracker import Tracker as DeepSortTracker
import deepsort.generate_detections as gdet
from deepsort import nn_matching
from deepsort.detection import Detection

import numpy as np
from ultralytics import YOLO

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

class Trackit:
    tracker = None
    encoder = None
    tracks = None

    def __init__(self):
        max_cosine_distance = 0.4
        nn_budget = None
        encoder_model_filename = '/home/bresilla/webots/webo/mars-small128.pb'
        metric = nn_matching.NearestNeighborDistanceMetric("cosine", max_cosine_distance, nn_budget)
        self.tracker = DeepSortTracker(metric)
        self.encoder = gdet.create_box_encoder(encoder_model_filename, batch_size=1)

    def update(self, frame, detections):
        bboxes = np.asarray([d[:-1] for d in detections])
        bboxes[:, 2:] = bboxes[:, 2:] - bboxes[:, 0:2]
        scores = [d[-1] for d in detections]
        features = self.encoder(frame, bboxes)
        dets = []
        for bbox_id, bbox in enumerate(bboxes):
            dets.append(Detection(bbox, scores[bbox_id], features[bbox_id]))
        self.tracker.predict()
        self.tracker.update(dets)
        self.update_tracks()

    def update_tracks(self):
        tracks = []
        for track in self.tracker.tracks:
            if not track.is_confirmed() or track.time_since_update > 1:
                continue
            bbox = track.to_tlbr()
            id = track.track_id
            tracks.append(Track(id, bbox))
        self.tracks = tracks

    def gen_detections(self, tags):
        detections = []
        for tag in tags.data:
            detections.append([tag.x, tag.y, tag.x+tag.w, tag.y+tag.h, tag.p])
        return detections

class Track:
    track_id = None
    bbox = None

    def __init__(self, id, bbox):
        self.track_id = id
        self.bbox = bbox

import random
colors = [(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)) for j in range(10)]

class Tracktor(Node):
    def __init__(self):
        super().__init__('tracker_node')
        self.bridge = CvBridge()
        self.bgr = (0, 255, 0)
        self.offset = 1.1
        self.dimensions = None

        self.tracker_front = Trackit()
        self.annotator_front = LineZoneAnnotator(thickness=2, text_thickness=2, text_scale=0.8)
        self.counter_front = None
        self.detections_front = 0
        self.detections_front_array = []

        self.tracker_back = Trackit()
        self.annotator_back = LineZoneAnnotator(thickness=2, text_thickness=2, text_scale=0.8)
        self.counter_back = None
        self.detections_back = 0
        self.detections_back_array = []

        self.image_front_pub = self.create_publisher(Image, '/Pioneer3at/camera_front/tracker', 10)
        self.image_back_pub = self.create_publisher(Image, '/Pioneer3at/camera_back/tracker', 10)
        
        self.distance = message_filters.Subscriber(self, Float32Stamped, '/gps/distance')
        self.image_front = message_filters.Subscriber(self, Image, '/Pioneer3at/camera_front')
        self.tags_front = message_filters.Subscriber(self, Tags, '/Pioneer3at/camera_front/tags')
        self.image_back = message_filters.Subscriber(self, Image, '/Pioneer3at/camera_back')
        self.tags_back = message_filters.Subscriber(self, Tags, '/Pioneer3at/camera_back/tags')

        self.front_ts = message_filters.ApproximateTimeSynchronizer([self.image_front, self.tags_front, self.distance], 10, slop=10)
        self.front_ts.registerCallback(self.callback_front)
        # self.back_ts = message_filters.ApproximateTimeSynchronizer([self.image_back, self.tags_back, self.distance], 10, slop=10)
        # self.back_ts.registerCallback(self.callback_back)


    def callback_front(self, image, tags, distance):
        frame = self.bridge.imgmsg_to_cv2(image, "bgr8")
        self.dimensions = frame.shape if self.dimensions is None else self.dimensions
        detections = self.tracker_front.gen_detections(tags)
        tracked_objects = self.tracker_front.update(frame, detections)
        if self.counter_front is None: 
            self.counter_front = LineZone(Point(20, int(self.dimensions[0]/2)), Point(self.dimensions[1]-20, int(self.dimensions[0]/2)))
        else:
            xyxy = np.empty((0,4))
            tracker_id = np.empty((0,))
            for track in self.tracker_front.tracks:
                bbox = track.bbox.astype(np.int)
                x1, y1, x2, y2 = bbox
                track_id = track.track_id
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (colors[track_id % len(colors)]), 3)
                xyxy = np.append(xyxy, [bbox], axis=0)
                tracker_id = np.append(tracker_id, [track_id], axis=0)
            detections = Detections(xyxy=xyxy, tracker_id=tracker_id)
            self.counter_front.trigger(detections)
            self.detections_front = abs(self.counter_front.in_count-self.counter_front.out_count)
            self.annotator_front.annotate(frame=frame, line_counter=self.counter_front)
            place_text_with_background(frame, f"Weeds: {'{: >3}'.format(self.detections_front)}", (self.dimensions[1]-200, int(self.dimensions[0]/2)-10))
            # place_text_with_background(frame, f"Dist: {'{:.2f}'.format(distance.data)}", (50, int(self.dimensions[0]/2)-10))
        labeled_frame = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_front_pub.publish(labeled_frame)

    def callback_back(self, image, tags, distance):
        frame = self.bridge.imgmsg_to_cv2(image, "bgr8")
        self.dimensions = frame.shape if self.dimensions is None else self.dimensions
        frame = cv2.flip(frame, 1)
        detections = self.tracker_back.gen_detections(tags)
        tracked_objects = self.tracker_back.update(frame, detections)
        # frame2 = self.tracker_back.pather.draw(frame, tracked_objects)
        # frame = cv2.add(frame, frame2)
        # self.tracker_back.draw(frame, tracked_objects)
        if self.counter_back is None: 
            self.counter_back = LineZone(Point(20, int(self.dimensions[0]/2)), Point(self.dimensions[1]-20, int(self.dimensions[0]/2)))
        else:
            xyxy = np.empty((0,4))
            tracker_id = np.empty((0,))
            for obj in tracked_objects:
                xyxy = np.append(xyxy, [obj.estimate.flatten()], axis=0)
                tracker_id = np.append(tracker_id, [obj.id], axis=0)
            detections = Detections(xyxy=xyxy, tracker_id=tracker_id)
            self.counter_back.trigger(detections)
            self.detections_back = abs(self.counter_back.in_count-self.counter_back.out_count)
            self.annotator_back.annotate(frame=frame, line_counter=self.counter_back)
            place_text_with_background(frame, f"Weeds: {'{: >3}'.format(self.detections_back)}", (self.dimensions[1]-200, int(self.dimensions[0]/2)-10))
            # place_text_with_background(frame, f"Dist: {'{:.2f}'.format(distance.data)}", (50, int(self.dimensions[0]/2)-10))
        labeled_frame = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_back_pub.publish(labeled_frame)
        

def main():
    print("---------------------------")
    rclpy.init(args=None)
    tracktor = Tracktor()
    rclpy.spin(tracktor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
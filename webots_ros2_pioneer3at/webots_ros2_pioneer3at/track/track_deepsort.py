from deepsort.tracker import Tracker as DeepSortTracker
import deepsort.generate_detections as gdet
from deepsort import nn_matching
from deepsort.detection import Detection

import numpy as np
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
import random


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
    
    def draw_tracks(self, frame):
        for track in self.tracks:
            bbox = track.bbox.astype(np.int)
            x1, y1, x2, y2 = bbox
            track_id = track.track_id
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), colors[track_id % 10], 2)
            place_text_with_background(frame, str(track_id), (int(x1), int(y1)), colors[track_id % 10])
colors = [(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)) for j in range(10)]

class Track:
    track_id = None
    bbox = None

    def __init__(self, id, bbox):
        self.track_id = id
        self.bbox = bbox

class Tracktor(Node):
    def __init__(self, name, img_sub_name, tag_sub_name, pub_name):
        super().__init__(name)
        self.bridge = CvBridge()
        self.bgr = (0, 255, 0)
        self.trackit = Trackit()
        self.offset = 1.1
        self.dimensions = None
        self.annotator = LineZoneAnnotator(thickness=2, text_thickness=2, text_scale=0.8)
        self.counter = None
        self.detections = 0

        distance_reset = self.create_client(Trigger, '/gps/reset_distance')
        distance_reset.call_async(Trigger.Request())

        self.image_pub = self.create_publisher(Image, pub_name, 10)
        self.distance = message_filters.Subscriber(self, Float32Stamped, '/gps/distance')
        self.image_sub = message_filters.Subscriber(self, Image, img_sub_name)
        self.tags = message_filters.Subscriber(self, Tags, tag_sub_name)
        self.time_sync = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.tags, self.distance], 10, slop=10)
        self.time_sync.registerCallback(self.callback)


    def callback(self, image, tags, distance):
        frame = self.bridge.imgmsg_to_cv2(image, "bgr8")
        self.dimensions = frame.shape if self.dimensions is None else self.dimensions
        detections = self.trackit.gen_detections(tags)
        tracked_objects = self.trackit.update(frame, detections)
        if self.counter is None: 
            self.counter = LineZone(Point(20, int(self.dimensions[0]/2)), Point(self.dimensions[1]-20, int(self.dimensions[0]/2)))
        else:
            xyxy = np.empty((0,4))
            tracker_id = np.empty((0,))
            for track in self.trackit.tracks:
                bbox = track.bbox.astype(np.int)
                xyxy = np.append(xyxy, [bbox], axis=0)
                track_id = track.track_id
                tracker_id = np.append(tracker_id, [track_id], axis=0)
                x1, y1, x2, y2 = bbox
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), colors[track_id % 10], 2)
                place_text_with_background(frame, str(track_id), (int(x1), int(y1)), colors[track_id % 10])
            # self.trackit.draw_tracks(frame)
            detections = Detections(xyxy=xyxy, tracker_id=tracker_id)
            self.counter.trigger(detections)
            self.detections = abs(self.counter.in_count-self.counter.out_count)
            self.annotator.annotate(frame=frame, line_counter=self.counter)
            place_text_with_background(frame, f"Weeds: {'{: >3}'.format(self.detections)}", (self.dimensions[1]-200, int(self.dimensions[0]/2)-10))
            place_text_with_background(frame, f"Dist: {'{:.2f}'.format(distance.data)}", (50, int(self.dimensions[0]/2)-10))
        labeled_frame = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_pub.publish(labeled_frame)

def main(args=None):
    rclpy.init(args=args)
    try:
        tracktor_front = Tracktor(
            'tracktor_front', 
            '/Pioneer3at/camera_front',
            '/Pioneer3at/camera_front/tags',
            '/Pioneer3at/camera_front/tracker')
        tracktor_back = Tracktor(
            'tracktor_back', 
            '/Pioneer3at/camera_back',
            '/Pioneer3at/camera_back/tags',
            '/Pioneer3at/camera_back/tracker')
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
    tracktor = Tracktor(
        'tracktor_front', 
        '/Pioneer3at/camera_front',
        '/Pioneer3at/camera_front/tags',
        '/Pioneer3at/camera_front/tracker')
    rclpy.spin(tracktor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
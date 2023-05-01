import torch
import numpy as np
import cv2
from time import time

import rclpy
import message_filters
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int8
from yolobot_interfaces.msg import Tag, Tags

class Detectron(Node):
    def __init__(self):
        super().__init__('detectron_node')
        self.bridge = CvBridge()
        self.i = 1
        self.bgr = (0, 255, 0)
        self.threshoold = 0.3

        self.declare_parameters(
            namespace='',
            parameters=[('weight_file', '/home/bresilla/down/yolov5m.pt')]
        )
        detections_param = self.get_parameter('weight_file').get_parameter_value().string_value

        torch.hub._validate_not_a_forked_repo = lambda a, b, c: True
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=detections_param, force_reload=True)
        self.classes = self.model.names
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'

        self.tags = self.create_publisher(Tags, 'tags', 10)
        self.fps = self.create_publisher(Int8, 'fps_processing', 10)

        self.color_image = message_filters.Subscriber(self, Image, '/color/video/image'),
        self.depth_image = message_filters.Subscriber(self, Image, '/stereo/depth')

        # self.ts = message_filters.ApproximateTimeSynchronizer([self.color_image, self.depth_image], 10, slop=10)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [
                message_filters.Subscriber(self, Image, '/color/video/image'),
                message_filters.Subscriber(self, Image, '/stereo/depth')
            ], 10, slop=10)
        self.ts.registerCallback(self.callback)


    def score_frame(self, color_frame):
        self.model.to(self.device)
        color_frame = [color_frame]
        results = self.model(color_frame)
        labels, cords, pred = results.xyxyn[0][:, -1], results.xyxyn[0][:, :-2], results.xyxyn[0][:, -2]
        return labels, cords, pred

    def get_center_distance(self, depth_frame, x1, x2, y1, y2):
        x, y, w, h = x1, y1, abs(x2-x1), abs(y2-y1)
        image_flatten = np.array(depth_frame[y1:y2, x1:x2]).flatten()
        image_flatten = image_flatten[image_flatten != 0]
        return ret
        ret = 0 if image_flatten.size == 0 else int(image_flatten.mean())


    def get_box_distance(self, depth_frame, x1, x2, y1, y2):
        x, y, w, h = x1, y1, abs(x2-x1), abs(y2-y1)
        image_flatten = np.array(depth_frame[w:w, h:h]).flatten()
        image_flatten = image_flatten[image_flatten != 0]
        ret = 0 if image_flatten.size == 0 else int(image_flatten.mean())
        return ret

    def pub_each(self, results, color_frame, depth_frame):
        tags = Tags()
        n = len(results[0])
        x_shape, y_shape = color_frame.shape[1], color_frame.shape[0]
        for i in range(n):
            tag = Tag()
            row = results[1][i]
            pred = float(results[2][i])
            if pred >= self.threshoold:
                x1, y1, x2, y2 = int(row[0]*x_shape), int(row[1]*y_shape), int(row[2]*x_shape), int(row[3]*y_shape)
                tag.x, tag.y, tag.w, tag.h = x1, y1, abs(x2-x1), abs(y2-y1)
                tag.l = self.classes[int((results[0][i]))]
                # tag.d = self.get_box_distance(depth_frame, x1, x2, y1, y2)
                tag.d = self.get_center_distance(depth_frame, x1, x2, y1, y2)
                tag.p = pred
                tags.data.append(tag)
        tags.header.stamp = self.get_clock().now().to_msg()
        tags.header.frame_id = "tags"
        # tags.header.seq = self.i
        self.tags.publish(tags)

    def callback(self, color, depth):
        self.i += 1
        start_time = time()

        print("checking " + str(self.i))
        color_frame = self.bridge.imgmsg_to_cv2(color, "bgr8")
        depth_frame = self.bridge.imgmsg_to_cv2(depth, 'passthrough')

        results = self.score_frame(color_frame)
        self.pub_each(results, color_frame, depth_frame)

        end_time = time()
        fps_data = Int8()
        fps_data.data = int(1/np.round(end_time - start_time, 2))
        self.fps.publish(fps_data)

def main():
    rclpy.init(args=None)
    print('Hi from yolobot_detection.')
    detector = Detectron()
    rclpy.spin(detector)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from handy_msgs.msg import Float32Stamped
import readchar
import json

class MyNode(Node):
    def __init__(self, name, published_image):
        super().__init__(name)
        self.image_publisher = self.create_publisher(Image, published_image, 10)
        self.dist_publisher = self.create_publisher(Float32Stamped, '/gps/distance', 10)
        self.bridge = CvBridge()
        self.image_paths = []
        self.gps_paths = []

    def load_image_paths(self, folder_path):
        for file_name in os.listdir(folder_path):
            if file_name.endswith('.jpg') or file_name.endswith('.png'):
                self.image_paths.append(os.path.join(folder_path, file_name))
        new_array = sorted(self.image_paths, key=lambda name: name.lower())
        for filename in new_array:
            name, ext = os.path.splitext(filename)
            new_filename = name + '.json'
            self.gps_paths.append(new_filename)
        self.image_paths = new_array

    def publish(self, index, distance):
            file_name = self.image_paths[index]
            image = cv2.imread(file_name)
            ros_image = self.bridge.cv2_to_imgmsg(image, 'bgr8')
            self.image_publisher.publish(ros_image)
            with open(self.gps_paths[index]) as f:
                data = json.load(f)
                json_str = json.dumps(data)
                self.get_logger().info(f"JSON {json_str}")
                if distance:
                    new_msg = Float32Stamped()
                    new_msg.header = ros_image.header
                    new_msg.data = float(data['DIST'])
                    self.dist_publisher.publish(new_msg)
            self.get_logger().info(f"Published image {file_name}")


def main(args=None):
    rclpy.init(args=args)
    folder_path = '/doc/DATA/R4C/data/Cam/20230223143203'
    front_node = MyNode('image_player', '/Pioneer3at/camera_front')
    back_node = MyNode('image_player', '/Pioneer3at/camera_back')
    front_node.load_image_paths(folder_path + '/RGB-18443010C1A2DF0F00')
    back_node.load_image_paths(folder_path + '/RGB-18443010B1F4DE0F00')

    counter = 0
    while rclpy.ok():
        key = readchar.readkey()
        if key == ' ':
            front_node.publish(counter, distance=True)
            back_node.publish(counter, distance=True)
            counter += 1
        if key == 'r':
            counter = 0
        if key == 'b':
            counter -= 2


    front_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
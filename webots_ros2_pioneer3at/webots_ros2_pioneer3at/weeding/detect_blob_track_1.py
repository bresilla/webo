import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import Float32, Int32
from handy_msgs.msg import Float32Stamped
from cv_bridge import CvBridge
import message_filters
import numpy as np
import matplotlib.pyplot as plt


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__("image_subscriber")
        self.cv_bridge = CvBridge()
        self.offset = 1.07

        self.object_dict_front = {}
        self.object_count_front = 0
        self.front_points_current = []
        self.front_points_previous = []
        self.object_dict_back = {}
        self.object_count_back = 0
        self.back_points_current = []
        self.back_points_previous = []

        self.distance = message_filters.Subscriber(self, Float32Stamped, '/gps/distance')
        self.image_back = message_filters.Subscriber(self, Image, '/Pioneer3at/camera_back')
        self.image_front = message_filters.Subscriber(self, Image, '/Pioneer3at/camera_front')
        self.navsatfix = message_filters.Subscriber(self, NavSatFix, '/gps/fix')

        self.blobs_front_pub = self.create_publisher(Int32, "/blobs/front", 10)
        self.blobs_back_pub = self.create_publisher(Int32, "/blobs/back", 10)

        self.pixels_front = []
        self.pixels_back = []

        self.back_sub = message_filters.ApproximateTimeSynchronizer([self.image_back, self.distance], 10, slop=10)
        self.back_sub.registerCallback(self.camera_back)
    
        self.front_sub = message_filters.ApproximateTimeSynchronizer([self.image_front, self.distance], 10, slop=10)
        self.front_sub.registerCallback(self.camera_front)

    def camera_front(self, img, dist):
        # print(gps.latitude)
        # print(gps.longitude)
        camera_front = []
        image = self.cv_bridge.imgmsg_to_cv2(img)
        image, _ = self.blober(image, self.blobs_front_pub, "front")#, self.object_dict_front, self.object_count_front, self.front_points_current, self.front_points_previous)
        cv2.imshow("CAM_FRONT", image)
        cv2.waitKey(1)

    def camera_back(self, img, dist):
        # print(gps.latitude)
        # print(gps.longitude)
        camera_back = []
        image = self.cv_bridge.imgmsg_to_cv2(img)
        image = cv2.flip(image, 1)
        image, _ = self.blober(image, self.blobs_back_pub, "back")#, self.object_dict_back, self.object_count_back, self.back_points_current, self.back_points_previous)
        cv2.imshow("CAM_BACK", image)
        cv2.waitKey(1)

    def blober(self, img, array, title):

        if title == "front":
            prev_points = self.front_points_previous
            curr_points = self.front_points_current
            objects = self.object_dict_front
            count = self.object_count_front
        elif title == "back":
            prev_points = self.back_points_previous
            curr_points = self.back_points_current
            objects = self.object_dict_back
            count = self.object_count_back

        print("-----")
        print(title)
        print(len(prev_points))
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

        for e in large_contours:
            moments = cv2.moments(e)
            if moments["m00"] != 0:
                x = int(moments["m10"] / moments["m00"])
                y = int(moments["m01"] / moments["m00"])
                # cv2.circle(seg_img, (x, y), 5, (0, 255, 0), -1)
                curr_points.append((x, y))
            #     object_id = self.tracker(x, y, dict)
            #     if object_id is None:
            #         dict[count] = (x, y)
            #         count += 1
            #     else:
            #         dict[object_id] = (x, y)
            #         cv2.putText(seg_img, "Object {}".format(object_id), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            # cv2.putText(seg_img, "Object Count: {}".format(count), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        if count <= 2:
            for pt in curr_points:
                for pt2 in prev_points:
                    distance = np.hypot(pt2[0] - pt[0], pt2[1] - pt[1])
                    if distance < 50:
                        objects[count] = pt
                        count += 1
        else:
            objects_copy = objects.copy()
            curr_points_copy = curr_points.copy()

            for object_id, pt2 in objects_copy.items():
                object_exists = False
                for pt in curr_points_copy:
                    distance = np.hypot(pt2[0] - pt[0], pt2[1] - pt[1])
                    if distance < 20:
                        objects[object_id] = pt
                        object_exists = True
                        if pt in curr_points:
                            curr_points_copy.remove(pt)
                            continue
                if not object_exists:
                    objects.pop(object_id)
            
            for pt in curr_points:
                objects[count] = pt 
                count += 1

        for object_id, pt in objects.items():
            cv2.circle(seg_img, pt, 5, (0, 0, 255), -1)
            cv2.putText(seg_img, str(object_id), (pt[0] - 10, pt[1] - 10), 0, 1, (0, 0, 255), 2)

        if title == "front":
            self.front_points_current = curr_points
            self.front_points_previous = curr_points
            self.object_count_front = count
            self.object_dict_front = objects
        elif title == "back":
            self.back_points_current = curr_points
            self.back_points_previous = curr_points
            self.object_count_back = count
            self.object_dict_back = objects
        # print(len(curr_points))
        print(len(prev_points))
        output = cv2.drawContours(seg_img, large_contours, -1, (0, 0, 255), 3)
        return output, seg_img
    
    def tracker(self, centroid_x, centroid_y, object_dict):
        for object_id, centroid in object_dict.items():
            distance = np.sqrt((centroid_x - centroid[0])**2 + (centroid_y - centroid[1])**2)
            if distance < 10:
                return object_id
        return None

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
import math
import rclpy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from rclpy.node import Node
from haversine import haversine
from example_interfaces.srv import Trigger

def distance(coord1, coord2):
    """
    Calculate the great-circle distance (in meters) between two points on the Earth's surface.

    Parameters:
    coord1 (tuple): A tuple representing the first GPS coordinate in the form (latitude, longitude).
    coord2 (tuple): A tuple representing the second GPS coordinate in the form (latitude, longitude).

    Returns:
    float: The distance (in meters) between the two coordinates.

    """
    # Radius of the Earth in meters
    radius_earth = 6371008
    
    # Convert latitude and longitude to radians
    lat1, lon1 = map(math.radians, coord1)
    lat2, lon2 = map(math.radians, coord2)
    
    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    distance = radius_earth * c
    
    return distance

class Remapper(Node):
    def __init__(self, args):
        super().__init__("gps_fix")
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/gps', self.gps_callback, 10)
        self.dist_pub = self.create_publisher(Float32, "/gps/distance", 10)
        self.gps_pub = self.create_publisher(NavSatFix, "/gps/fix", 10)
        self.srv = self.create_service(Trigger, 'reset_distance', self.reset_distance)
        self.prev = None
        self.dist = 0

    def reset_distance(self, request, response):
        self.dist = 0
        return response

    def gps_callback(self, msg):
        latitude = msg.latitude
        longitude = msg.longitude
        self.gps_pub.publish(msg)
        if not math.isnan(latitude) and not math.isnan(longitude):
            delta = 0 if self.prev is None else distance((self.prev[0], self.prev[1]), (latitude, longitude))
            self.dist = float(self.dist + delta)
            self.prev = [latitude, longitude]
            new_msg = Distance()
            new_msg.header = msg.header
            new_msg.distance = self.dist*1000
            self.dist_pub.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    navfix = Remapper(args=args)
    rclpy.spin(navfix)
    rclpy.shutdown()


if __name__ == '__main__':
    main()


import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Quaternion, Pose
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import math
import message_filters
import threading
import numpy as np

from r4c_interfaces.action import Nav

def gps_to_local(origin, point):
    """
    Convert GPS coordinates to local coordinates relative to an origin point.

    Parameters:z
        origin (tuple): A tuple containing the latitude and longitude of the origin point in decimal degrees.
        point (tuple): A tuple containing the latitude and longitude of the point to be converted in decimal degrees.

    Returns:
        tuple: A tuple containing the local coordinates (x, y) of the point relative to the origin point.
    """

    # Convert decimal degrees to radians
    lat1, lon1 = math.radians(origin[0]), math.radians(origin[1])
    lat2, lon2 = math.radians(point[0]), math.radians(point[1])

    # Calculate Earth radius at origin latitude
    R = 6378137 / math.sqrt(1 - 0.006694 * math.pow(math.sin(lat1), 2))

    # Calculate local coordinates
    x = (lon2 - lon1) * R * math.cos(lat1)
    y = (lat2 - lat1) * R

    return (x, y)

def gps_to_local_array(origin, points):
    """
    Convert GPS coordinates to local coordinates relative to an origin point.

    Parameters:
        origin (tuple): A tuple containing the latitude and longitude of the origin point in decimal degrees.
        points (list): A list of tuples, where each tuple contains the latitude and longitude of a point to be converted in decimal degrees.

    Returns:
        list: A list of tuples, where each tuple contains the local coordinates (x, y) of the corresponding point relative to the origin point.
    """

    local_points = []
    for point in points:
        local_point = gps_to_local(origin, point)
        local_points.append(local_point)

    return local_points


def local_to_gps(origin, point):
    """
    Convert local coordinates relative to an origin point to GPS coordinates.

    Parameters:
        origin (tuple): A tuple containing the latitude and longitude of the origin point in decimal degrees.
        local (tuple): A tuple containing the local coordinates (x, y) of the point relative to the origin point.

    Returns:
        tuple: A tuple containing the GPS coordinates (latitude, longitude) of the point.
    """

    # Convert decimal degrees to radians
    lat1, lon1 = math.radians(origin[0]), math.radians(origin[1])

    # Calculate Earth radius at origin latitude
    R = 6378137 / math.sqrt(1 - 0.006694 * math.pow(math.sin(lat1), 2))

    # Calculate latitude and longitude
    lat2 = lat1 + (point[1] / R)
    lon2 = lon1 + (point[0] / (R * math.cos(lat1)))

    # Convert radians to decimal degrees
    lat2, lon2 = math.degrees(lat2), math.degrees(lon2)

    return (lat2, lon2)

def local_to_gps_array(origin, points):
    """
    Convert GPS coordinates to local coordinates relative to an origin point.

    Parameters:
        origin (tuple): A tuple containing the latitude and longitude of the origin point in decimal degrees.
        points (list): A list of tuples, where each tuple contains the local coordinates (x, y) of the point relative to the origin point.

    Returns:
        list: A list of tuples, where each tuple contains the GPS coordinates (latitude, longitude) of the point.
    """

    gps_points = []
    for point in points:
        gps_point = local_to_gps(origin, point)
        gps_points.append(gps_point)

    return gps_points

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
    radius_earth = 6378137
    
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

def bearing(coord1, coord2, degree=False):
    """
    Calculate the angle (in degrees) between two GPS points.

    Parameters:
    coord1 (tuple): A tuple representing the first GPS coordinate in the form (latitude, longitude).
    coord2 (tuple): A tuple representing the second GPS coordinate in the form (latitude, longitude).
    degree (bool): If True, the result will be returned in degrees. If False (default), the result will be in radians.

    Returns:
    float: The angle (in degrees or radians) between the two coordinates.
    
    Example:
    >>> bearing((52.205, 0.119), (48.857, 2.351))
    1.373405989846147

    Reference:
    This function is based on the "Haversine formula" and the "Bearing formula" from the "Navigation Formulas" section of
    the "Movable Type Scripts" website: https://www.movable-type.co.uk/scripts/latlong.html
    """
    # Convert latitude and longitude to radians
    lat1, lon1 = map(math.radians, coord1)
    lat2, lon2 = map(math.radians, coord2)
    
    # Calculate the difference in longitude
    dlon = lon2 - lon1
    
    # Calculate the angle in degrees using the arctan2 function
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    rads = math.atan2(y, x)
    angle = math.degrees(rads) if degree else rads
    
    return angle

def displace(start_coord, offset_coord):
    """
    Calculate the new latitude and longitude that are displaced by the given offsets.

    Parameters:
    start_coord (tuple): A tuple representing the starting GPS coordinate in the form (latitude, longitude).
    offset_coord (tuple): A tuple representing the offset in the form (latitude_offset, longitude_offset) in meters.

    Returns:
    tuple: A tuple representing the new latitude and longitude in the form (latitude, longitude).

    """
    # Earth's radius in meters
    radius_earth = 6378137
    
    # Convert starting latitude and longitude to radians
    lat, lon = map(math.radians, start_coord)
    
    # Calculate offset in latitude and longitude
    lat_offset, lon_offset = offset_coord
    dlat = lat_offset / radius_earth
    dlon = lon_offset / (radius_earth * math.cos(math.pi * lat / 180))
    
    # Offset position in decimal degrees
    lat_o = lat + dlat * 180/math.pi
    lon_o = lon + dlon * 180/math.pi
    
    # Convert back to degrees and return as tuple
    return math.degrees(lat_o), math.degrees(lon_o)

def rotate_points(points, theta):
    """
    Rotate a 2D vector by a given angle (in degrees).

    Parameters:
    vector (tuple or list): A tuple or list representing a 2D vector in the form (x, y).
    theta (float): The angle (in degrees) by which to rotate the vector.

    Returns:
    tuple: A tuple representing the rotated vector in the form (x, y).

    """
    # Convert vector to numpy array and transpose
    points = np.array(points).T
    
    # Convert angle to radians
    theta = np.radians(theta)
    
    # Create rotation matrix
    rotation_matrix = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])
    
    # Apply rotation and transpose back to row vector
    output = (rotation_matrix @ points).T
    
    # Return as tuple
    return tuple(output.squeeze())

def rotate_cords(coords, theta):
    """
    Rotate an array of GPS coordinates by a given angle (in degrees).

    Parameters:
    coords (numpy.ndarray): A numpy array of GPS coordinates, where each row is a coordinate in the form (latitude, longitude).
    theta (float): The angle (in degrees) by which to rotate the coordinates.

    Returns:
    numpy.ndarray: A numpy array of rotated GPS coordinates, where each row is a coordinate in the form (latitude, longitude).

    """
    # Convert angle to radians
    theta = np.radians(theta)
    
    # Create rotation matrix
    rotation_matrix = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])
    
    # Convert coordinates to radians
    coords_rad = np.radians(coords)
    
    # Convert coordinates to cartesian coordinates
    x = np.cos(coords_rad[:, 0]) * np.cos(coords_rad[:, 1])
    y = np.cos(coords_rad[:, 0]) * np.sin(coords_rad[:, 1])
    z = np.sin(coords_rad[:, 0])
    xyz = np.vstack((x, y, z)).T
    
    # Apply rotation matrix to cartesian coordinates
    xyz_rotated = (rotation_matrix @ xyz.T).T
    
    # Convert rotated cartesian coordinates back to GPS coordinates
    lat = np.arcsin(xyz_rotated[:, 2] / np.sqrt(xyz_rotated[:, 0]**2 + xyz_rotated[:, 1]**2 + xyz_rotated[:, 2]**2))
    lon = np.arctan2(xyz_rotated[:, 1], xyz_rotated[:, 0])
    
    # Convert back to degrees and return as numpy array
    return np.degrees(np.vstack((lat, lon)).T)

pose = Pose()
fix = NavSatFix()
import rclpy
from sensor_msgs.msg import NavSatFix

def callback(msg):
    print("Received message: ", msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('simple_subscriber')
    subscription = node.create_subscription(NavSatFix, '/gps/fix', callback, 10)

    subscription  # prevent unused variable warning

    while rclpy.ok():
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

class GetThePosition(Node):
    def __init__(self):
        super().__init__('go_to_position')
        self.pub_ = self.create_publisher(Pose, '/pose/local', 10)
        self.pose_sub = message_filters.Subscriber(self, Odometry, '/odometry/global')
        self.fix_sub = message_filters.Subscriber(self, NavSatFix, '/gps/fix')
        self.pose_sub = message_filters.ApproximateTimeSynchronizer([self.pose_sub, self.fix_sub], 10, slop=10)
        self.pose_sub.registerCallback(self.pose_callback)

    def pose_callback(self, pose_sub, fix_sub):
        global pose
        global fix
        pose.position = pose_sub.pose.pose.position
        pose.orientation = pose_sub.pose.pose.orientation
        fix = fix_sub
        self.pub_.publish(pose)


class GoToPosition(Node):
    def __init__(self):
        super().__init__('go_to_position')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.current_pose_ = Point()
        self.target_pose_ = Point()
        self.current_orientation_ = Quaternion()

        self.log_file = False

        self._goal_handle = None
        self._goal_lock = threading.Lock()

        self._action_server = ActionServer(self, Nav, '/navigation', 
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            callback_group=ReentrantCallbackGroup())

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def cancel_callback(self, goal):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                self._goal_handle.abort()
            self._goal_handle = goal_handle
        goal_handle.execute()
     
    async def execute_callback(self, goal_handle):
        global pose
        global fix
        self.get_logger().info('Executing goal...')
        feedback_msg = Nav.Feedback()
        points = goal_handle.request.initial_path.poses
        new_points = []
        for i in points:
            new_points.append((i.pose.position.x, i.pose.position.y))
        print(local_to_gps_array((fix.latitude, fix.longitude), new_points))
        for i in points:
            target = i.pose.position
            self.target_pose_ = target
            print(f"going to: {i}")
            while True:
                if goal_handle.is_cancel_requested:
                    self.stop_moving()
                    return Nav.Result()
                if not goal_handle.is_active:
                    self.stop_moving()
                    return Nav.Result()
                self.current_pose_ = pose.position
                self.current_orientation_ = pose.orientation
                twist = Twist()
                distance, twist.linear.x, twist.angular.z  = self.get_nav_params()
                self.publisher_.publish(twist)
                feedback_msg.longitude = fix.longitude
                feedback_msg.latitude = fix.latitude
                if self.log_file:
                    with open('example.txt', 'a') as txt: txt.write(f"{new_points}, {fix.latitude}, {fix.longitude}\n")
                goal_handle.publish_feedback(feedback_msg)
                if distance < 0.5: 
                    break
        self.stop_moving()
        goal_handle.succeed()
        result = Nav.Result()
        return result

    def stop_moving(self):
        self.get_logger().info('Stopping...')
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def start_moving(self, feedback_msg, goal_handle):
        """
        Move the robot towards the target position and orientation using the calculated navigation parameters.

        Args:
            feedback_msg (Feedback): A message containing the current longitude and latitude of the robot.
            goal_handle (ServerGoalHandle): The handle for the current navigation goal.

        Returns:
            True if the robot has not yet reached the target position and orientation, False otherwise.
        """
        # declare global variables for the robot's pose and GPS fix
        global pose
        global fix
        # initialize the robot's velocity and get the necessary navigation parameters
        twist = Twist()
        self.current_pose_ = pose.position
        self.current_orientation_ = pose.orientation
        distance, twist.linear.x, twist.angular.z  = self.get_nav_params()
        # publish the robot's velocity
        self.publisher_.publish(twist)
        # update the feedback message with the current GPS fix and publish it
        feedback_msg.longitude = fix.longitude
        feedback_msg.latitude = fix.latitude
        goal_handle.publish_feedback(feedback_msg)
        # return False if the robot has reached the target position and orientation
        if distance < 0.3: 
            return False
        # return True if the robot has not yet reached the target position and orientation
        return True

    def get_nav_params(self, angle_max=0.5, velocity_max=0.1):
        """
        Calculate the distance, velocity, and angular error needed to move from the current position and orientation to the target position and orientation.

        Args:
            angle_max (float): The maximum allowable angular error in radians. Default value is 0.35 radians (approximately 20 degrees).
            velocity_max (float): The maximum allowable velocity in m/s. Default value is 0.15 m/s.

        Returns:
            A tuple containing the distance, velocity, and angular error required to move to the target position and orientation.
            - distance (float): The Euclidean distance between the current position and the target position in meters.
            - velocity (float): The desired velocity in m/s based on the distance between the current position and the target position.
            - angular (float): The desired angular error in radians based on the difference between the current orientation and the desired orientation.
        """
        distance = math.sqrt((self.target_pose_.x - self.current_pose_.x) ** 2 + (self.target_pose_.y - self.current_pose_.y) ** 2)
        # calculate the desired velocity based on the distance to the target position
        velocity = 0.2 * distance
        # calculate the initial heading towards the target position
        preheading = math.atan2(self.target_pose_.y - self.current_pose_.y, self.target_pose_.x - self.current_pose_.x)
        # calculate the current orientation of the robot using quaternions
        orientation = yaw = math.atan2(2 * (self.current_orientation_.w * self.current_orientation_.z + self.current_orientation_.x * self.current_orientation_.y), 
                                       1 - 2 * (self.current_orientation_.y**2 + self.current_orientation_.z**2))
        # calculate the difference between the initial heading and the robot's orientation
        heading = preheading - orientation
        # correct the heading to ensure the robot turns the shortest distance towards the target
        heading_corrected = np.arctan2(np.sin(heading), np.cos(heading))
        # limit the angular error and velocity to the maximum allowable values
        angular = max(-angle_max, min(heading_corrected, angle_max))
        velocity = max(-velocity_max, min(velocity, velocity_max))
        return distance, velocity, angular



def main(args=None):
    rclpy.init(args=args)
    try:
        getpos=GetThePosition()
        gotopos = GoToPosition()
        gotopos.log_file = False
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(getpos)
        executor.add_node(gotopos)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            getpos.destroy_node()
            gotopos.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':\
    main()
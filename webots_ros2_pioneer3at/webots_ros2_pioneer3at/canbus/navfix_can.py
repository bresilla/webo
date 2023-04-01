import math
import rclpy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from rclpy.node import Node
import can
import cantools
from haversine import haversine

class NavFix2Ccan(Node):
    def __init__(self, args):
        super().__init__("navfix_can")
        self.gps_sub = self.create_subscription(NavSatFix, '/gps', self.gps_callback, 10)
        self.speed_sub = self.create_subscription(Float32, '/gps/speed', self.speed_callback, 10)
        self.prev = None
        self.dist = 0
        self.dbc = """VERSION ""
        BO_ 2365475321 GBSD: 8 Vector__XXX
         SG_ GroundBasedMachineSpeed : 0|16@1+ (0.001,0) [0|64.255] "m/s" Vector__XXX
        BO_ 2314732030 GNSSPositionRapidUpdate: 8 Bridge
         SG_ Longitude : 32|32@1- (1E-007,0) [-180|180] "deg" Vector__XXX
         SG_ Latitude : 0|32@1- (1E-007,0) [-90|90] "deg" Vector__XXX
        """
        self.canbus = None
        self.gbsd = cantools.db.load_string(self.dbc, 'dbc').get_message_by_name("GBSD")
        self.gnss = cantools.db.load_string(self.dbc, 'dbc').get_message_by_name("GNSSPositionRapidUpdate")

    def send2can(self, message):
        try:
            self.canbus.send(message)
        except can.CanError:
            print("COULD NOT SEND THE MESSAGE")
        print(message)

    def gps_callback(self, msg):
        latitude = msg.latitude
        longitude = msg.longitude
        if not math.isnan(latitude) and not math.isnan(longitude):
            self.send2can(can.Message(arbitration_id=self.gnss.frame_id, data=self.gnss.encode({'Latitude': latitude, 'Longitude': longitude})))
            self.get_logger().info('Latitude: {latitude}, Longitude: {longitude}'.format(latitude=latitude, longitude=longitude))
    
    def speed_callback(self, msg):
        speed = msg.data
        if not math.isnan(speed):
            self.send2can(can.Message(arbitration_id=self.gbsd.frame_id, data=self.gbsd.encode({'GroundBasedMachineSpeed': int(speed)})))
            self.get_logger().info('Speed: {speed}'.format(speed=speed))

def main(args=None):
    rclpy.init(args=args)
    navfix = NavFix2Ccan(args=args)
    navfix.canbus = can.interface.Bus(channel='vcan0', bustype='socketcan')
    rclpy.spin(navfix)
    rclpy.shutdown()


if __name__ == '__main__':
    main()


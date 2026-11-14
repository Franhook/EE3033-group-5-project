import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

class ScanChecker(Node):
    def __init__(self):
        super().__init__('scan_checker')
        self.create_subscription(
            LaserScan,
            '/scan_fixed',
            self.callback,
            qos_profile_sensor_data)

    def callback(self, msg):
        calculated = (
            round((msg.angle_max - msg.angle_min) /
                  msg.angle_increment) + 1
            if msg.angle_increment != 0.0 else 0
        )

        print(
            f'n={len(msg.ranges):3d}, '
            f'calculated={calculated:3d}, '
            f'min={msg.angle_min:+.6f}, '
            f'max={msg.angle_max:+.6f}, '
            f'inc={msg.angle_increment:.8f}'
        )

rclpy.init()
node = ScanChecker()

try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass

node.destroy_node()
rclpy.shutdown()

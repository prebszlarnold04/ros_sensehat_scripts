#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from sense_hat import SenseHat
import time

class DeviceStatusChecker(Node):

    def __init__(self):
        super().__init__('device_status_checker')
        self.sense = SenseHat()
        self.sense.clear()

        self.lidar_ok = False
        self.camera_ok = False

        # Feliratkozás a működő topicokra
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        self.camera_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.camera_callback,
            10
        )

        # 2 másodpercenként frissíti a kijelzőt
        self.timer = self.create_timer(2.0, self.update_display)

    def lidar_callback(self, msg):
        self.lidar_ok = True
        self.get_logger().info("LiDAR adat érkezik")

    def camera_callback(self, msg):
        self.camera_ok = True
        self.get_logger().info("Kamera adat érkezik")

    def update_display(self):
        self.sense.clear()

        if self.lidar_ok:
            # zöld L betű ha lidar aktív
            self.sense.show_letter('L', text_colour=[0, 255, 0])
        else:
            self.sense.show_letter('L', text_colour=[255, 0, 0])
        time.sleep(1)

        if self.camera_ok:
            # zöld C betű ha kamera aktív
            self.sense.show_letter('C', text_colour=[0, 255, 0])
        else:
            self.sense.show_letter('C', text_colour=[255, 0, 0])
        time.sleep(1)

        self.lidar_ok = False
        self.camera_ok = False
        self.sense.clear()

def main(args=None):
    rclpy.init(args=args)
    node = DeviceStatusChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.sense.clear()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

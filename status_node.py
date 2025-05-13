#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sense_hat import SenseHat

class SenseHatStatus(Node):
    def __init__(self):
        super().__init__('sensehat_status')
        self.sense = SenseHat()
        # Rögtön jelezzük, hogy futunk:
        self.sense.show_message(
            "ROS2 OK",
            scroll_speed=0.1,
            text_colour=[0, 255, 0]
        )
        self.get_logger().info('Sense HAT status node started, message shown.')

    def destroy(self):
        # Node leállásakor töröljük a mátrixot:
        self.sense.clear()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SenseHatStatus()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

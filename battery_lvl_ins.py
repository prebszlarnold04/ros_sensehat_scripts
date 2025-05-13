#!/usr/bin/env python3
from sense_hat import SenseHat
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('sense_hat_disp')
        self.sense = SenseHat()
        self.sense.clear()
        self.voltage = None

        # Feliratkozás a feszültség topicra
        self.create_subscription(
            Float32,
            'power_voltage',
            self.voltage_callback,
            10
        )

        # Timer a folyamatos kijelzéshez (másodpercenként egyszer)
        self.create_timer(1.0, self.update_display)

    def voltage_callback(self, msg: Float32):
        # Csak tároljuk az értéket
        self.voltage = msg.data
        self.get_logger().info(f"Voltage received: {self.voltage}")

    def update_display(self):
        # Ha még nem érkezett mérés, ne mutassunk semmit
        if self.voltage is None:
            return

        # Küszöbszint alatti feszültség esetén X, különben _
        if self.voltage < 21:
            self.sense.show_letter("X", text_colour=(255, 0, 0))
        else:
            self.sense.show_letter("O", text_colour=(255, 0, 255))

def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # CTRL+C esetén csak kilépünk a spinből
        pass
    finally:
        # Előbb mindig pusztítsd el a node-ot
        node.destroy_node()
        # Csak ha a context még nem lett lezárva:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()


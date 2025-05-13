#!/usr/bin/env python3
import os
from sense_hat import SenseHat
import rclpy
from sense_hat import SenseHat
import rclpy
from rclpy.node import Node


class CpuTempMonitor(Node):

    def __init__(self):
        super().__init__('cpu_temp_monitor')
        self.sense = SenseHat()
        self.timer = self.create_timer(5.0, self.check_temperature)  # 5 mp-enként

    def check_temperature(self):
        temp_str = os.popen("vcgencmd measure_temp").readline()
        try:
            temp = float(temp_str.replace("temp=", "").replace("'C\n", ""))
        except ValueError:
            self.get_logger().error(f"Hibás hőmérséklet: {temp_str}")
            return

        self.get_logger().info(f"CPU hőmérséklet: {temp} °C")

        color = [0, 255, 0] if temp < 70.0 else [255, 0, 0]  # zöld vagy piros
        self.sense.show_message(f"{temp:.1f}C", text_colour=color, scroll_speed=0.1)


def main(args=None):
    rclpy.init(args=args)
    node = CpuTempMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


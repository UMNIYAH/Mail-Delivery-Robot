import os
import threading
from datetime import datetime
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensors.lidar_sensor import LidarSensor

class Logger(Node):
    """
    Logger node that logs messages from multiple topics into a .txt file, 
    and tracks how long the robot is wall-following.
    To run: ros2 run mail-delivery-robot logger

    """

   #!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import BatteryState
from jinja2 import Environment, FileSystemLoader
import os
import re

class GeneralLogger(Node):
    def __init__(self):
        super().__init__('general_logger')

        self.declare_parameter('log_dir', './tools/logs')
        self.log_dir = self.get_parameter('log_dir').value
        os.makedirs(self.log_dir, exist_ok=True)

        self.log_path = os.path.join(self.log_dir, "robot_log_wallFollowing.txt")
        self.log_file = open(self.log_path, "a")
        self.write_log("SYSTEM", f"Logging all data to {self.log_path}")

        self.generate_report()

    def write_log(self, source, message):
        self.log_file.write(f"[{source}] {message}\n")
        self.log_file.flush()

    def generate_report(self):
        battery_data = {}

        def battery_callback(msg):
            battery_data['level'] = msg.percentage * 100
            battery_data['voltage'] = msg.voltage
            battery_data['temperature'] = msg.temperature

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=1
        )

        sub = self.create_subscription(BatteryState, '/battery_state', battery_callback, qos_profile)

        start_time = self.get_clock().now()
        timeout_sec = 2.0
        while 'level' not in battery_data:
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > timeout_sec:
                self.get_logger().warn("No battery message received, using default values.")
                battery_data = {'level': 0.0, 'voltage': 0.0, 'temperature': 0.0}
                break

        self.destroy_subscription(sub)

        battery_level = battery_data['level']
        voltage_level = battery_data['voltage']
        temperature_level = battery_data['temperature']

        wall_follow_time = "N/A"
        if os.path.exists(self.log_path):
            with open(self.log_path, "r") as f:
                lines = f.readlines()
            for line in reversed(lines):
                match = re.search(r"Total wall-following time:\s*([\d.]+)s", line)
                if match:
                    wall_follow_time = f"{match.group(1)} s"
                    break

       
        template_dir = os.path.dirname(os.path.realpath(__file__))
        env = Environment(loader=FileSystemLoader(template_dir))
        try:
            template = env.get_template("template.html")
        except Exception as e:
            self.get_logger().error(f"template.html not found in {template_dir}: {e}")
            return

        html_content = template.render(
            battery_level=battery_level,
            voltage_level=voltage_level,
            temperature_level=temperature_level,
            wall_follow_time=wall_follow_time
        )

        output_path = os.path.join(self.log_dir, "robot_report.html")
        with open(output_path, "w") as f:
            f.write(html_content)

        self.get_logger().info(f"Report generated at {output_path}!")

        open(self.log_path, 'w').close()
        self.get_logger().info(f"Cleared wall-following log: {self.log_path}")


def main(args=None):
    rclpy.init(args=args)
    node = GeneralLogger()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

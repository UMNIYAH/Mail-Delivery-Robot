import os
import time
import re
from datetime import datetime, timedelta

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import BatteryState
from irobot_create_msgs.msg import DockStatus
from jinja2 import Environment, FileSystemLoader


class GeneralLogger(Node):
    def __init__(self):
        super().__init__('general_logger')

        # --- Log directory ---
        self.declare_parameter('log_dir', './tools/logs')
        self.log_dir = os.path.abspath(self.get_parameter('log_dir').value)
        os.makedirs(self.log_dir, exist_ok=True)
        self.get_logger().info(f"Logs will be saved to: {self.log_dir}")

        # --- Wall-following log path ---
        self.wall_log_path = os.path.join(self.log_dir, "robot_log_wallFollowing.txt")

        # --- Trip timing ---
        self.trip_start_time = time.perf_counter()
        self.trip_start_timestamp = datetime.now()

        # --- Battery info ---
        self.battery = {'level': 0.0, 'voltage': 0.0, 'temperature': 0.0}
        self.battery_start = None
        self.battery_end = None
        self.battery_used = None

        # --- Subscriptions ---
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)
        self.create_subscription(BatteryState, '/battery_state', self.battery_callback, qos)
        self.create_subscription(DockStatus, '/dock_status', self.dock_status_callback, 10)

        self.get_logger().info("GeneralLogger started. Waiting for first battery data...")

        # Wait for first battery message
        while self.battery['level'] == 0.0 and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        self.battery_start = self.battery['level']
        self.get_logger().info(f"Battery at trip start: {self.battery_start:.2f}%")

    
    def battery_callback(self, msg):
        self.battery['level'] = msg.percentage * 100
        self.battery['voltage'] = msg.voltage
        self.battery['temperature'] = msg.temperature

    def dock_status_callback(self, msg: DockStatus):
        if msg.is_docked:
            self.get_logger().info("Docking detected, ending trip...")
            self.end_trip()
            rclpy.shutdown()

    def get_wall_follow_time(self):
        if not os.path.exists(self.wall_log_path):
            return "N/A"

        with open(self.wall_log_path, 'r') as f:
            for line in reversed(f.readlines()):
                match = re.search(r"Total wall-following time:\s*([\d.]+)s", line)
                if match:
                    return f"{match.group(1)} s"
        return "N/A"

   
    def end_trip(self):
        """Handles trip completion and generates reports."""
        self.trip_end_timestamp = datetime.now()
        delivery_time_sec = time.perf_counter() - self.trip_start_time
        delivery_time_str = str(timedelta(seconds=int(delivery_time_sec)))

        # Battery at end
        self.battery_end = self.battery['level']
        self.battery_used = self.battery_start - self.battery_end

        self.get_logger().info(f"Battery Start: {self.battery_start:.2f}% | "
                               f"End: {self.battery_end:.2f}% | "
                               f"Used: {self.battery_used:.2f}%")

        wall_time = self.get_wall_follow_time()

        # Write run file
        self.write_run_file(self.battery, wall_time, delivery_time_sec)

        # Generate HTML report
        self.generate_html_report(self.battery, wall_time, delivery_time_str)

        # Clear wall log
        open(self.wall_log_path, 'w').close()

        self.get_logger().info("Trip logging complete.")


    def write_run_file(self, battery, wall_time, delivery_time):
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filepath = os.path.join(self.log_dir, f"run_{timestamp}.txt")
        self.get_logger().info(f"Writing run file: {filepath}")

        with open(filepath, "w") as f:
            f.write(f"battery_start={self.battery_start}\n")
            f.write(f"battery_end={self.battery_end}\n")
            f.write(f"battery_used={self.battery_used}\n\n")
            f.write(f"delivery_time={delivery_time:.2f}\n")
            f.write(f"wall_follow_time={wall_time}\n")
            f.write(f"voltage_level={battery['voltage']}\n")
            f.write(f"temperature_level={battery['temperature']}\n")
            f.write(f"trip_start_time={self.trip_start_timestamp}\n")
            f.write(f"trip_end_time={self.trip_end_timestamp}\n")

    def generate_html_report(self, battery, wall_time, delivery_time_str):
        template_dir = os.path.dirname(os.path.realpath(__file__))
        env = Environment(loader=FileSystemLoader(template_dir))
        template = env.get_template("template.html")

        html_path = os.path.join(self.log_dir, "robot_report.html")
        self.get_logger().info(f"Generating HTML report: {html_path}")

        html_content = template.render(
            battery_level=battery['level'],
            voltage_level=battery['voltage'],
            temperature_level=battery['temperature'],
            wall_follow_time=wall_time,
            delivery_time=delivery_time_str,
            battery_start=self.battery_start,
            battery_end=self.battery_end,
            battery_used=self.battery_used,
            trip_start_time=self.trip_start_timestamp,
            trip_end_time=self.trip_end_timestamp
        )

        with open(html_path, "w") as f:
            f.write(html_content)


def main(args=None):
    rclpy.init(args=args)
    node = GeneralLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt detected, ending trip...")
        node.end_trip()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

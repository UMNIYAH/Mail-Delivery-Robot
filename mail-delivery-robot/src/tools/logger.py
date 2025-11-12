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

    def __init__(self):
        super().__init__('general_logger')
        self.declare_parameter('log_dir', './tools/logs')
        self.log_dir = self.get_parameter('log_dir').value
        os.makedirs(self.log_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_path = os.path.join(self.log_dir, f"robot_log_{timestamp}.txt")
        self.log_file = open(self.log_path, "a")
        self.get_logger().info(f"Logging all data to {self.log_path}")

        # Subscribe to captain actions
        self.create_subscription(String, '/actions', self.captain_callback, 10)

        # Docking status to stop logging
        self.create_subscription(String, '/docking_status', self.docking_callback, 10)

        # Wall-following tracking
        self.wall_following_start = None
        self.total_wall_following_time = 0.0

    def captain_callback(self, msg: String):
        """
        Track wall-following duration from Captain output
        """
        self.write_log("/actions", msg.data)
        # Detect if any action contains WALL_FOLLOW
        if "WALL_FOLLOW" in msg.data:
            if self.wall_following_start is None:
                self.wall_following_start = datetime.now()
        else:
            if self.wall_following_start is not None:
                elapsed = (datetime.now() - self.wall_following_start).total_seconds()
                self.total_wall_following_time += elapsed
                self.wall_following_start = None
                self.get_logger().info(f"Wall-following segment ended, duration: {elapsed:.2f}s")

    def docking_callback(self, msg: String):
        if msg.data == "DOCKED":
            # If wall-following was active, finalize time
            if self.wall_following_start is not None:
                elapsed = (datetime.now() - self.wall_following_start).total_seconds()
                self.total_wall_following_time += elapsed
            self.get_logger().info(f"Total wall-following time: {self.total_wall_following_time:.2f}s")
            self.get_logger().info("Docking complete. Shutting down logger...")
            rclpy.shutdown()

    def write_log(self, topic, message):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        line = f"[{timestamp}] [{topic}] {message}\n"
        self.log_file.write(line)
        self.log_file.flush()
        self.get_logger().info(line.strip())

    def destroy_node(self):
        self.log_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Logger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt detected. Finalizing logs...")
        if node.wall_following_start is not None:
            elapsed = (datetime.now() - node.wall_following_start).total_seconds()
            node.total_wall_following_time += elapsed
            node.get_logger().info(f"Wall-following segment ended, duration: {elapsed:.2f}s")
            node.wall_following_start = None
        node.get_logger().info(f"Total wall-following time: {node.total_wall_following_time:.2f}s")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

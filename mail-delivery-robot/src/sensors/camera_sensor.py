import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import sys
import subprocess
import numpy
import cv2

class CameraSensor(Node):
    def __init__(self):
        super().__init__('camera_sensor')

        self.camera_publisher = self.create_publisher(Bool, '/camera_data', 10)

        self.timer = self.create_timer(0.2, self.get_yellow)

        self.bool_msg = Bool()
        self.bool_msg.data = True

        self.lower_yellow = numpy.array([5, 50, 50])
        self.upper_yellow = numpy.array([45, 255, 255])

        self.image_width = "640"
        self.image_height = "480"
        self.total_pixels = int(self.image_width) * int(self.image_height)

    def get_yellow(self):
        subprocess.run(["libcamera-still", "--output", "image.jpg", "--immediate", "--width", self.image_width, "--height", self.image_height, "-n", "--verbose", "0"])
        image = cv2.imread("image.jpg")
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        yellow_mask = cv2.inRange(hsv_image, self.lower_yellow, self.upper_yellow)
        yellow_pixels = numpy.sum(yellow_mask > 0)
        yellow_percentage = yellow_pixels / self.total_pixels
        if yellow_percentage > 0.1:
            self.camera_publisher.publish(self.bool_msg)

def main():
    rclpy.init()
    camera_sensor = CameraSensor()
    rclpy.spin(camera_sensor)

if __name__ == '__main__':
    main()
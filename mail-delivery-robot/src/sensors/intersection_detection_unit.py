import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class IntersectionDetectionUnit(Node):
    '''
    The Node in charge of intersection detection.

    @Subscribers:
    - Subscribes to /camera_data for information about intersection markers.
    - Subscribes to /lidar_data for information about nearby walls

    @Publishers:
    - Publishes intersection detection data to /intersection_detection
    '''
    def __init__(self):
        '''
        The constructor for the node.
        Defines the necessary publishers and subscribers.
        '''
        super().__init__('intersection_detection_unit')

        self.lidar_data_sub = self.create_subscription(String, 'lidar_data', self.lidar_data_callback, 10)
        self.camera_data_sub = self.create_subscription(String, 'camera_data', self.camera_data_callback, 10)

        self.intersection_detection_publisher = self.create_publisher(String, 'intersection_detection', 10)
        self.intersection_detection_timer = self.create_timer(1, self.update_intersection_detection)

        self.true_msg = String()
        self.true_msg.data = 'TRUE'
        self.false_msg = String()
        self.false_msg.data = 'FALSE'
    
    def lidar_data_callback(self, data):
        '''
        The callback for /lidar_data.
        Reads information from the lidar sensor.
        '''
        pass
    
    def camera_data_callback(self, data):
        '''
        The callback for /camera_data.
        Reads information about intersection markers.
        '''
        pass

    def update_intersection_detection(self):
        '''
        The timer callback. Updates the internal state of this node and sends
        updates to /navigation when necessary
        '''
        pass

def main():
    rclpy.init()
    intersection_detection_unit = IntersectionDetectionUnit()
    rclpy.spin(intersection_detection_unit)

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool

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
        self.camera_data_sub = self.create_subscription(Bool, 'camera_data', self.camera_data_callback, 10)

        self.intersection_detection_publisher = self.create_publisher(String, 'intersection_detection', 10)
        self.intersection_detection_timer = self.create_timer(0.5, self.update_intersection_detection)

        self.true_msg = String()
        self.true_msg.data = 'TRUE'
        self.false_msg = String()
        self.false_msg.data = 'FALSE'

        self.lidar_indicates_intersection = False
        self.camera_indicates_intersection = False
    
    def lidar_data_callback(self, data):
        '''
        The callback for /lidar_data.
        Reads information from the lidar sensor.
        '''
        lidar_data = str(data.data)

        split_data = lidar_data.split(":")
        
        # waiting for the lidar to calibrate
        if split_data[0] == "-1" and split_data[1] == "-1":
            return
        
        if ((split_data[2] == "-1" and split_data[3] == "-1")
            or (split_data[2] == "-1" and split_data[4] == "-1")
            or (split_data[3] == "-1" and split_data[4] == "-1")):
            self.lidar_indicates_intersection = True
        else:
            self.lidar_indicates_intersection = False
    
    def camera_data_callback(self, data):
        '''
        The callback for /camera_data.
        Reads information about intersection markers.
        '''
        #TODO: actually do something with the camera data.
        self.camera_indicates_intersection = True

    def update_intersection_detection(self):
        '''
        The timer callback. Updates the internal state of this node and sends
        updates to /navigation when necessary
        '''
        if self.lidar_indicates_intersection and self.camera_indicates_intersection:
            self.intersection_detection_publisher.publish(self.true_msg)
        else:
            self.intersection_detection_publisher.publish(self.false_msg)

def main():
    rclpy.init()
    intersection_detection_unit = IntersectionDetectionUnit()
    rclpy.spin(intersection_detection_unit)

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from enum import Enum

class TurningLayerStates(Enum):
    '''
    An enum for the internal states of the turning layer.
    '''
    NO_DEST = 'NO_DEST'
    HAS_DEST = 'HAS_DEST'
    LEFT_TURN = 'LEFT_TURN'
    RIGHT_TURN = 'RIGHT_TURN'
    U_TURN = 'U_TURN'
    PASS = 'PASS'

class TurningLayer(Node):
    '''
    The subsumption layer responsible for moving the robot through intersections.

    @Subscribers:
    - Listens to /destinations for data about the robot's current destination
    - Listens to /beacon_data for data about nearby beacons
    - Listens to /lidar_data for data about nearby walls
    - Listens to /camera_data for data about intersection markers the robot passes over

    @Publishers:
    - Publishes actions to /actions
    '''
    def __init__(self):
        '''
        The constructor for the node.
        Defines the necessary publishers and subscribers.
        '''
        super().__init__('turning_layer')

        self.state = TurningLayerStates.NO_DEST
        self.current_destination = 'NONE'

        self.destinations_sub = self.create_subscription(String, 'destinations', self.destinations_callback, 10)
        self.beacon_data_sub = self.create_subscription(String, 'beacon_data', self.beacon_data_callback, 10)
        self.lidar_data_sub = self.create_subscription(String, 'lidar_data', self.lidar_data_callback, 10)
        self.camera_data_sub = self.create_subscription(String, 'camera_data', self.camera_data_callback, 10)
        
        self.action_publisher = self.create_publisher(String, 'actions', 10)

        self.no_msg = String()
        self.no_msg.data = '2:NONE'

        self.timer = self.create_timer(0.2, self.update_actions)

        self.action_publisher.publish(self.no_msg)

    def destinations_callback(self, data):
        '''
        The callback for /destinations.
        Reads the robot's current destination when one is published.
        '''
        self.current_destination = data.data.split(':')[1]

    def beacon_data_callback(self, data):
        '''
        The callback for /beacon_data.
        Reads information about nearby beacons.
        '''
        self.current_beacon = data.data.split(',')[0]

    def lidar_data_callback(self, data):
        '''
        The callback for /lidar_data.
        Reads information about nearby walls.
        '''
        pass

    def camera_data_callback(self, data):
        '''
        The callback for /camera_data.
        Reads information about intersection markers the robot passes over.
        '''
        pass

    def update_actions(self):
        '''
        The timer callback. Updates the internal state of this node and sends
        updates to /actions when necessary
        '''
        pass

def main():
    rclpy.init()
    turning_layer = TurningLayer()
    rclpy.spin(turning_layer)

if __name__ == '__main__':
    main()
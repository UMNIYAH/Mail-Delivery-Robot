import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from enum import Enum

class TravelLayerStates(Enum):
    '''
    An enum for the internal states of the travel layer.
    '''
    NO_DEST = 'NO_DEST'
    HAS_DEST = 'HAS_DEST'

class TravelLayer(Node):
    '''
    The subsumption layer responsible for moving the robot forward, wall following, etc.

    @Subscribers:
    - Listens to /lidar_data for data about nearby walls
    - Listens to /destinations for data about the robot's current destination

    @Publishers:
    - Publishes actions to /actions
    '''
    def __init__(self):
        '''
        The constructor for the node.
        Defines the necessary publishers and subscribers.
        '''
        super().__init__('travel_layer')

        self.state = TravelLayerStates.NO_DEST
        self.current_destination = 'NONE'

        self.lidar_data_sub = self.create_subscription(String, 'lidar_data', self.lidar_data_callback, 10)
        self.destinations_sub = self.create_subscription(String, 'destinations', self.destinations_callback, 10)
        
        self.action_publisher = self.create_publisher(String, 'actions', 10)

        self.go_msg = String()
        self.go_msg.data = '3:GO'
        self.no_msg = String()
        self.no_msg.data = '3:NONE'

        self.timer = self.create_timer(0.2, self.update_actions)

        self.action_publisher.publish(self.no_msg)

    def lidar_data_callback(self, data):
        '''
        The callback for /lidar_data.
        Reads information about nearby walls.
        '''
        pass

    def destinations_callback(self, data):
        '''
        The callback for /destinations.
        Reads the robot's current destination when one is published.
        '''
        pass

    def update_actions(self):
        '''
        The timer callback. Updates the internal state of this node and sends
        updates to /actions when necessary
        '''
        #todo: implement this beyond this testing version
        self.action_publisher.publish(self.go_msg)

def main():
    rclpy.init()
    travel_layer = TravelLayer()
    rclpy.spin(travel_layer)

if __name__ == '__main__':
    main()
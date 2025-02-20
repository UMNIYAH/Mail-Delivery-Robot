import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from enum import Enum

class AvoidanceLayerStates(Enum):
    '''
    An enum for the internal states of the travel layer.
    '''
    #todo: this

class TravelLayer(Node):
    '''
    The subsumption layer responsible for moving the robot forward, wall following, etc.

    @Subscribers:
    - Listens to /lidar_data for data about nearby walls

    @Publishers:
    - Publishes actions to /actions
    '''
    def __init__(self):
        '''
        The constructor for the node.
        Defines the necessary publishers and subscribers.
        '''
        super().__init__('travel_layer')

        self.lidar_data_sub = self.create_subscription(String, 'lidar_data', self.lidar_data_callback, 10)
        
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
        #todo: this
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
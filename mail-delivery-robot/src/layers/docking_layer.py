import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from enum import Enum

class DockingLayerStates(Enum):
    '''
    An enum for the internal states of the docking layer.
    '''
    #todo: this

class DockingLayer(Node):
    '''
    The subsumption layer responsible for moving the robot on to and off of the dock.

    @Subscribers:
    - Listens to /destinations for data about the robot's current destination
    - Listens to /beacon_data for data about nearby beacons

    @Publishers:
    - Publishes actions to /actions
    '''
    def __init__(self):
        '''
        The constructor for the node.
        Defines the necessary publishers and subscribers.
        '''
        super().__init__('docking_layer')

        self.destinations_sub = self.create_subscription(String, 'destinations', self.destinations_callback, 10)
        self.beacon_data_sub = self.create_subscription(String, 'beacon_data', self.beacon_data_callback, 10)
        
        self.action_publisher = self.create_publisher(String, 'actions', 10)

        self.timer = self.create_timer(0.2, self.update_actions)

        self.action_publisher.publish(self.no_msg)

    def destinations_callback(self, data):
        pass

    def beacon_data_callback(self, data):
        pass

    def update_actions(self):
        pass

def main():
    rclpy.init()
    docking_layer = DockingLayer()
    rclpy.spin(docking_layer)

if __name__ == '__main__':
    main()
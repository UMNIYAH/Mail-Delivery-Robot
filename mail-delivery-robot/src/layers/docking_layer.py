import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from enum import Enum
from irobot_create_msgs.msg import DockStatus
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class DockingLayerStates(Enum):
    '''
    An enum for the internal states of the docking layer.
    '''
    NO_DEST = 'NO_DEST'
    HAS_DEST = 'HAS_DEST'

class DockingLayer(Node):
    '''
    The subsumption layer responsible for moving the robot on to and off of the dock.

    @Subscribers:
    - Listens to /destinations for data about the robot's current destination
    - Listens to /beacon_data for data about nearby beacons
    - Listens to /dock_status for data about nearby docking stations and the robot's dock status

    @Publishers:
    - Publishes actions to /actions
    '''
    def __init__(self):
        '''
        The constructor for the node.
        Defines the necessary publishers and subscribers.
        '''
        super().__init__('docking_layer')

        self.state = DockingLayerStates.NO_DEST
        self.current_destination = 'NONE'
        self.current_beacon = 'NONE'
        self.dock_visible = False
        self.is_docked = False


        self.destinations_sub = self.create_subscription(String, 'destinations', self.destinations_callback, 10)
        self.beacon_data_sub = self.create_subscription(String, 'beacon_data', self.beacon_data_callback, 10)
        self.dock_status_sub = self.create_subscription(DockStatus, 'dock_status', self.dock_status_callback, qos_profile=QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        ))
        
        self.action_publisher = self.create_publisher(String, 'actions', 10)

        self.no_msg = String()
        self.no_msg.data = '1:NONE'
        self.dock_msg = String()
        self.dock_msg.data = '1:DOCK'
        self.undock_msg = String()
        self.undock_msg.data = '1:UNDOCK'

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

    def dock_status_callback(self, data):
        '''
        The callback for /dock_status.
        Reads information about nearby docks, and the robot's current dock status.
        '''
        self.dock_visible = data.dock_visible
        self.is_docked = data.is_docked

    def update_actions(self):
        '''
        The timer callback. Updates the internal state of this node and sends
        updates to /actions when necessary
        '''
        if self.state == DockingLayerStates.NO_DEST and self.current_destination != 'NONE':
            self.state = DockingLayerStates.HAS_DEST
            if self.is_docked:
                self.action_publisher.publish(self.undock_msg)
        elif self.state == DockingLayerStates.HAS_DEST and self.current_beacon == self.current_destination:
            if self.dock_visible:
                self.state = DockingLayerStates.NO_DEST
                self.current_destination = 'NONE'
                self.current_beacon = 'NONE'
                self.action_publisher.publish(self.dock_msg)
        else:
            self.action_publisher.publish(self.no_msg)
        

def main():
    rclpy.init()
    docking_layer = DockingLayer()
    rclpy.spin(docking_layer)

if __name__ == '__main__':
    main()
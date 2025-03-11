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
    - Listens to /navigation for the next action the robot should take at an intersection
    - Listens to /intersection_detection for data about whether or not the robot is in an intersection

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

        self.navigation_sub = self.create_subscription(String, 'navigation', self.navigation_callback, 10)
        self.intersection_detection_sub = self.create_subscription(String, 'intersection_detection', self.intersection_detection_callback, 10)
        
        self.action_publisher = self.create_publisher(String, 'actions', 10)

        self.no_msg = String()
        self.no_msg.data = '2:NONE'

        self.timer = self.create_timer(0.2, self.update_actions)

        self.action_publisher.publish(self.no_msg)

    def navigation_callback(self, data):
        '''
        The callback for /navigation.
        Reads information about navigation actions the robot should take.
        '''
        pass

    def intersection_detection_callback(self, data):
        '''
        The callback for /intersection_detection.
        Reads information about whether the robot is currently in an intersection.
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
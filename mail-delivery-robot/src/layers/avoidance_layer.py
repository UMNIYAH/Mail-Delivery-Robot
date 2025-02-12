import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AvoidanceLayer(Node):
    '''
    The subsumption layer responsible for obstacle avoidance.

    @Subscribers:
    - Listens to /bumper_data for collision detection

    @Publishers:
    - Publishes actions to /actions
    '''
    def __init__(self):
        '''
        The constructor for the node.
        Defines the necessary publishers and subscribers.
        '''
        super().__init__('avoidance_layer')

        self.bumper_data_sub = self.create_subscription(String, 'bumper_data', self.bumper_data_callback)
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class NavigationUnit(Node):
    '''
    The Node in charge of navigation.

    @Subscribers:
    - Subscribes to /beacon_data for information about where the robot is.

    @Publishers:
    - Publishes navigation data to /navigation
    '''
    def __init__(self):
        '''
        The constructor for the node.
        Defines the necessary publishers and subscribers.
        '''
        super().__init__('navigation_unit')

        self.destinations_sub = self.create_subscription(String, 'destinations', self.destinations_callback, 10)
        self.beacon_data_sub = self.create_subscription(String, 'beacon_data', self.beacon_data_callback, 10)

        self.navigation_publisher = self.create_publisher(String, 'navigation', 10)
        self.navigation_timer = self.create_timer(1, self.update_navigation)

        self.current_destination = None
        self.current_beacon = None

        self.left_msg = String()
        self.left_msg.data = 'LEFT_TURN'
        self.right_msg = String()
        self.right_msg.data = 'RIGHT_TURN'
        self.straight_msg = String()
        self.straight_msg.data = 'STRAIGHT'
        self.uturn_msg = String()
        self.uturn_msg.data = 'U_TURN'
        self.dock_msg = String()
        self.dock_msg.data = 'DOCK'
        self.undock_msg = String()
        self.undock_msg.data = 'UNDOCK'
    
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

    def update_navigation(self):
        '''
        The timer callback. Updates the internal state of this node and sends
        updates to /navigation when necessary
        '''
        pass

def main():
    rclpy.init()
    navigation_unit = NavigationUnit()
    rclpy.spin(navigation_unit)

if __name__ == '__main__':
    main()
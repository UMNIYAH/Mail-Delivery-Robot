from geometry_msgs.msg import Twist

def translate_action(action):
    msg = Twist()
    match action:
        case 'GO':
            msg.linear.x = 0.1
        case 'WAIT':
            msg.linear.x = 0.0
        case 'BACK':
            msg.linear.x = -1.0
        case 'LEFT_TURN':
            msg.angular.z = 1.0
        case 'RIGHT_TURN':
            msg.angular.z = -1.0
    
    return msg
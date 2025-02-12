from geometry_msgs.msg import Twist

def translate_action(action):
    msg = Twist()
    match action:
        case 'GO':
            msg.linear.x = 1.0
        case 'WAIT':
            msg.linear.x = 0.0
    
    return msg
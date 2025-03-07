from geometry_msgs.msg import Twist

def translate_action(action):
    '''
    Translates an action message into a Twist command.
    This translator performs no computation; it simply maps the provided
    movement orders to the appropriate Twist fields.
    
    Expected action formats:
      - "GO"         : Standard forward motion.
      - "WAIT"       : Stop moving.
      - "BACK"       : Reverse.
      - "LEFT_TURN"  : Turn left.
      - "RIGHT_TURN" : Turn right.
      - "WALL_FOLLOW:<linear_speed>:<angular_speed>" : Use the provided speeds.
    
    Returns:
      A Twist message with the movement order.
    '''
    msg = Twist()
    
    match action:
        case 'GO':
            msg.linear.x = 0.2
        case 'WAIT':
            msg.linear.x = 0.0
        case 'BACK':
            msg.linear.x = -1.0
        case 'LEFT_TURN':
            msg.angular.z = 1.0
        case 'RIGHT_TURN':
            msg.angular.z = -1.0
        case _:
            try:
                parts = action.split(",")
                # Expecting format: 3:WALL_FOLLOW:<linear_speed>:<angular_speed>
                msg.linear.x = float(parts[1])
                msg.angular.z = float(parts[2])
            except (IndexError, ValueError):
                # If parsing fails, set speeds to zero.
                msg.linear.x = 0.0
                msg.angular.z = 0.0
    return msg


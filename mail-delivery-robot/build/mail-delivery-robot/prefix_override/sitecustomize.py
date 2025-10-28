import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/capstone2526/cmdr_ws/src/carleton-mail-delivery-robot/mail-delivery-robot/install/mail-delivery-robot'

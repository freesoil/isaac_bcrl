import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/beable/BCRL/isaac_bcrl/install/joystick_controller'

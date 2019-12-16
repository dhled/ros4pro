from intera_interface.gripper import Gripper as RealGripper
from rospy import get_param

class Gripper(object):
    # This is an abstraction for simulated and real electric grippers
    def __init__(self):
        self._gripper = None if get_param("ros4pro/light_simulation", False) else RealGripper()

    def open(self):
        if self._gripper:
            return self._gripper.open()
        else:
            pass
     
    def close(self):
        if self._gripper:
            return self._gripper.close()
        else:
            pass
    
    def is_gripping(self):
        if self._gripper:
            return self._gripper.is_gripping()
        else:
            return True
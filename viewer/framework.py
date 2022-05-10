import pyphysx as px
import numpy as np

class FrameworkBase:
    """
    The base of the main testbed framework.

    If you are planning on using the testbed framework and:
    * Want to implement your own renderer (other than Pygame, etc.):
      You should derive your class from this one to implement your own tests.
      See empty.py or any of the other tests for more information.
    * Do NOT want to implement your own renderer:
      You should derive your class from Framework. The renderer chosen in
      fwSettings (see settings.py) or on the command line will automatically
      be used for your test.
    """
    name = "None"
    description = None
    TEXTLINE_START = 30
    colors = {
        'mouse_point': (0, 1, 0),
        'bomb_center': (0, 0, 1.0),
        'bomb_line': (0, 1.0, 1.0),
        'joint_line': (0.8, 0.8, 0.8),
        'contact_add': (0.3, 0.95, 0.3),
        'contact_persist': (0.3, 0.3, 0.95),
        'contact_normal': (0.4, 0.9, 0.4),
    }

    def __reset(self):
        """ Reset all of the variables to their starting values.
        Not to be called except at initialization."""
        

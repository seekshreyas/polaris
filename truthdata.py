""" I realize globals are hated but since this is the truth data from X-Plane and it makes everything easier, I'm just going to do it. In radians as always.
"""
class TruthData(object):
    """
    This is a library for doing a comprehensive, realtime display of all the states within the EKF.
    """
    
    # Hacks for singleton functionality (is there a better way to do this?)
    _instance = None
    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super(TruthData, cls).__new__(
                                cls, *args, **kwargs)
        return cls._instance

    def __init__(self):
        self.DT = 0.05
        self.P = 0.0
        self.Q = 0.0
        self.R = 0.0
        self.AX = 0.0
        self.AY = 0.0
        self.AZ = 0.0
        self.BX = 0.0
        self.BY = 0.0
        self.BZ = 0.0
        self.PITCH = 0.0
        self.ROLL = 0.0
        self.HEADING = 0.0
        self.ALTITUDE = 0.0
        self.LATITUDE = 0.0
        self.LONGITUDE = 0.0
        self.AIRSPEED = 0.0
        self.SPEEDOVERGROUND = 0.0
        self.COURSEOVERGROUND = 0.0

from math import radians, sin, sqrt, atan2, cos
from time import time
from utils import safe_tangent, wrap
from truthdata import TruthData

TD = TruthData()

class EmulatedXplaneGPS:
    ''' Emulates the behavior of GPS via X-Plane truth data
    '''
    
    def __init__(self, delay=1.1, Hz=2.0):
        self.delay = delay
        self.update_dt = 1.0/Hz
        self.last_update = time() + delay
        self.speed_over_ground = 0
        self.latitude = 0
        self.longitude = 0
        self.altitude = 0
        self.last_latitude = 0
        self.last_longitude = 0
        self.sog = []
        self.lat = []
        self.lon = []
        self.alt = []

    def update(self, dt):
        self.sog.append(TD.SPEEDOVERGROUND)
        self.lat.append(TD.LATITUDE)
        self.lon.append(TD.LONGITUDE)
        self.alt.append(TD.ALTITUDE)
        now = time()
        if now - self.last_update > self.update_dt:
            self.last_update = now
            self.speed_over_ground = self.sog.pop()
            self.last_latitude = self.latitude
            self.last_longitude = self.longitude
            self.latitude = self.lat.pop()
            self.longitude = self.lon.pop()
            self.altitude = self.alt.pop()
            self.course_over_ground = gps_coords_to_heading(self.last_latitude, self.last_longitude, self.latitude, self.longitude)
        return { "speed_over_ground": self.speed_over_ground,
                 "course_over_ground": self.course_over_ground,
                 "latitude": self.latitude,
                 "longitude": self.longitude,
                 "altitude": self.altitude }

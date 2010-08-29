from math import radians, sin, sqrt, atan2, cos, pi
from time import time
from utils import safe_tangent, wrap
from truthdata import TruthData
from geonavigation import heading
from display import Display

TD = TruthData()
display = Display()

class EmulatedXplaneGPS:
    ''' Emulates the behavior of GPS via X-Plane truth data
    '''
    
    def __init__(self, delay=1.1, Hz=2.0):
        self.delay = delay
        self.update_dt = 1.0/Hz
        self.last_update = time() + delay
        self.speed_over_ground = 0.0
        self.course_over_ground = 0.0
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.last_latitude = 0.0
        self.last_longitude = 0.0
        self.home_latitude = 0.0
        self.home_longitude = 0.0
        self.cog = []
        self.sog = []
        self.lat = []
        self.lon = []
        self.alt = []

    def relative_gps(self):
        lat = self.home_latitude - TD.LATITUDE
        lon = self.home_longitude - TD.LONGITUDE
        lamb = heading(self.home_latitude,self.home_longitude, TD.LATITUDE, TD.LONGITUDE)
        d = self.dist_to_wpt(self.home_latitude,self.home_longitude, TD.LATITUDE, TD.LONGITUDE)
        lamb = radians(lamb)
        display.register_scalars({"lambda":lamb,"d": d,}, "Dead reckoning")
        Pn = cos(lamb)*d
        Pe = sin(lamb)*d
        return (Pn, Pe)

    def dist_to_wpt(self,lat1,lon1,lat2,lon2):
        # calculates great route circle in between waypoints
        R = 6371 # earth's radius
        #distance calc
        delta_lat = (lat2 - lat1)#(pi/180);
        delta_lon = (lon2 - lon1)#*(pi/180);
        a = sin((delta_lat/2)*(pi/180))*sin((delta_lat/2)*(pi/180)) + cos(lat1*(pi/180))*cos(lat2*(pi/180)) * sin((delta_lon/2)*(pi/180))*sin((delta_lon/2)*(pi/180))
        c = 2 * atan2(sqrt(a),sqrt(1-a))
        d = R * c #Km
        #d = d * 3280.8399 #ft
        return d*1000 #meters



    def update(self, dt):
        self.cog.append(TD.COURSEOVERGROUND)
        self.sog.append(TD.SPEEDOVERGROUND)
        self.lat.append(TD.LATITUDE)
        self.lon.append(TD.LONGITUDE)
        self.alt.append(TD.ALTITUDE)
        if self.home_latitude == 0.0 and self.home_longitude == 0.0:
            self.home_latitude = TD.LATITUDE
            self.home_longitude = TD.LONGITUDE
        now = time()
        if now - self.last_update > self.update_dt:
            self.last_update = now
            self.speed_over_ground = self.sog.pop()
            self.last_latitude = self.latitude
            self.last_longitude = self.longitude
            self.latitude = self.lat.pop()
            self.longitude = self.lon.pop()
            self.altitude = self.alt.pop()
            self.course_over_ground = self.cog.pop()
        return { "speed_over_ground": self.speed_over_ground,
                 "course_over_ground": self.course_over_ground,
                 "latitude": self.latitude,
                 "longitude": self.longitude,
                 "altitude": self.altitude }

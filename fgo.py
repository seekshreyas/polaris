from math import radians, sin, sqrt, atan2, cos, degrees
from utils import safe_tangent, wrap
from display import Display
from truthdata import TruthData

display = Display()
TD = TruthData()

''' FGO = Fixed-Gain Observers
'''

class AltitudeObserver:
    ''' http://diydrones.com/profiles/blogs/no-baro-altitude-filter-for#comments
    '''

    def __init__(self):
        self.altitude = 0.0

    def estimate(self, theta, Vair, GPS_altitude, dt):
        k = -0.01
        alpha = radians(0.52) # average angle of attack
        gamma = theta - alpha
        self.altitude += Vair * sin(gamma) * dt
        error = self.altitude - GPS_altitude
        self.altitude += k * error
        return self.altitude

class WindObserver:
    ''' Ryan Beall is awesome.
    '''
    def __init__(self):
        self.k = 0.05
        self.Wn_fgo = 0.0
        self.We_fgo = 0.0

    def estimate(self, theta, psi, Vair, ground_speed, course_over_ground, dt):
        ''' Units:
                radians: theta, psi, course_over_ground
                meters per second: Vair, ground_speed
                seconds: dt
            Issues to fix:
                Speeds need to come in as the right units for this estimator to work properly
        '''
        # if they come in as knots
        #Vair = Vair / 0.5144444444
        #ground_speed = ground_speed / 0.5144444444
        display.register_scalars({"Vair":Vair, "GSPD":ground_speed, "TD.Wdir":TD.WIND_DIRECTION, "TD.Wvel":TD.WIND_VELOCITY}, "Wind FGO")
        wind_north_error = (Vair * cos(psi) * cos(theta)) - (ground_speed * cos(course_over_ground)) - self.Wn_fgo
        wind_east_error = (Vair * sin(psi) * cos(theta)) - (ground_speed * sin(course_over_ground)) - self.We_fgo
        self.Wn_fgo += self.k * wind_north_error
        self.We_fgo += self.k * wind_east_error
        display.register_scalars({"CoG":course_over_ground,"psi":psi,"Wn error": wind_north_error, "We error": wind_east_error, "Wn_fgo": self.Wn_fgo, "We_fgo": self.We_fgo, "cos(psi)": cos(psi), "cos(theta)": cos(theta), "sin(psi)": sin(psi), "cos(cog)": cos(course_over_ground), "sin(cog)": sin(course_over_ground), "wind_north_error": wind_north_error, "wind_east_error": wind_east_error}, "Wind FGO")
        wind_direction = degrees(atan2(self.We_fgo,self.Wn_fgo))
        wind_velocity = sqrt(self.We_fgo**2 + self.Wn_fgo**2) / 0.5144444444 # convert from m/s to knots
        return wind_direction, wind_velocity

from math import radians, sin, sqrt, atan2, cos
from utils import safe_tangent, wrap
from display import Display

display = Display()

''' FGO = Fixed-Gain Observers
'''

class AltitudeObserver:
    ''' http://diydrones.com/profiles/blogs/no-baro-altitude-filter-for#comments
    '''

    def estimate(self, theta, Vair, GPS_altitude, dt):
        k = -0.01
        alpha = radians(0.52) # average angle of attack
        gamma = theta - alpha
        self.estimate += Vair * sin(gamma) * dt
        error = estimate - GPS_altitude
        self.estimate += k * error
        return self.estimate

class WindObserver:
    ''' Ryan Beall is awesome.
    '''

    def estimate(self, theta, psi, Vair, ground_speed, course_over_ground, dt):
        k = 0.01
        wind_north_error = Vair * cos(psi) * cos(theta) - ground_speed * cos(course_over_ground) - self.Wn_fgo;
        wind_east_error = Vair * sin(psi) * cos(theta) - ground_speed * sin(course_over_ground) - self.We_fgo;

        self.Wn_fgo = Wn_fgo + k * wind_north_error;
        self.We_fgo = We_fgo + k * wind_east_error;

        wind_direction = atan2(self.We_fgo,self.Wn_fgo)*(180/pi);
        wind_velocity = sqrt(self.We_fgo^2 + self.Wn_fgo^2);

        return wind_direction, wind_velocity

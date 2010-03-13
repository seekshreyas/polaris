from math import cos, sin, atan2, pi, degrees
from numpy import matrix
import logging
import logging.config
from display import Display
from utils import safe_tangent, wrap

logging.config.fileConfig("logging.conf")
logger = logging.getLogger("shumai")

display = Display()

''' VGO = Variable-Gain Observers
'''

class HeadingObserver:
    """
    Since X-Plane doesn't given out the magnetic field components, this will
    have to do until I can finish getting it working (and tested) with a
    real magnetometer.
    """

    def __init__(self):
        self.k_psi = -0.08
        self.dt = 0.05
        self.prevent_division_by_zero = 0.00000000001
        self.psi_estimate = 0
        self.psi = 0
        self.psihat = 0
        self.A = 0
        self.I = 1
        self.P = 1
        self.C = 1
        self.Q = 0.1
        self.L = 0
        self.r = 0
        self.mxyz = matrix("25396.8; 2011.7; 38921.5") # magnetic field strength for Corpus Christi in nanotesla units

    def magnetometer_readings_to_tilt_compensated_heading(self, bx, by, bz, phi, theta):
        self.variation = 4.528986*(pi/180)
        Xh = bx * cos(theta) + by * sin(phi) * sin(theta) + bz * cos(phi) * sin(theta)
        Yh = by * cos(phi) - bz * sin(phi)
        heading = wrap((atan2(-Yh, Xh) + self.variation))
        if heading < 0:
            heading += 2*pi
        return heading

    def estimate_heading(self, bx, by, bz, phi, theta, q, r, dt):
        self.dt = dt
        self.phi, self.theta = phi, theta
        psi_measured = self.magnetometer_readings_to_tilt_compensated_heading(bx, by, bz, phi, theta)
        psi_dot = (q * sin(phi) / cos(theta)) + (r * cos(phi) / cos(theta)) # old ((q * sin(phi)) + (r * cos(phi) / cos(theta)))
        self.psi_estimate += (psi_dot * dt)
        psi_error = self.psi_estimate - psi_measured
        self.psi_estimate += self.k_psi * psi_error
        logger.info("Heading %f" % self.psi_estimate)
        display.register_scalars({"Psi":degrees(self.psi_estimate)})
        return self.psi_estimate

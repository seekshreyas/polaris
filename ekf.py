from math import cos, sin, tan, atan, atan2, isnan, pi, degrees, radians, sqrt
from numpy import matrix
from datetime import datetime
import logging
import logging.config
from display import Display
from utils import safe_tangent, wrap, GRAVITY
import logging
import logging.config
from display import Display

display = Display()

logging.config.fileConfig("logging.conf")
logger = logging.getLogger("shumai")

''' EKF = Extended Kalman Filter
'''

class AttitudeObserver:
    """
    This class is stage one and estimates roll and pitch only. Stages two and three will be released in the near future.
    
    States/outputs
    x = [phi, theta]
    
    Inputs
    u = [p, q, r, Vair]
    z = [ax, ay, az]

    Variables representing external states:
    xhat: estimated state vector [phihat, thetahat]
    f(xhat, u): non-linear state update [phi,theta]
    h(xhat, u): non-linear model outputs [p,q,r]
    A: linearized state update matrix [2x2]
    C: linearized model output matrix 3x[2x1]
    Q: process noise covariance matrix [2x2] How much we trust the gyros and accelerometers
    R: sensor noise covariance matrix [3x1] (what about Vair and ax, ay and az?)
    """

    def __init__(self):
        self.A = matrix("0 0; 0 0")
        self.I = matrix("1 0; 0 1")
        self.Px = matrix("1 0; 0 1")
        self.Py = matrix("1 0; 0 1")
        self.Pz = matrix("1 0; 0 1")
        self.Q = matrix("0.1 0; 0 0.1")
        self.Cx = matrix("0 0")
        self.Cy = matrix("0 0")
        self.Cz = matrix("0 0")
        self.Lx = matrix("0; 0")
        self.Ly = matrix("0; 0")
        self.Lz = matrix("0; 0")
        self.acceleration_magnitude = 0
        self.acceleration_weight = 0
        self.p, self.q, self.r = 0, 0, 0
        self.ax, self.ay, self.az = 0, 0, 0
        self.axhat, self.ayhat, self.azhat = 0, 0, 0
        self.rx, self.ry, self.rz = 0, 0, 0
        self.rx0, self.ry0, self.rz0 = 8, 8, 8 # 0.05, 2, 1
        self.tau = 55
        self.dt = 0.05
        self.phi, self.theta = 0, 0
        self.phihat, self.thetahat = 0, 0
        self.psi_estimate = 0
        self.last_time = datetime.now()
        self.roll_is_departed = False
        self.pitch_is_departed = False

    def update_state_estimate_using_kinematic_update(self, p, q, r, dt):
        """ When we get new gyro readings we can update the estimate of pitch and roll with the new values. [ISEFMAV 2.20] """
        self.phihat = self.phi + ((p + (q * sin(self.phi) * safe_tangent(self.theta)) + (r * cos(self.phi) * safe_tangent(self.theta))) * dt)
        self.thetahat = self.theta + (((q * cos(self.phi)) - (r * sin(self.phi))) * dt)
        logger.debug("kinematic update, phihat = %f, thetahat = %f" % (self.phihat, self.thetahat))

    def compute_linearized_state_update_matrix(self, q, r):
        """ Once we've updated the state estimates for phi and theta we need to linearize it again. [ISEFMAV 2.21] """
        self.A[0,0] = (q * cos(self.phihat) * safe_tangent(self.thetahat)) - (r * sin(self.phihat) * safe_tangent(self.thetahat))
        self.A[0,1] = ((q * sin(self.phihat)) - (r * cos(self.phihat))) / pow(cos(self.thetahat), 2)
        self.A[1,0] = (-q * sin(self.phihat)) + (r * cos(self.phihat))
        # self.A[1,1] = 0

    def propagate_covariance_matrix(self, dt):
        """ With the newly updated linearization of the state estimates, we can update the covariance matrix that stores how confident we are with something. Email me if I haven't replaced something with a better choice of words. [ISEFMAV 2.12] """
        self.Px += ((self.A*self.Px + self.Px*self.A.transpose() + self.Q) * dt)
        self.Py += ((self.A*self.Py + self.Py*self.A.transpose() + self.Q) * dt)
        self.Pz += ((self.A*self.Pz + self.Pz*self.A.transpose() + self.Q) * dt)

    def linearized_model_output_matrix(self, p, q, r, Vair):
        """ Axhat, ayhat and azhat are used to compute the the expected accelerometer readings given the model (the flight dynamics of a fixed-wing aircraft). Once we have these we can look at the actual readings and adjust things accordingly. [ISEFMAV 2.26] """
        self.axhat = ((Vair * q * sin(self.thetahat))/GRAVITY) + sin(self.thetahat)
        self.ayhat = ((Vair * ((r * cos(self.thetahat)) - (p * sin(self.thetahat))))/GRAVITY) - (cos(self.thetahat) * sin(self.phihat))
        self.azhat = ((-Vair * q * cos(self.thetahat))/GRAVITY) - (cos(self.thetahat) * cos(self.phihat))

    def gain_calculation_and_variance_update(self, p, q, r, Vair):
        """ Calculate linearized output equations...this is the magic of the Kalman filter; here we are figuring out how much we trust the sensors [ISEFMAV 2.27] """
        self.Cx = matrix([[0, ((q * Vair / GRAVITY) * cos(self.thetahat)) + cos(self.thetahat)]])
        self.Cy = matrix([[-cos(self.thetahat) * cos(self.phihat), ((-r * Vair / GRAVITY) * sin(self.thetahat)) - ((p * Vair / GRAVITY) * cos(self.thetahat)) + (sin(self.thetahat) * sin(self.phihat))]])
        self.Cz = matrix([[cos(self.thetahat) * cos(self.phihat), (((q * Vair / GRAVITY) * sin(self.thetahat)) + cos(self.phihat)) * sin(self.thetahat)]])

    def get_sensor_noise_covariance_matrix(self, q):
        """ Sensor noise penalty, needs a better explanation of how it works. Tau is a tuning parameter which is increased to reduce the Kalman gain on x-axis accelerometer during high pitch rate maneuvers (that flight dynamic is unmodeled). I'm trying values between 10-100. [ISEFMAV 2.28] """
        self.rx = self.rx0 + (self.tau * abs(q))
        self.ry = self.ry0
        self.rz = self.rz0

    def calc_kalman_gain_matrix(self):
        """ Basically to smooth out noisy sensor measurements we calculate a gain which is small if we want to mostly ignore things during times of high-noise. This is accomplished by large values of rxyz.
        This is a simplified way to calculate the Kalman gains which basically means we're correcting the sensor measurements (and independently which is the simplification by assuming they are uncorrelated). [ISEFMAV 2.17] """
        self.Lx = (self.Px * self.Cx.transpose()) / (self.rx + (self.Cx * self.Px * self.Cx.transpose()))
        self.Ly = (self.Py * self.Cy.transpose()) / (self.ry + (self.Cy * self.Py * self.Cy.transpose()))
        self.Lz = (self.Pz * self.Cz.transpose()) / (self.rz + (self.Cz * self.Pz * self.Cz.transpose()))
        self.acceleration_magnitude = sqrt(self.ax**2 + self.ay**2 + self.az**2) / GRAVITY
        self.acceleration_weight = min(max(0, (1 - 2 * abs(1 - self.acceleration_magnitude))), 1)
        self.Lx *= self.acceleration_weight
        self.Ly *= self.acceleration_weight
        self.Lz *= self.acceleration_weight

    def update_state_estimate(self, ax, ay, az):
        """ This is where we update the roll and pitch with our final estimate given all the adjustments we've calculated for the modeled and measured states. [ISEFMAV 2.16] """
        self.phi = self.phihat + self.Lx[0,0] * (ax - self.axhat) + self.Ly[0,0] * (ay - self.ayhat) + self.Lz[0,0] * (az - self.azhat)
        self.theta = self.thetahat + self.Lx[1,0] * (ax - self.axhat) + self.Ly[1,0] * (ay - self.ayhat) + self.Lz[1,0] * (az - self.azhat)
        self.check_for_divergence()
        logger.info("updated state estimate, phi = %f, theta = %f" % (self.phi, self.theta))

    def check_for_divergence(self):
        """ Divergence is when the EKF has departed from reality is is confused. I log it so you can see when the EKF goes into a broken state. You can try resetting it and it may come back (no guarantees but it's been known to work). """
        if abs(degrees(self.phi)) > 90:
            if self.roll_is_departed == False:
                self.roll_is_departed = True
                logger.critical("Roll has departed.")
        else:
            self.roll_is_departed = False
        if abs(degrees(self.theta)) > 90:
            if self.pitch_is_departed == False:
                self.pitch_is_departed = True
                logger.critical("Pitch has departed.")
        else:
            self.pitch_is_departed = False

    def update_covariance_matrix(self):
        """ Here we update how much we trust the state estimates given the linearized model output and the Kalman gains. [ISEFMAV 2.15] """
        self.Px = (self.I - self.Lx * self.Cx) * self.Px
        self.Py = (self.I - self.Ly * self.Cy) * self.Py
        self.Pz = (self.I - self.Lz * self.Cz) * self.Pz

    def register_states(self):
        scalars = {"p (rad)":self.p,
                   "q (rad)":self.q,
                   "r (rad)":self.r,
                   "ax":self.ax,
                   "ay":self.ay,
                   "az":self.az,
                   "Acc mag":self.acceleration_magnitude,
                   "Acc wght":self.acceleration_weight,
                   "axhat":self.axhat,
                   "ayhat":self.ayhat,
                   "azhat":self.azhat,
                   "Phi (deg)":degrees(self.phi),
                   "Theta (deg)":degrees(self.theta),}
        matrices = {"A":self.A,
                    "Q":self.Q,
                    "Px":self.Px,
                    "Py":self.Py,
                    "Pz":self.Pz,
                    "Cx":self.Cx,
                    "Cy":self.Cy,
                    "Cz":self.Cz,
                    "Lx":self.Lx,
                    "Ly":self.Ly,
                    "Lz":self.Lz,}
        display.register_scalars(scalars)
        display.register_matrices(matrices)

    def estimate_roll_and_pitch(self, p, q, r, Vair, ax, ay, az, dt):
        self.p, self.q, self.r = p, q, r
        # Time update -- we can run this as often as we want (I think).
        self.update_state_estimate_using_kinematic_update(p, q, r, dt)
        self.compute_linearized_state_update_matrix(q, r)
        self.propagate_covariance_matrix(dt)
        # Measurement update -- we want to run this every time new measurements are available.
        self.linearized_model_output_matrix(p, q, r, Vair)
        self.gain_calculation_and_variance_update(p, q, r, Vair)
        self.get_sensor_noise_covariance_matrix(q)
        self.calc_kalman_gain_matrix()
        self.update_state_estimate(ax, ay, az)
        self.update_covariance_matrix()
        logger.info("roll = %f, pitch = %f" % (degrees(self.phi), degrees(self.theta)))
        self.register_states()
        return self.phi, self.theta

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

    def update_state_estimate_using_kinematic_update(self, phi, theta, q, r, dt):
        self.psihat = (q * sin(phi) / cos(theta)) + (r * cos(phi) / cos(theta))
        logger.debug("kinematic update, psihat = %f" % self.psihat)

    def propagate_covariance_matrix(self, dt):
        """ With the newly updated linearization of the state estimates, we can update the covariance matrix that stores how confident we are with something. Email me if I haven't replaced something with a better choice of words. [ISEFMAV 2.12] """
        self.P += ((self.A*self.P + self.P*self.A + self.Q) * dt)

    def linearized_model_output_matrix(self, phi, theta):
        """ Axhat, ayhat and azhat are used to compute the the expected accelerometer readings given the model (the flight dynamics of a fixed-wing aircraft). Once we have these we can look at the actual readings and adjust things accordingly. [ISEFMAV 2.26] """
        bxyz = matrix("0 0 0; 0 0 0; 0 0 0")
        self.psi = wrap(self.psi)
        bxyz[0,0] = cos(theta) * cos(self.psi)
        bxyz[0,1] = cos(theta) * sin(self.psi)
        bxyz[0,2] = -sin(theta)
        bxyz[1,0] = (sin(phi) * sin(theta) * cos(self.psi)) - (cos(phi) * sin(self.psi))
        bxyz[1,1] = (sin(phi) *sin(theta) * sin(self.psi)) + (cos(phi) * cos(self.psi))
        bxyz[1,2] = sin(phi) * cos(theta)
        bxyz[2,0] = (cos(phi) * sin(theta) * cos(self.psi)) + (sin(phi) * sin(self.psi))
        bxyz[2,1] = (cos(phi) * sin(theta) * sin(self.psi)) - (sin(phi) * cos(self.psi))
        bxyz[2,2] = cos(phi) * cos(theta)
        b = bxyz * self.mxyz
        self.bxhat = b[0,0]
        self.byhat = b[1,0]
        self.bzhat = b[2,0]

    def gain_calculation_and_variance_update(self, phi, theta):
        """ Calculate linearized output equations...this is the magic of the Kalman filter; here we are figuring out how much we trust the sensors [ISEFMAV 2.27] """
        self.Cx = (-cos(theta) * sin(self.psihat) * self.bx) + (cos(theta) * cos(self.psihat) * self.by)
        self.Cy = (((-sin(phi) * sin(theta) * sin(self.psihat)) - (cos(phi) * cos(self.psihat))) * self.bx) + (((sin(phi) * sin(theta) * sin(self.psihat)) - (cos(phi) * sin(self.psihat))) * self.by)
        self.Cz = (((-cos(phi) * sin(theta) * sin(self.psihat)) - (sin(phi) * cos(self.psihat))) * self.bx) + (((sin(phi) * sin(theta) * cos(self.psihat)) - (cos(phi) * sin(self.psihat))) * self.by)

    def calc_kalman_gain_matrix(self):
        """ Basically to smooth out noisy sensor measurements we calculate a gain which is small if we want to mostly ignore things during times of high-noise. This is accomplished by large values of rxyz.
        This is a simplified way to calculate the Kalman gains which basically means we're correcting the sensor measurements (and independently which is the simplification by assuming they are uncorrelated). [ISEFMAV 2.17] """
        self.L = (self.P * self.C) / (self.r + (self.C * self.P * self.C))

    def update_state_estimate(self, bx, by, bz):
        """ This is where we update the roll and pitch with our final estimate given all the adjustments we've calculated for the modeled and measured states. [ISEFMAV 2.16] """
        self.psi = self.psihat + self.L * (bx - self.bxhat) + self.L * (by - self.byhat) + self.L * (bz - self.bzhat)
        logger.info("updated state estimate, psi = %f" % self.psi)

    def update_covariance_matrix(self):
        """ Here we update how much we trust the state estimates given the linearized model output and the Kalman gains. [ISEFMAV 2.15] """
        self.P = (self.I - self.L * self.C) * self.P

    def full_kalman_estimate_heading(self, bx, by, bz, phi, theta, q, r, dt):
        self.bx, self.by, self.bz = bx, by, bz
        self.update_state_estimate_using_kinematic_update(phi, theta, q, r, dt)
        #self.compute_linearized_state_update_matrix() # not needed? A is always zero(s)
        self.propagate_covariance_matrix(dt)
        # Measurement update -- we want to run this every time new measurements are available.
        self.linearized_model_output_matrix(phi, theta)
        self.gain_calculation_and_variance_update(phi, theta)
        #self.get_sensor_noise_covariance_matrix(q)
        self.calc_kalman_gain_matrix()
        self.update_state_estimate(bx, by, bz)
        self.update_covariance_matrix()
        display.register_scalars({"Psi":180-degrees(self.psi)})
        return self.psi

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

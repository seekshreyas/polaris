#!/usr/bin/env python
# encoding: utf-8

import socket
import sys
from math import cos, sin, tan, atan, isnan, pi, degrees, radians, sqrt
from struct import unpack_from
from datetime import datetime
import logging
import logging.config
import numpy
from twisted.internet.protocol import DatagramProtocol
from twisted.internet import reactor
import csv
from display import Display

logging.config.fileConfig("logging.conf")
logger = logging.getLogger("shumai")

display = Display()

LEVELS = {'debug': logging.DEBUG,
          'info': logging.INFO,
          'warning': logging.WARNING,
          'error': logging.ERROR,
          'critical': logging.CRITICAL}

FOUT = csv.writer(open('data.csv', 'w'), delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)

UDP_IP="127.0.0.1"
UDP_PORT=49045 # was 49503
UDP_SENDTO_PORT=49501

GRAVITY = 9.80665

"""
Please read the documentation on http://dronedynamics.com/shumai-the-extended-kalman-filter/

Everything is based on http://contentdm.lib.byu.edu/ETD/image/etd1527.pdf which I will reference from here on out as ISEFMAV

Handy things to know:
- I use a capital variable letter to represent a matrix and a lowercase to represent a single value
- An "overdot" (dot above a variable) indicates a derivative taken with respect to time, e.g. xdot is equivalent to dx/dt. It is spoken as "x dot" [http://mathworld.wolfram.com/Overdot.html]
- A "hat" (a "^" above a variable) is used to give special meaning. It is commonly used to denote a unit vector or an estimator [http://mathworld.wolfram.com/Hat.html]
- A matrix using the variable "I" usually refers to the identity matrix, which is an all-zero matrix with ones down the main diagonal [http://en.wikipedia.org/wiki/Identity_matrix]
- Remember almost everything is in radians.
"""

def safe_tangent(x):
    """ This is my awful attempt at preventing NaN from returning. """
    tangent = tan(x)
    if isnan(tangent):
        logger.error("Tangent departed, input x = %f" % x)
        if tangent % (pi / 2) > 0:
            tangent = tan(x+0.000000001)
        else:
            tangent = tan(x-0.000000001)
    return tangent

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
        self.A = numpy.mat("0 0; 0 0")
        self.I = numpy.mat("1 0; 0 1")
        self.Px = numpy.mat("1 0; 0 1")
        self.Py = numpy.mat("1 0; 0 1")
        self.Pz = numpy.mat("1 0; 0 1")
        self.Q = numpy.mat("0.1 0; 0 0.1")
        self.Cx = numpy.mat("0 0")
        self.Cy = numpy.mat("0 0")
        self.Cz = numpy.mat("0 0")
        self.Lx = numpy.mat("0; 0")
        self.Ly = numpy.mat("0; 0")
        self.Lz = numpy.mat("0; 0")
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
        self.Cx = numpy.matrix([[0, ((q * Vair / GRAVITY) * cos(self.thetahat)) + cos(self.thetahat)]])
        self.Cy = numpy.matrix([[-cos(self.thetahat) * cos(self.phihat), ((-r * Vair / GRAVITY) * sin(self.thetahat)) - ((p * Vair / GRAVITY) * cos(self.thetahat)) + (sin(self.thetahat) * sin(self.phihat))]])
        self.Cz = numpy.matrix([[cos(self.thetahat) * cos(self.phihat), (((q * Vair / GRAVITY) * sin(self.thetahat)) + cos(self.phihat)) * sin(self.thetahat)]])

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

class Shumai:
    """
    Shumai [pronounced "Shoe-my"] is a six-degree-of-freedom extended Kalman filter for inertial navigation, and *requires* a 3-axis gyro, 3-axis accelerometer, a 3-axis magnetometer, an airspeed sensor, an altimeter and GPS. It is based on the work in this paper: http://contentdm.lib.byu.edu/ETD/image/etd1527.pdf
    """

    def __init__(self, imu, differential_pressure_sensor, static_pressure_sensor, magnetometer, gps):
        self.phi, self.theta, self.psi = 0, 0, 0
        self.Vair = 0
        self.attitude_observer = AttitudeObserver()
        # Comming soon:
        # self.heading_observer = HeadingObserver()
        # self.navigation_observer = NavigationObserver()
        self.imu = imu
        self.differential_pressure_sensor = differential_pressure_sensor
        self.static_pressure_sensor = static_pressure_sensor
        self.magnetometer = magnetometer
        self.gps = gps
        self.dt = 0.05
        self.last_time = datetime.now()

    def loop(self):
        self.dt = (datetime.now()-self.last_time).microseconds/1000000.0
        self.last_time = datetime.now()
        p, q, r = self.imu.read_gyros()
        ax, ay, az = self.imu.read_accelerometers()
        Vair = self.differential_pressure_sensor.read_airspeed() * 0.5144444444 # kias to meters per second
        phi, theta = self.attitude_observer.estimate_roll_and_pitch(p, q, r, Vair, ax, ay, az, self.dt)
        return {
            "roll": degrees(phi),
            "pitch": degrees(theta),
            # Coming soon:
            # "yaw": degrees(psi),
            # "airspeed": Vair,
            # "position": (position_north, position_east),
            # "wind": (wind_north, wind_east),
        }

class XplaneListener(DatagramProtocol):

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP,49998))
        self.dt = 0.05
        self.last_time = datetime.now()
        self.ekf = Shumai(self, self, self, self, self) # hack for X-Plane
        self.bxyz = numpy.mat("0.0 0.0 0.0; 0.0 0.0 0.0; 0.0 0.0 0.0")
        self.mxyz = numpy.mat("25396.8; 2011.7; 38921.5") # in nanotesla units

    def datagramReceived(self, data, (host, port)):
        """
        When we receive a UDP packet from X-Plane we'll need to unpack the data.
        I should probably document this more. Bug me until I do so...
        """
        fmt = '<f'
        self.Vair = unpack_from(fmt, data, 9)[0]
        self.az = 0 - unpack_from(fmt, data, 9+16+36)[0]
        self.ax = unpack_from(fmt, data, 9+20+36)[0]
        self.ay = unpack_from(fmt, data, 9+24+36)[0]
        self.q = radians(unpack_from(fmt, data, 9+108+0)[0])
        self.p = radians(unpack_from(fmt, data, 9+108+4)[0])
        self.r = radians(unpack_from(fmt, data, 9+108+8)[0])
        self.pitch = radians(unpack_from(fmt, data, 9+144+0)[0])
        self.roll = radians(unpack_from(fmt, data, 9+144+4)[0])
        self.heading = radians(unpack_from(fmt, data, 9+144+8)[0])
        phi, theta, psi = self.roll, self.pitch, self.heading
        self.bxyz[0,0] = cos(theta) * cos(psi)
        self.bxyz[0,1] = cos(theta) * sin(psi)
        self.bxyz[0,2] = -sin(theta)
        self.bxyz[1,0] = (sin(phi) * sin(theta) * cos(psi)) - (cos(phi) * sin(psi))
        self.bxyz[1,1] = (sin(phi) *sin(theta) * sin(psi)) + (cos(phi) * cos(psi))
        self.bxyz[1,2] = sin(phi) * cos(theta)
        self.bxyz[2,0] = (cos(phi) * sin(theta) * cos(psi)) + (sin(phi) * sin(psi))
        self.bxyz[2,1] = (cos(phi) * sin(theta) * sin(psi)) - (sin(phi) * cos(psi))
        self.bxyz[2,2] = cos(phi) * cos(theta)
        b = self.bxyz * self.mxyz
        display.register_matrices({"b":b,"bxyz":self.bxyz,"mxyz":self.mxyz})
        self.bx = b[0,0]/10000 # conversion from nanotesla to gauss
        self.by = b[1,0]/10000 # conversion from nanotesla to gauss
        self.bz = b[2,0]/10000 # conversion from nanotesla to gauss
        emulated_magnetometer = self.gauss_to_heading(self.bx, self.by, self.bz)
        display.register_scalars({"bx":self.bx,"by":self.by,"bz":self.bz,"mag heading":emulated_magnetometer})
        logger.debug("Vair %0.1f, accelerometers (%0.2f, %0.2f, %0.2f), gyros (%0.2f, %0.2f, %0.2f)" % (self.Vair, self.ax, self.ay, self.az, self.p, self.q, self.r))
        current_state = self.ekf.loop()
        if display.curses_available is True:
            display.draw()
        else:
            sys.stdout.write("%sRoll = %f, pitch = %f      " % (chr(13), current_state['roll'], current_state['pitch']))
            sys.stdout.flush()
        FOUT.writerow([degrees(self.roll), degrees(self.pitch), current_state['roll'], current_state['pitch'], current_state['roll'] - degrees(self.roll), current_state['pitch'] - degrees(self.pitch)])

    def gauss_to_heading(self, x, y, z):
        heading = 0
        if x == 0 and y < 0:
            heading = PI/2.0
        if x == 0 and y > 0:
            heading = 3.0 * pi / 2.0
        if x < 0:
            heading = pi - atan(y/x)
        if x > 0 and y < 0:
            heading = -atan(y/x)
        if x > 0 and y > 0:
            heading = 2.0 * pi - atan(y/x)
        return degrees(heading)

    def read_gyros(self):
        return self.p, self.q, self.r

    def read_accelerometers(self):
        return self.ax, self.ay, self.az

    def read_airspeed(self):
        return self.Vair

class XplaneIMU():

    def __init__(self):
        """
        We need to setup the conections, send a setup packet to X-Plane and then start listening.
        """
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP,49999))
        self.send_data_selection_packet()
        self.listener = XplaneListener()

    def run(self):
        reactor.listenUDP(UDP_PORT, self.listener)
        reactor.run()

    def send_data_selection_packet(self):
        """
        This will send a packet to X-Plane to select the data you need to read.
        Once it's sent X-Plane will output data automatically at a default of 20Hz.
        In this string, "\x03\x00\x00\x00", we are selecting the third checkbox in the
        "Settings" > "Data Input and Output" menu item ("speeds" in this example).
        The default rate is 20Hz but you can change it if you want.
        """
        data_selection_packet = "DSEL0" # this is the xplane packet type
        data_selection_packet += "\x03\x00\x00\x00" # airspeed
        data_selection_packet += "\x04\x00\x00\x00" # accelerometers
        data_selection_packet += "\x06\x00\x00\x00" # temperature
        data_selection_packet += "\x11\x00\x00\x00" # gyros
        data_selection_packet += "\x12\x00\x00\x00" # pitch and roll (for sanity check)
        data_selection_packet += "\x14\x00\x00\x00" # altimeter and GPS
        self.sock.sendto(data_selection_packet,(UDP_IP,UDP_SENDTO_PORT))

if __name__ == "__main__":
    try:
        if len(sys.argv) > 1:
            level_name = sys.argv[1]
            level = LEVELS.get(level_name, logging.NOTSET)
            logger.level=level
        xplane_imu = XplaneIMU()
        xplane_imu.run()
    except Exception, e:
        del display
        print e
        exit()

import sys
import socket
from twisted.internet.protocol import DatagramProtocol
from twisted.internet import reactor
from struct import unpack_from
from math import cos, sin, tan, atan, atan2, isnan, pi, degrees, radians, sqrt
from numpy import matrix
from datetime import datetime
import logging
import logging.config
import csv
from display import Display
from ekf import AttitudeObserver
from vgo import HeadingObserver
from fgo import AltitudeObserver
from utils import get_ip_address, wrap

logging.config.fileConfig("logging.conf")
logger = logging.getLogger("shumai")

display = Display()

LEVELS = {'debug': logging.DEBUG,
          'info': logging.INFO,
          'warning': logging.WARNING,
          'error': logging.ERROR,
          'critical': logging.CRITICAL}

FOUT = csv.writer(open('data.csv', 'w'), delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)

UDP_PORT=49045 # was 49503
UDP_SENDTO_PORT=49501

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

class Shumai:
    """
    Shumai [pronounced "Shoe-my"] is a six-degree-of-freedom extended Kalman filter for inertial navigation, and *requires* a 3-axis gyro, 3-axis accelerometer, a 3-axis magnetometer, an airspeed sensor, an altimeter and GPS. It is based on the work in this paper: http://contentdm.lib.byu.edu/ETD/image/etd1527.pdf
    States tracked (or to be tracked):
        * Yaw, pitch, roll
        * Airspeed, ground speed
        * Altitude
        * Position
        * Wind vector
        * Battery state
        * What else?
    """

    def __init__(self, imu, differential_pressure_sensor, static_pressure_sensor, magnetometer, gps):
        self.phi, self.theta, self.psi = 0, 0, 0
        self.Vair = 0
        self.attitude_observer = AttitudeObserver()
        self.heading_observer = HeadingObserver()
        # Comming soon:
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
        bx, by, bz = self.magnetometer.read_magnetometer()
        psi = self.heading_observer.estimate_heading(bx, by, bz, phi, theta, q, r, self.dt)
        return {
            "roll": degrees(phi),
            "pitch": degrees(theta),
            "yaw": degrees(psi),
            # Coming soon:
            # "airspeed": Vair,
            # "position": (position_north, position_east),
            # "wind": (wind_north, wind_east),
        }

class XplaneListener(DatagramProtocol):

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((get_ip_address(),49998))
        self.dt = 0.05
        self.last_time = datetime.now()
        self.ekf = Shumai(self, self, self, self, self) # hack for X-Plane
        self.bxyz = matrix("0.0 0.0 0.0; 0.0 0.0 0.0; 0.0 0.0 0.0")
        self.mxyz = matrix("25396.8; 2011.7; 38921.5") # some sort of magnetic field strength table in nanotesla units

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
        self.latitude = unpack_from(fmt, data, 9+180+0)[0]
        self.longitude = unpack_from(fmt, data, 9+180+4)[0]
        self.speed_over_ground = unpack_from(fmt, data, 9+12)[0]
        display.register_scalars({"lat":self.latitude,"lon":self.longitude,"sog":self.speed_over_ground})
        self.generate_virtual_magnetometer_readings(self.roll, self.pitch, self.heading)
        display.register_scalars({"bx":self.bx,"by":self.by,"bz":self.bz,"true heading":degrees(self.heading)})
        logger.debug("Vair %0.1f, accelerometers (%0.2f, %0.2f, %0.2f), gyros (%0.2f, %0.2f, %0.2f)" % (self.Vair, self.ax, self.ay, self.az, self.p, self.q, self.r))
        current_state = self.ekf.loop()
        display.register_scalars({"Psi error":current_state['yaw']-degrees(self.heading)})
        if display.curses_available is True:
            display.draw()
        else:
            sys.stdout.write("%sRoll = %f, pitch = %f      " % (chr(13), current_state['roll'], current_state['pitch']))
            sys.stdout.flush()
        FOUT.writerow([degrees(self.roll), degrees(self.pitch), current_state['roll'], current_state['pitch'], current_state['roll'] - degrees(self.roll), current_state['pitch'] - degrees(self.pitch)])

    def generate_virtual_magnetometer_readings(self, phi, theta, psi):
        psi = wrap(psi)
        self.bxyz[0,0] = cos(theta) * cos(psi)
        self.bxyz[0,1] = cos(theta) * sin(psi)
        self.bxyz[0,2] = -sin(theta)
        self.bxyz[1,0] = (sin(phi) * sin(theta) * cos(psi)) - (cos(phi) * sin(psi))
        self.bxyz[1,1] = (sin(phi) *sin(theta) * sin(psi)) + (cos(phi) * cos(psi))
        self.bxyz[1,2] = sin(phi) * cos(theta)
        self.bxyz[2,0] = (cos(phi) * sin(theta) * cos(psi)) + (sin(phi) * sin(psi))
        self.bxyz[2,1] = (cos(phi) * sin(theta) * sin(psi)) - (sin(phi) * cos(psi))
        self.bxyz[2,2] = cos(phi) * cos(theta)
        # self.bxyz[0,0] = cos(theta) * cos(psi)
        # self.bxyz[0,1] = (sin(phi) * sin(theta) * cos(psi)) - (cos(phi) * sin(psi))
        # self.bxyz[0,2] = (cos(phi) * sin(theta) * cos(psi)) + (sin(phi) * sin(psi))
        # self.bxyz[1,0] = cos(theta) * sin(psi)
        # self.bxyz[1,1] = (sin(phi) *sin(theta) * sin(psi)) + (cos(phi) * cos(psi))
        # self.bxyz[1,2] = (cos(phi) * sin(theta) * sin(psi)) - (sin(phi) * cos(psi))
        # self.bxyz[2,0] = -sin(theta)
        # self.bxyz[2,1] = sin(phi) * cos(theta)
        # self.bxyz[2,2] = cos(phi) * cos(theta)
        display.register_matrices({"bxyz":self.bxyz})
        b = self.bxyz * self.mxyz
        self.bx = b[0,0]
        self.by = b[1,0]
        self.bz = b[2,0]

    def read_gyros(self):
        return self.p, self.q, self.r

    def read_accelerometers(self):
        return self.ax, self.ay, self.az

    def read_airspeed(self):
        return self.Vair

    def read_magnetometer(self):
        return self.bx, self.by, self.bz

class XplaneIMU():

    def __init__(self):
        """
        We need to setup the conections, send a setup packet to X-Plane and then start listening.
        """
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((get_ip_address(),49999))
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
        self.sock.sendto(data_selection_packet,(get_ip_address(),UDP_SENDTO_PORT))

if __name__ == "__main__":
    try:
        if len(sys.argv) > 1:
            level_name = sys.argv[1]
            level = LEVELS.get(level_name, logging.NOTSET)
            logger.level=level
        xplane_imu = XplaneIMU()
        xplane_imu.run()
    except Exception, e:
        del xplane_imu, display
        print e
        exit()

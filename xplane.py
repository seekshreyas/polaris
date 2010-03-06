import sys
import socket
from twisted.internet.protocol import DatagramProtocol
from twisted.internet import reactor
from struct import unpack_from
from math import cos, sin, tan, atan, isnan, pi, degrees, radians, sqrt
from numpy import matrix

UDP_IP="127.0.0.1"
UDP_PORT=49045 # was 49503
UDP_SENDTO_PORT=49501

"""Just an example of how to read data from X-Plane"""

class XplaneListener(DatagramProtocol):

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP,49998))
        self.bxyz = matrix("0.0 0.0 0.0; 0.0 0.0 0.0; 0.0 0.0 0.0")
        self.mxyz = matrix("25396.8; 2011.7; 38921.5") # some sort of magnetic field strength table in
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
        self.generate_virtual_magnetometer_readings(self.roll, self.pitch, self.heading)
        emulated_magnetometer = self.gauss_to_heading(self.bx, self.by, self.bz)
        sys.stdout.write("%sVair %0.1f, accelerometers (%0.2f, %0.2f, %0.2f), gyros (%0.2f, %0.2f, %0.2f)       " % (chr(13), self.Vair, self.ax, self.ay, self.az, self.p, self.q, self.r))
        sys.stdout.flush()

    def generate_virtual_magnetometer_readings(self, phi, theta, psi):
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
        self.bx = b[0,0]/10000 # conversion from nanotesla to gauss
        self.by = b[1,0]/10000 # conversion from nanotesla to gauss
        self.bz = b[2,0]/10000 # conversion from nanotesla to gauss

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
        xplane_imu = XplaneIMU()
        xplane_imu.run()
    except Exception, e:
        print e
        exit()

from math import tan, pi, isnan
import socket
import logging
import logging.config

logging.config.fileConfig("logging.conf")
logger = logging.getLogger("shumai")

GRAVITY = 9.80665

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

def wrap(angle):
    if angle > pi:
        angle -= (2*pi)
    if angle < -pi:
        angle += (2*pi)
    return angle

def get_ip_address():
    return [ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith("127.")][0]

from truthdata import TruthData
from math import degrees, radians, atan
from display import Display
from utils import GRAVITY

TD = TruthData()
display = Display()

class Autopilot:

    def __init__(self):
        self.pitch_cmd = 90.0
        self.roll_cmd = 0.0
        self.hdg_cmd = 235.0
        self.xplane_gain = 1 / 45.0

    def condition_heading(self, heading):
        heading_error = self.hdg_cmd - heading
        if heading > 180: # never goes <180 from GPS
            heading = heading - 360
            heading_error = self.hdg_cmd - heading
        if self.hdg_cmd - heading > 180:
            heading_error = (self.hdg_cmd - heading) - 360;
        if self.hdg_cmd - heading < -180:
            heading_error = (self.hdg_cmd - heading) + 360;
        display.register_scalars({"heading_error": heading_error,})
        heading_error = min(30, max(-30, heading_error))
        return heading_error

    def heading_hold(self):
        kp_psi = 0.75
        trim_airspeed = 45.0
        kp_deweight = 0.0
        hdg_deweight = (trim_airspeed / TD.SPEEDOVERGROUND) * kp_deweight
        hdg_deweight = min(1.6, max(0.6, hdg_deweight))
        kp_psi_dot = 0.333
        psi_dot_cmd = self.condition_heading(degrees(TD.HEADING)) * kp_psi_dot
        psi_dot_cmd = min(6, max(-6, psi_dot_cmd))
        psi_dot_cmd = radians(psi_dot_cmd)
        self.roll_cmd = atan(psi_dot_cmd * (TD.SPEEDOVERGROUND * 1.68780986) / 32.2)
        self.roll_cmd = min(30, max(-30, degrees(self.roll_cmd)))
        display.register_scalars({"roll_cmd": self.roll_cmd,"psi_dot_cmd": degrees(psi_dot_cmd),})
        # self.roll_cmd = kp_psi * hdg_deweight * self.condition_heading(degrees(TD.HEADING))
        return self.roll_cmd

    def pitch_hold(self):
        kp_theta = 1.0
        self.ele_cmd = kp_theta * (self.pitch_cmd - degrees(TD.PITCH)) * self.xplane_gain
        return self.ele_cmd

    def roll_hold(self):
        kp_phi = 1.0
        self.ail_cmd = kp_phi * (self.roll_cmd - degrees(TD.ROLL)) * self.xplane_gain
        display.register_scalars({"ail_cmd": self.ail_cmd,})
        return self.ail_cmd

    def throttle(self):
        return 1.0

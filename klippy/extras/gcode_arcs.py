# adds support fro ARC commands via G2/G3
#
# Copyright (C) 2019  Aleksej Vasiljkovic <achmed21@gmail.com>
#
# function planArc() originates from https://github.com/MarlinFirmware/Marlin
# Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from math import floor, fabs, hypot, sqrt, pi, sin, cos, atan2

# Coordinates created by this are converted into G1 commands.
#
# note: only IJ version available

class ArcSupport:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.tolerance = config.getfloat('tolerance', 0.0125, above=0.0)
        self.circumscribed = config.getfloat('circumscribed', 0, minval=0.0, maxval=1.0)
        self._update()

        self.gcode_move = self.printer.load_object(config, 'gcode_move')
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command("G2", self.cmd_G2)
        self.gcode.register_command("G3", self.cmd_G2)
        self.gcode.register_command("SET_GCODE_ARCS", self.cmd_SET_GCODE_ARCS)
    def _update(self):
        x = self.circumscribed
        x2 = x*x
        # This number multiplied by the square root of the radius is the first order
        # aproximation of the number of segments required for a full circle
        self.segments_factor = sqrt((x2*(4*x - 3) + 1) / (24*self.tolerance))
        # This number multiplied by the radius and the squared apex angle is the
        # aproximation of the difference betwwen circumscribing circle and the radius
        self.radius_add_frac = (1-x2) / 8
    def cmd_SET_GCODE_ARCS(self, gcmd):
        self.tolerance = tolerance = gcmd.get_float('TOLERANCE', self.tolerance, above=0.0)
        self.circumscribed = circumscribed = gcmd.get_float('CIRCUMSCRIBED', self.circumscribed,  minval=0.0, maxval=1.0)
        self._update()
        gcmd.respond_info(f'{tolerance=} {circumscribed=}', log=False)
    def cmd_G2(self, gcmd):
        gcodestatus = self.gcode_move.get_status()
        if not gcodestatus['absolute_coordinates']:
            raise gcmd.error("G2/G3 does not support relative move mode")
        curX, curY, curZ, curE = gcodestatus['gcode_position']

        # Parse parameters
        asX = gcmd.get_float("X", curX)
        asY = gcmd.get_float("Y", curY)
        asZ = gcmd.get_float("Z", curZ)
        if gcmd.get_float("R", None) is not None:
            raise gcmd.error("G2/G3 does not support R moves")
        asI = gcmd.get_float("I", 0.)
        asJ = gcmd.get_float("J", 0.)
        if not asI and not asJ:
            raise gcmd.error("G2/G3 neither I nor J given")
        asE = gcmd.get_float("E", None)
        asF = gcmd.get_float("F", None)
        clockwise = (gcmd.get_command() == 'G2')

        # Build list of linear coordinates to move to
        coords, radius = self.planArc(curX, curY, curZ, curE, asX, asY, asZ, asI, asJ,
                              clockwise)
        e_per_move = e_base = 0.
        if asE is not None:
            if gcodestatus['absolute_extrude']:
                e_base = curE
            e_per_move = (asE - e_base) / len(coords)

        # Convert coords into G1 commands
        toolhead = self.printer.lookup_object('toolhead')
        create_gcode_command = self.gcode.create_gcode_command
        cmd_G1 = self.gcode_move.cmd_G1
        for cX, cY, cZ in coords:
            g1_params = {'X': cX, 'Y': cY, 'Z': cZ}
            if e_per_move:
                g1_params['E'] = e_base + e_per_move
                if gcodestatus['absolute_extrude']:
                    e_base += e_per_move
            if asF is not None:
                g1_params['F'] = asF
            cmd_G1(create_gcode_command("G1", "G1", g1_params))
            toolhead.current_arc_radius = radius
        toolhead.current_arc_radius = None

    # function planArc() originates from marlin plan_arc()
    # https://github.com/MarlinFirmware/Marlin
    #
    # The arc is approximated by generating many small linear segments.
    # The distance betwen chords and the arc will be within `tolerance`
    def planArc(self, curX, curY, curZ, curE, asX, asY, asZ, asI, asJ, clockwise):
        # todo: sometimes produces full circles
        tolerance = self.tolerance

        # Radius vector from center to current location
        r_P = -asI
        r_Q = -asJ

        # Determine angular travel
        center_P = curX - r_P
        center_Q = curY - r_Q
        rt_X = asX - center_P
        rt_Y = asY - center_Q
        cos_angle = r_P * rt_Y - r_Q * rt_X
        if (cos_angle != 0.
                or curX != asX
                or curY != asY):

            angular_travel = atan2(cos_angle,
                                r_P * rt_X + r_Q * rt_Y)
            if angular_travel < 0.:
                angular_travel += 2. * pi
            if clockwise:
                angular_travel -= 2. * pi
        else:
            # Make a circle if the angular rotation is 0 and the
            # target is current position
            angular_travel = 2. * pi

        # Determine number of segments
        radius = hypot(r_P, r_Q)
        segments = floor(fabs(angular_travel) * sqrt(radius) * self.segments_factor)
        linear_travel = asZ - curZ
        if segments <= 1:
            return [[asX, asY, asZ]], None

        # Generate coordinates
        theta_per_segment = angular_travel / segments
        linear_per_segment = linear_travel / segments

        radius_frac = 1 + theta_per_segment*theta_per_segment*self.radius_add_frac
        asI *= radius_frac
        asJ *= radius_frac
        #print(f"{radius=} {angular_travel=} {segments=} {theta_per_segment=} rf={theta_per_segment*theta_per_segment*self.radius_add_frac}")

        coords = []
        for i in range(1, int(segments)):
            dist_Z = i * linear_per_segment
            angle = i * theta_per_segment
            cos_Ti = cos(angle)
            sin_Ti = sin(angle)
            r_P = -asI * cos_Ti + asJ * sin_Ti
            r_Q = -asI * sin_Ti - asJ * cos_Ti

            c = [center_P + r_P, center_Q + r_Q, curZ + dist_Z]
            coords.append(c)

        coords.append([asX, asY, asZ])
        return coords, radius

def load_config(config):
    return ArcSupport(config)

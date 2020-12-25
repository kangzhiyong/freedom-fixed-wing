
# -*- coding: utf-8 -*-
import numpy as np
PI = 3.14159
class LongitudinalAutoPilot(object):
    def __init__(self):
        self.max_throttle_rpm = 2500
        self.max_elevator = 30.0*PI/180.0
        
        self.min_throttle = 0.0
        self.max_throttle = 1.0
        self.min_pitch_cmd = -10*np.pi/180.0
        self.max_pitch_cmd = 30.0*np.pi/180.0
        self.max_pitch_cmd2 = 45.0*np.pi/180.0
        
        self.speed_int = 0.0
        self.alt_int = 0.0
        self.climb_speed_int = 0.0
        
        # Pitch attitude hold
        self.k_p_theta = 0.0
        self.k_d_theta = 0.0
        # Altitude hold
        self.k_p_z = 0.0
        self.k_i_z = 0.0
        self.i_z_t = 0.0
        # Airspeed hold using commanded pitch
        self.i_v_t2 = 0.0
        self.k_p_v2 = 0.0
        self.k_i_v2 = 0.0
        # Airspeed hold using commanded throttle
        self.i_v_t = 0.0
        self.k_p_v = 0.0
        self.k_i_v = 0.0
        self.deta_star_t = 0.67  # feed-forward value for the thrust based on trim calculations
        
        self.altitude_switch = 25
        return
    
    
    
    """Used to calculate the elevator command required to acheive the target
    pitch
    
        Args:
            pitch: in radians
            pitch_rate: in radians/sec
            pitch_cmd: in radians
        
        Returns:
            elevator_cmd: in percentage elevator [-1,1]
    """
    def pitch_loop(self, pitch, pitch_rate, pitch_cmd):
        elevator_cmd = 0.0
        # STUDENT CODE HERE
        elevator_cmd = self.k_p_theta * (pitch_cmd - pitch) + self.k_d_theta * pitch_rate
        return elevator_cmd
    
    """Used to calculate the pitch command required to maintain the commanded
    altitude
    
        Args:
            altitude: in meters (positive up)
            altitude_cmd: in meters (positive up)
            dt: timestep in seconds
        
        Returns:
            pitch_cmd: in radians
    """
    def altitude_loop(self, altitude, altitude_cmd, dt):
        pitch_cmd = 0.0
        # STUDENT CODE HERE
        Ezt = altitude_cmd - altitude
        self.i_z_t = self.i_z_t + Ezt * dt
        pitch_cmd_unsat = self.k_p_z * Ezt + self.k_i_z * self.i_z_t

        if (pitch_cmd_unsat > self.max_pitch_cmd):
            pitch_cmd = self.max_pitch_cmd
        elif (pitch_cmd_unsat < self.min_pitch_cmd):
            pitch_cmd = self.min_pitch_cmd
        else:
            pitch_cmd = pitch_cmd_unsat

        # Integrator anti-windup
        if (self.k_i_z != 0):
            self.i_z_t = self.i_z_t + dt/self.k_i_z * (pitch_cmd - pitch_cmd_unsat)
        return pitch_cmd    

    """Used to calculate the throttle command required command the target 
    airspeed
        
        Args:
            airspeed: in meters/sec
            airspeed_cmd: in meters/sec
            dt: timestep in seconds
        
        Returns:
            throttle_command: in percent throttle [0,1]
    """
    def airspeed_loop(self, airspeed, airspeed_cmd, dt):        
        throttle_cmd = 0.0
        # STUDENT CODE HERE
        Evt = airspeed_cmd - airspeed
        self.i_v_t = self.i_v_t + Evt * dt
        throttle_cmd_unstat = self.deta_star_t + self.k_p_v * Evt + self.k_i_v * self.i_v_t

        # Anti windup
        if (throttle_cmd_unstat > self.max_throttle):
            throttle_cmd = self.max_throttle
        elif (throttle_cmd_unstat < self.min_throttle):
            throttle_cmd = self.min_throttle
        else:
            throttle_cmd = throttle_cmd_unstat

        if (self.k_i_v != 0):
            self.i_v_t = self.i_v_t + dt/self.k_i_v * (throttle_cmd - throttle_cmd_unstat)
        return throttle_cmd

    """Used to calculate the pitch command required to maintain the commanded
    airspeed
    
        Args:
            airspeed: in meters/sec
            airspeed_cmd: in meters/sec
            dt: timestep in seconds
        
        Returns:
            pitch_cmd: in radians
    """ 
    def airspeed_pitch_loop(self, airspeed, airspeed_cmd, dt):
        pitch_cmd = 0.0
        Evt = airspeed_cmd - airspeed
        self.i_v_t2 = self.i_v_t2 + Evt * dt

        pitch_cmd_unsat = self.k_p_v2 * Evt + self.k_i_v2 * self.i_v_t2

        if (np.abs(pitch_cmd_unsat) > self.max_pitch_cmd2):
            pitch_cmd = np.sign(pitch_cmd_unsat) * self.max_pitch_cmd2
        else:
            pitch_cmd = pitch_cmd_unsat

        # Anti wind-up
        if (self.k_i_v2 != 0):
            self.i_v_t2 = self.i_v_t2 + dt / self.k_i_v2 * (pitch_cmd - pitch_cmd_unsat)

        return pitch_cmd

    """Used to calculate the pitch command and throttle command based on the
    aicraft altitude error
    
        Args:
            airspeed: in meter/sec
            altitude: in meters (positive up)
            airspeed_cmd: in meters/sec
            altitude_cmd: in meters/sec (positive up)
            dt: timestep in seconds
            
        Returns:
            pitch_cmd: in radians
            throttle_cmd: in in percent throttle [0,1]
    """
    def longitudinal_loop(self, airspeed, altitude, airspeed_cmd, altitude_cmd,
                          dt):
        pitch_cmd = 0.0
        throttle_cmd = 0.0
        # STUDENT CODE HERE
        if ((altitude_cmd - altitude) > altitude_switch):
            throttle_cmd = 1.0
            pitch_cmd = self.airspeed_pitch_loop(airspeed, airspeed_cmd, dt)
        elif ((altitude - altitude_cmd) > altitude_switch):
            throttle_cmd = 0.1
            pitch_cmd = self.airspeed_pitch_loop(airspeed, airspeed_cmd, dt)
        else:
            throttle_cmd = self.airspeed_loop(airspeed, airspeed_cmd, dt)
            pitch_cmd = self.altitude_loop(altitude, altitude_cmd, dt)
        return[pitch_cmd, throttle_cmd]


 
class LateralAutoPilot:
    
    def __init__(self):
        self.g = 9.81
        self.integrator_yaw = 0.0 
        self.integrator_beta = 0.0
        self.gate = 1
        self.max_roll = 60*np.pi/180.0
        self.state = 1

        # roll attitude hold
        self.k_p_phi = 0
        self.k_d_phi = 0

        # sideslip hold
        self.k_p_beta = 0
        self.k_i_beta = 0

        # course hold
        self.k_p_yaw = 0.0
        self.k_i_yaw = 0.0

        self.gain_p_xtrack = 0.003
        self.gain_p_orbit = 0.0

    """Used to calculate the commanded aileron based on the roll error
    
        Args:
            phi_cmd: commanded roll in radians
            phi: roll angle in radians
            roll_rate: in radians/sec
            T_s: timestep in sec
            
        Returns:
            aileron: in percent full aileron [-1,1]
    """
    def roll_attitude_hold_loop(self,
                                phi_cmd,  # commanded roll
                                phi,    # actual roll 
                                roll_rate, 
                                T_s = 0.0):
        aileron = 0
        # STUDENT CODE HERE
        Ephit = phi_cmd - phi
        aileron = self.k_p_phi * Ephit - self.k_d_phi * roll_rate

        return aileron

    """Used to calculate the commanded roll angle from the course/yaw angle
    
        Args:
            yaw_cmd: commanded yaw in radians
            yaw: roll angle in radians
            roll_rate: in radians/sec
            T_s: timestep in sec
            
        Returns:
            roll_cmd: commanded roll in radians
    """
    def yaw_hold_loop(self, yaw_cmd, yaw, T_s, roll_ff=0):
        roll_cmd = 0
        Eyawt = yaw_cmd - yaw
        while (Eyawt < np.pi):
            Eyawt = Eyawt + 2*np.pi

        while (Eyawt >= np.pi):
            Eyawt = Eyawt - 2*np.pi

        self.integrator_yaw = self.integrator_yaw + Eyawt * T_s

        roll_cmd_unsat = self.k_p_yaw * Eyawt + roll_ff
        if (np.abs(roll_cmd_unsat) > self.max_roll):
            roll_cmd_unsat = np.sign(roll_cmd_unsat) * self.max_roll
        roll_cmd_unsat = roll_cmd_unsat + self.k_i_yaw * self.integrator_yaw
        if (np.abs(roll_cmd_unsat) > self.max_roll):
            roll_cmd = np.sign(roll_cmd_unsat) * self.max_roll
        else:
            roll_cmd = roll_cmd_unsat

        if (self.k_i_yaw != 0):
            self.integrator_yaw = self.integrator_yaw + (T_s/self.k_i_yaw)*(roll_cmd - roll_cmd_unsat)

        return roll_cmd

    """Used to calculate the commanded rudder based on the sideslip
    
        Args:
            beta: sideslip angle in radians
            T_s: timestep in sec
            
        Returns:
            rudder: in percent full rudder [-1,1]
    """
    def sideslip_hold_loop(self,
                           beta, # sideslip angle 
                           T_s):
        rudder = 0
        # STUDENT CODE HERE
        rudder_unsat = self.k_d_phi * (0.0 - beta)
        self.integrator_beta = self.integrator_beta + (0.0 - beta)*T_s
        if (np.abs(rudder_unsat) > 1):
            rudder_unsat = 1 * np.sign(rudder_unsat)

        rudder_unsat = rudder_unsat + self.k_i_beta * self.integrator_beta

        if (np.abs(rudder_unsat) > 1):
            rudder = 1 * np.sign(rudder_unsat)
        else:
            rudder = rudder_unsat

        if (self.k_i_beta != 0):
            self.integrator_beta = self.integrator_beta + (T_s/self.k_i_beta) * (rudder - rudder_unsat)

        return rudder
    
    """Used to calculate the desired course angle based on cross-track error
    from a desired line
    
        Args:
            line_origin: point on the desired line in meters [N, E, D]
            line_course: heading of the line in radians
            local_position: vehicle position in meters [N, E, D]
            
        Returns:
            course_cmd: course/yaw cmd for the vehicle in radians
    """
    def straight_line_guidance(self, line_origin, line_course, 
                               local_position):
        course_cmd = 0
        # STUDENT CODE HERE
        xtrack_error = np.cos(line_course) * (local_position[1] - line_origin[1])+\
                        -np.sin(line_course) * (local_position[0] - line_origin[0])
        course_cmd = -np.pi/2 * np.arctan(self.gain_p_xtrack * xtrack_error) + line_course
        
        return course_cmd
    
    """Used to calculate the desired course angle based on radius error from
    a specified orbit center
    
        Args:
            orbit_center: in meters [N, E, D]
            orbit_radius: desired radius in meters
            local_position: vehicle position in meters [N, E, D]
            yaw: vehicle heading in radians
            clockwise: specifies whether to fly clockwise (increasing yaw)
            
        Returns:
            course_cmd: course/yaw cmd for the vehicle in radians
    """
    def orbit_guidance(self, orbit_center, orbit_radius, local_position, yaw,
                       clockwise = True):
        course_cmd = 0
        # STUDENT CODE HERE
        radius = np.linalg.norm(orbit_center[0:2] - local_position[0:2])
        course_cmd = np.pi / 2 + np.arctan(self.gain_p_orbit * (radius - orbit_radius) / orbit_radius)
        if (clockwise == False):
            course_cmd = -course_cmd

        addon = np.arctan2(local_position[1] - orbit_center[1], local_position[0] - orbit_center[0])
        if ((addon - yaw) < -np.pi ):
            while ((addon - yaw) < -np.pi ):
                addon = addon + np.pi * 2

        if ((addon - yaw) > np.pi ):
            while ((addon - yaw) > np.pi ):
                addon = addon - np.pi * 2
        course_cmd = course_cmd + addon
        return course_cmd

    """Used to calculate the feedforward roll angle for a constant radius
    coordinated turn
    
        Args:
            speed: the aircraft speed during the turn in meters/sec
            radius: turning radius in meters
            cw: true=clockwise turn, false = counter-clockwise turn
            
        Returns:
            roll_ff: feed-forward roll in radians
    """
    def coordinated_turn_ff(self, speed, radius, cw):
        
        roll_ff = 0
        # STUDENT CODE HERE
        if (cw):
            roll_ff = np.arctan(speed**2/(self.g*radius))
        else:
            roll_ff = -np.arctan(speed**2/(self.g*radius))
        
        return roll_ff

    """Used to calculate the desired course angle and feed-forward roll
    depending on which phase of lateral flight (orbit or line following) the 
    aicraft is in
    
        Args:
            local_position: vehicle position in meters [N, E, D]
            yaw: vehicle heading in radians
            airspeed_cmd: in meters/sec
            
        Returns:
            roll_ff: feed-forward roll in radians
            yaw_cmd: commanded yaw/course in radians
    """
    def path_manager(self, local_position, yaw, airspeed_cmd):
        
        roll_ff = 0
        yaw_cmd = 0
        # STUDENT CODE HERE
        if (self.gate == 1):
            if(local_position[0] > 500):
                self.gate = self.gate + 1
                print('Gate 1 Complete')
                print('Yaw Int = ', self.integrator_yaw)
                self.integrator_yaw = 0.0
            else:
                roll_ff = 0.0
                line_origin = np.array([0.0, 20.0, -450.0])
                line_course = 0.0
                yaw_cmd = self.straight_line_guidance(line_origin, line_course, local_position)
        if (self.gate == 2):
            if(local_position[1] < -380):
                self.gate = self.gate + 1
                print('Gate 2 Complete')
                print('Yaw Int = ', self.integrator_yaw)
                self.integrator_yaw = 0.0
            else:
                radius = 400
                cw = False
                orbit_center = np.array([500.0, -380.0, -450.0])
                roll_ff = self.coordinated_turn_ff(airspeed_cmd, radius, cw)
                yaw_cmd = self.orbit_guidance(orbit_center, radius, local_position, yaw, cw)
        if(self.gate == 3):
            if(local_position[0] < 600):
                self.gate = self.gate + 1
                print('Gate 3 Complete')
                print('Yaw Int = ', self.integrator_yaw)
                self.integrator_yaw = 0.0
            else:
                radius = 300
                cw = False
                orbit_center = np.array([600.0, -380.0, -450.0])
                roll_ff = self.coordinated_turn_ff(airspeed_cmd, radius, cw)
                yaw_cmd = self.orbit_guidance(orbit_center, radius, local_position, cw)
        if(self.gate == 4):
            if(local_position[0] < -500):
                print('Lateral Challenge Finished')
                print('Yaw Int = ', self.integrator_yaw)
                self.integrator_yaw = 0.0
            else:
                roll_ff = 0.0
                line_origin = np.array([600.0, -680.0, -450.0])
                line_course = np.pi
                yaw_cmd = self.straight_line_guidance(line_origin, line_course, local_position)
        if(self.gate > 4):
            roll_ff = 0.0
            yaw_cmd = 0.0
            print('Invalid gate')

        return(roll_ff,yaw_cmd)
    
    
    """Used to calculate the desired course angle and feed-forward roll
    depending on which phase of lateral flight (orbit or line following) the 
    aicraft is in
    
        Args:
            waypoint_tuple: 3 waypoints, (prev_waypoint, curr_waypoint, next_waypoint), waypoints are in meters [N, E, D]
            local_position: vehicle position in meters [N, E, D]
            yaw: vehicle heading in radians
            airspeed_cmd: in meters/sec
            
        Returns:
            roll_ff: feed-forward roll in radians
            yaw_cmd: commanded yaw/course in radians
            cycle: True=cycle waypoints (at the end of orbit segment)
    """
    def waypoint_follower(self, waypoint_tuple, local_position, yaw, airspeed_cmd):
        roll_ff = 0.0
        yaw_cmd = 0.0
        cycle = False
        
        # STUDENT CODE HERE
        radius = 500

        prev_waypoint = waypoint_tuple[0][0:2]
        curr_waypoint = waypoint_tuple[1][0:2]
        next_waypoint = waypoint_tuple[2][0:2]
        q0 = (curr_waypoint - prev_waypoint)/np.linalg.norm(curr_waypoint - prev_waypoint)
        q1 = (next_waypoint - curr_waypoint)/np.linalg.norm(next_waypoint - curr_waypoint)
        angle = np.arccos(-np.dot(q0, q1))

        if self.state == 1:
            q = q0
            z = curr_waypoint - (radius/np.tan(angle/2))*q0
            course = np.arctan2(q[1], q[0])
            if np.dot(local_position - z, q) > 0:
                self.state = 2
                self.integrator_yaw = 0
            else:
                roll_ff = 0
                line_origin = prev_waypoint
                line_course = course
                yaw_cmd = self.straight_line_guidance(line_origin, line_course, local_position)
        elif self.state == 2:
            c = curr_waypoint - (radius/np.sin(angle/2))*(q0-q1)/np.linalg.norm(q0-q1)
            z = curr_waypoint + (radius/np.tan(angle/2))*q1
            cw = np.sign(q0[0]*q1[1] - q0[1]*q1[0])
            if np.dot(local_position - z, q1) > 0:
                print('Center: ', c)
                print('Transtion: ', z)
                self.state = 1
                self.integrator_yaw = 0
                cycle = True
            else:
                yaw_cmd = self.orbit_guidance(c, radius, local_position, yaw, cw>0)
                roll_ff = self.coordinated_turn_ff(airspeed_cmd, radius, cw>0)
        
        return(roll_ff, yaw_cmd, cycle)



def euler2RM(roll,pitch,yaw):
    R = np.array([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]])
    cr = np.cos(roll)
    sr = np.sin(roll)
    
    cp = np.cos(pitch)
    sp = np.sin(pitch)
    
    cy = np.cos(yaw)
    sy = np.sin(yaw)
    
    R[0,0] = cp*cy
    R[1,0] = -cr*sy+sr*sp*cy
    R[2,0] = sr*sy+cr*sp*cy
    
    R[0,1] = cp*sy
    R[1,1] = cr*cy+sr*sp*sy
    R[2,1] = -sr*cy+cr*sp*sy
    
    R[0,2] = -sp
    R[1,2] = sr*cp
    R[2,2] = cr*cp
    
    return R.transpose()

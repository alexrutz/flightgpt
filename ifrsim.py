import jsbsim
import time


class PIDController:
    """Very small PID controller used for the autopilot."""

    def __init__(self, kp, ki=0.0, kd=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative


class Autopilot:
    """Simple heading, altitude and speed hold with some system automation."""

    def __init__(self, fdm, dt):
        self.fdm = fdm
        self.dt = dt
        self.altitude = 0.0
        self.heading = 0.0
        self.speed = 0.0
        # Altitude control is split into an altitude to vertical speed loop
        # and a vertical speed to pitch loop for better stability
        self.alt_pid = PIDController(0.002)
        self.vs_pid = PIDController(0.005)
        self.hdg_pid = PIDController(0.02)
        self.spd_pid = PIDController(0.01)
        self.yaw_pid = PIDController(0.1)

    def set_targets(self, altitude=None, heading=None, speed=None):
        if altitude is not None:
            self.altitude = altitude
        if heading is not None:
            self.heading = heading
        if speed is not None:
            self.speed = speed

    def _manage_systems(self, alt, speed):
        """Very naive flap and gear scheduling for a bit of system depth."""
        f = self.fdm
        # Landing gear
        gear_cmd = 1.0 if alt < 1500 and speed < 180 else 0.0
        f['gear/gear-cmd-norm'] = gear_cmd

        # Simple flap schedule based on speed
        if speed < 120:
            flap = 1.0
        elif speed < 140:
            flap = 0.75
        elif speed < 160:
            flap = 0.5
        elif speed < 180:
            flap = 0.25
        else:
            flap = 0.0
        f['fcs/flap-cmd-norm'] = flap

    def update(self):
        f = self.fdm
        alt = f.get_property_value('position/h-sl-ft')
        psi = f.get_property_value('attitude/psi-deg')
        speed = f.get_property_value('velocities/vt-fps') / 1.68781
        vs = f.get_property_value('velocities/h-dot-fps')  # ft/s

        # Altitude hold -> vertical speed target (ft/min)
        alt_error = self.altitude - alt
        vs_target_fpm = self.alt_pid.update(alt_error, self.dt) * 60.0
        vs_target_fpm = max(min(vs_target_fpm, 3000.0), -3000.0)
        vs_error = vs_target_fpm / 60.0 - vs
        pitch_cmd = max(min(self.vs_pid.update(vs_error, self.dt), 0.5), -0.5)
        f['fcs/elevator-cmd-norm'] = pitch_cmd

        heading_error = (self.heading - psi + 180) % 360 - 180
        aileron_cmd = max(min(self.hdg_pid.update(heading_error, self.dt), 0.3), -0.3)
        f['fcs/aileron-cmd-norm'] = aileron_cmd

        slip = f.get_property_value('aero/beta-rad')
        rudder_cmd = max(min(self.yaw_pid.update(-slip, self.dt), 0.3), -0.3)
        f['fcs/rudder-cmd-norm'] = rudder_cmd

        speed_error = self.speed - speed
        throttle_cmd = max(min(self.spd_pid.update(speed_error, self.dt), 1.0), 0.0)
        f['fcs/throttle-cmd-norm'] = throttle_cmd

        self._manage_systems(alt, speed)

        return alt, speed, psi, pitch_cmd, aileron_cmd, throttle_cmd

class A320IFRSim:
    def __init__(self, root_dir='jsbsim-master', dt=0.02):
        self.fdm = jsbsim.FGFDMExec(None, None)
        self.fdm.disable_output()
        self.fdm.set_root_dir(root_dir)
        self.fdm.load_model('A320')
        self.fdm.set_dt(dt)
        self.target_altitude = 4000  # feet
        self.target_psi = 0  # heading degrees
        self.target_speed = 250  # knots
        self.autopilot = Autopilot(self.fdm, dt)
        self.init_conditions()
        self.autopilot.set_targets(self.target_altitude, self.target_psi, self.target_speed)

    def init_conditions(self):
        f = self.fdm
        f['ic/altitude-ft'] = self.target_altitude
        f['ic/psi-true-deg'] = self.target_psi
        f['ic/u-fps'] = self.target_speed * 1.68781  # convert kts to fps
        f['ic/vt-fps'] = self.target_speed * 1.68781
        f['ic/vc-kts'] = self.target_speed
        f['ic/h-sl-ft'] = self.target_altitude
        f['ic/long_gc-deg'] = -122.0
        f['ic/lat_gc-deg'] = 37.615
        f['ic/weight-lbs'] = 130000
        f.run_ic()
        f['propulsion/set-running'] = 1

    def step(self):
        alt, speed, psi, pitch_cmd, aileron_cmd, throttle_cmd = self.autopilot.update()
        fuel = self.fdm.get_property_value('propulsion/total-fuel-lbs')
        flap = self.fdm.get_property_value('fcs/flap-pos-norm')
        gear = self.fdm.get_property_value('gear/gear-pos-norm')
        vs_fpm = self.fdm.get_property_value('velocities/h-dot-fps') * 60.0
        self.fdm.run()
        return {
            'altitude_ft': alt,
            'speed_kt': speed,
            'heading_deg': psi,
            'vs_fpm': vs_fpm,
            'pitch_cmd': pitch_cmd,
            'aileron_cmd': aileron_cmd,
            'throttle_cmd': throttle_cmd,
            'flap': flap,
            'gear': gear,
            'fuel_lbs': fuel,
        }

    def run(self, steps=600):
        for i in range(steps):
            data = self.step()
            if i % 50 == 0:
                print(
                    f"t={i*self.fdm.get_delta_t():.1f}s alt={data['altitude_ft']:.1f}ft "
                    f"spd={data['speed_kt']:.1f}kt hdg={data['heading_deg']:.1f} "
                    f"vs={data['vs_fpm']:.0f}fpm flap={data['flap']:.2f} "
                    f"gear={data['gear']:.0f} fuel={data['fuel_lbs']:.0f}lb"
                )
            time.sleep(self.fdm.get_delta_t())

if __name__ == '__main__':
    sim = A320IFRSim()
    sim.run(300)

import jsbsim
import time

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
        self.init_conditions()

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

    def step(self):
        f = self.fdm
        alt = f.get_property_value('position/h-sl-ft')
        speed = f.get_property_value('velocities/vt-fps') / 1.68781
        psi = f.get_property_value('attitude/psi-deg')

        # Simple controllers
        alt_error = self.target_altitude - alt
        pitch_cmd = max(min(alt_error * 0.0005, 0.2), -0.2)
        f['fcs/elevator-cmd-norm'] = pitch_cmd

        heading_error = (self.target_psi - psi + 180) % 360 - 180
        aileron_cmd = max(min(heading_error * 0.02, 0.3), -0.3)
        f['fcs/aileron-cmd-norm'] = aileron_cmd

        speed_error = self.target_speed - speed
        throttle_cmd = max(min(speed_error * 0.01, 1.0), 0)
        f['fcs/throttle-cmd-norm'] = throttle_cmd

        f.run()
        return {
            'altitude_ft': alt,
            'speed_kt': speed,
            'heading_deg': psi,
            'pitch_cmd': pitch_cmd,
            'aileron_cmd': aileron_cmd,
            'throttle_cmd': throttle_cmd,
        }

    def run(self, steps=600):
        for i in range(steps):
            data = self.step()
            if i % 50 == 0:
                print(f"t={i*self.fdm.get_delta_t():.1f}s alt={data['altitude_ft']:.1f}ft speed={data['speed_kt']:.1f}kt heading={data['heading_deg']:.1f}")
            time.sleep(self.fdm.get_delta_t())

if __name__ == '__main__':
    sim = A320IFRSim()
    sim.run(300)

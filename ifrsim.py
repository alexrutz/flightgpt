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


class Engine:
    """Simple engine model with throttle spool dynamics."""

    def __init__(self, fdm, spool_rate=0.2):
        self.fdm = fdm
        self.spool_rate = spool_rate
        self.throttle = 0.0
        self.target = 0.0

    def set_target(self, throttle: float) -> None:
        """Set desired throttle position."""
        self.target = max(0.0, min(throttle, 1.0))

    def update(self, dt: float) -> None:
        """Update engine throttle command with a simple first order lag."""
        diff = self.target - self.throttle
        max_delta = self.spool_rate * dt
        diff = max(min(diff, max_delta), -max_delta)
        self.throttle += diff
        # Apply command to both engines
        self.fdm['fcs/throttle-cmd-norm'] = self.throttle
        self.fdm['fcs/throttle-cmd-norm[1]'] = self.throttle

    def n1(self) -> float:
        """Return the N1 (fan speed) of the first engine."""
        return self.fdm.get_property_value('propulsion/engine/n1')


class HydraulicSystem:
    """Very small hydraulic system model."""

    def __init__(self, pump_rate=0.3, usage_factor=0.5, leak_rate=0.01):
        self.pressure = 1.0
        self.pump_rate = pump_rate
        self.usage_factor = usage_factor
        self.leak_rate = leak_rate
        self.pump_on = True

    def update(self, demand: float, dt: float) -> float:
        if self.pump_on:
            self.pressure += self.pump_rate * dt
        self.pressure -= (self.leak_rate + demand * self.usage_factor) * dt
        self.pressure = max(0.0, min(self.pressure, 1.0))
        return self.pressure


class SystemManager:
    """Handles slow actuation of gear and flaps with hydraulics."""

    def __init__(self, fdm, flap_rate=0.5, gear_rate=0.5):
        self.fdm = fdm
        self.flap_rate = flap_rate
        self.gear_rate = gear_rate
        self.flap = 0.0
        self.gear = 0.0
        self.target_flap = 0.0
        self.target_gear = 0.0
        self.hydraulics = HydraulicSystem()

    def set_targets(self, gear=None, flap=None):
        if gear is not None:
            self.target_gear = max(0.0, min(gear, 1.0))
        if flap is not None:
            self.target_flap = max(0.0, min(flap, 1.0))

    def update(self, dt):
        d_flap = self.target_flap - self.flap
        d_gear = self.target_gear - self.gear
        demand = abs(d_flap) + abs(d_gear)
        pressure = self.hydraulics.update(demand, dt)

        max_df = self.flap_rate * dt * pressure
        d_flap = max(min(d_flap, max_df), -max_df)
        self.flap += d_flap
        self.fdm['fcs/flap-cmd-norm'] = self.flap

        max_dg = self.gear_rate * dt * pressure
        d_gear = max(min(d_gear, max_dg), -max_dg)
        self.gear += d_gear
        self.fdm['gear/gear-cmd-norm'] = self.gear

        return pressure, demand


class FuelSystem:
    """Track fuel quantity and burn rate for a simple two tank setup."""

    def __init__(self, fdm):
        self.fdm = fdm
        self.prev_used_0 = fdm.get_property_value('propulsion/engine/fuel-used-lbs')
        self.prev_used_1 = fdm.get_property_value('propulsion/engine[1]/fuel-used-lbs')

    def update(self):
        dt = self.fdm.get_delta_t()
        used_0 = self.fdm.get_property_value('propulsion/engine/fuel-used-lbs')
        used_1 = self.fdm.get_property_value('propulsion/engine[1]/fuel-used-lbs')
        flow_0 = (used_0 - self.prev_used_0) / dt * 3600.0
        flow_1 = (used_1 - self.prev_used_1) / dt * 3600.0
        self.prev_used_0 = used_0
        self.prev_used_1 = used_1
        left = self.fdm.get_property_value('propulsion/tank/contents-lbs')
        right = self.fdm.get_property_value('propulsion/tank[1]/contents-lbs')
        total = self.fdm.get_property_value('propulsion/total-fuel-lbs')
        return {
            'left_lbs': left,
            'right_lbs': right,
            'total_lbs': total,
            'flow0_pph': flow_0,
            'flow1_pph': flow_1,
        }


class ElectricSystem:
    """Very small electrical system with a battery and generator."""

    def __init__(self, charge_rate=0.2, discharge_rate=0.05):
        self.charge = 1.0
        self.charge_rate = charge_rate
        self.discharge_rate = discharge_rate

    def update(self, generator_on: bool, demand: float, dt: float) -> float:
        if generator_on:
            self.charge += self.charge_rate * dt
        self.charge -= demand * self.discharge_rate * dt
        self.charge = max(0.0, min(self.charge, 1.0))
        return self.charge

    def is_powered(self) -> bool:
        return self.charge > 0.05


class Autopilot:
    """Heading, altitude and speed hold with basic system automation."""

    def __init__(self, fdm, dt, engine, systems, electrics):
        self.fdm = fdm
        self.dt = dt
        self.engine = engine
        self.systems = systems
        self.electrics = electrics
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
        # Landing gear
        gear_cmd = 1.0 if alt < 1500 and speed < 180 else 0.0

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
        self.systems.set_targets(gear=gear_cmd, flap=flap)

    def update(self):
        f = self.fdm
        alt = f.get_property_value('position/h-sl-ft')
        psi = f.get_property_value('attitude/psi-deg')
        speed = f.get_property_value('velocities/vt-fps') / 1.68781
        vs = f.get_property_value('velocities/h-dot-fps')  # ft/s

        powered = self.electrics.is_powered()

        # Altitude hold -> vertical speed target (ft/min)
        alt_error = self.altitude - alt
        vs_target_fpm = self.alt_pid.update(alt_error, self.dt) * 60.0
        vs_target_fpm = max(min(vs_target_fpm, 3000.0), -3000.0)
        vs_error = vs_target_fpm / 60.0 - vs
        pitch_cmd = max(min(self.vs_pid.update(vs_error, self.dt), 0.5), -0.5)
        if powered:
            f['fcs/elevator-cmd-norm'] = pitch_cmd
        else:
            f['fcs/elevator-cmd-norm'] = 0.0

        heading_error = (self.heading - psi + 180) % 360 - 180
        aileron_cmd = max(min(self.hdg_pid.update(heading_error, self.dt), 0.3), -0.3)
        if powered:
            f['fcs/aileron-cmd-norm'] = aileron_cmd
        else:
            f['fcs/aileron-cmd-norm'] = 0.0

        slip = f.get_property_value('aero/beta-rad')
        rudder_cmd = max(min(self.yaw_pid.update(-slip, self.dt), 0.3), -0.3)
        if powered:
            f['fcs/rudder-cmd-norm'] = rudder_cmd
        else:
            f['fcs/rudder-cmd-norm'] = 0.0

        speed_error = self.speed - speed
        throttle_cmd = max(min(self.spd_pid.update(speed_error, self.dt), 1.0), 0.0)
        if powered:
            self.engine.set_target(throttle_cmd)
        else:
            self.engine.set_target(0.0)
        self.engine.update(self.dt)

        self._manage_systems(alt, speed)
        pressure, demand = self.systems.update(self.dt)

        n1 = self.engine.n1()
        return (
            alt,
            speed,
            psi,
            pitch_cmd,
            aileron_cmd,
            self.engine.throttle,
            n1,
            pressure,
            demand,
        )

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
        self.engine = Engine(self.fdm)
        self.systems = SystemManager(self.fdm)
        self.fuel = FuelSystem(self.fdm)
        self.electrics = ElectricSystem()
        self.autopilot = Autopilot(self.fdm, dt, self.engine, self.systems, self.electrics)
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
        (
            alt,
            speed,
            psi,
            pitch_cmd,
            aileron_cmd,
            throttle_cmd,
            n1,
            pressure,
            hyd_demand,
        ) = self.autopilot.update()
        elec = self.electrics.update(n1 > 0.5, hyd_demand + 0.1, self.fdm.get_delta_t())
        fuel_data = self.fuel.update()
        fuel = fuel_data['total_lbs']
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
            'n1': n1,
            'flap': flap,
            'gear': gear,
            'hyd_press': pressure,
            'elec_charge': elec,
            'fuel_lbs': fuel,
            'fuel_flow_lbs_hr_eng1': fuel_data['flow0_pph'],
            'fuel_flow_lbs_hr_eng2': fuel_data['flow1_pph'],
        }

    def run(self, steps=600):
        for i in range(steps):
            data = self.step()
            if i % 50 == 0:
                print(
                    f"t={i*self.fdm.get_delta_t():.1f}s alt={data['altitude_ft']:.1f}ft "
                    f"spd={data['speed_kt']:.1f}kt hdg={data['heading_deg']:.1f} "
                    f"vs={data['vs_fpm']:.0f}fpm flap={data['flap']:.2f} "
                    f"gear={data['gear']:.0f} thr={data['throttle_cmd']:.2f} "
                    f"n1={ (data['n1']*100 if data['n1']<=2 else data['n1']):.0f}% "
                    f"hyd={data['hyd_press']:.2f} elec={data['elec_charge']:.2f} "
                    f"fuel={data['fuel_lbs']:.0f}lb "
                    f"ff={data['fuel_flow_lbs_hr_eng1']:.0f}/{data['fuel_flow_lbs_hr_eng2']:.0f} pph"
                )
            time.sleep(self.fdm.get_delta_t())

if __name__ == '__main__':
    sim = A320IFRSim()
    sim.run(300)

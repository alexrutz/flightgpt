import jsbsim
import time
import math
import random


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
        self.efficiency = 1.0
        self.extra_fuel_factor = 0.0

    def set_target(self, throttle: float) -> None:
        """Set desired throttle position."""
        self.target = max(0.0, min(throttle, 1.0))

    def update(self, dt: float) -> None:
        """Update engine throttle command with a simple non-linear lag."""
        diff = self.target - self.throttle
        # Allow faster spool when the commanded change is large
        rate = self.spool_rate * (1.0 + 2.0 * abs(diff))
        max_delta = rate * dt
        diff = max(min(diff, max_delta), -max_delta)
        self.throttle += diff
        # Apply command to both engines with efficiency factor
        cmd = self.throttle * self.efficiency
        self.fdm['fcs/throttle-cmd-norm'] = cmd
        self.fdm['fcs/throttle-cmd-norm[1]'] = cmd

    def n1(self) -> float:
        """Return the N1 (fan speed) of the first engine."""
        return self.fdm.get_property_value('propulsion/engine/n1')


class Autothrottle:
    """Maintain target airspeed by commanding engine thrust."""

    def __init__(self, fdm, engine, kp=0.01, ki=0.0, kd=0.0):
        self.fdm = fdm
        self.engine = engine
        self.pid = PIDController(kp, ki, kd)
        self.target_speed = 0.0

    def set_target(self, speed_kt: float) -> None:
        """Set desired airspeed in knots."""
        self.target_speed = speed_kt

    def update(self, dt: float, powered: bool = True) -> float:
        """Update engine throttle to hold the target speed."""
        speed = self.fdm.get_property_value('velocities/vt-fps') / 1.68781
        throttle_cmd = self.pid.update(self.target_speed - speed, dt)
        throttle_cmd = max(0.0, min(throttle_cmd, 1.0))
        if powered:
            self.engine.set_target(throttle_cmd)
        else:
            self.engine.set_target(0.0)
        self.engine.update(dt)
        return self.engine.throttle


class HydraulicSystem:
    """Very small hydraulic system model with basic failures."""

    def __init__(self, pump_rate=0.3, usage_factor=0.5, leak_rate=0.01, failure_chance=0.0):
        self.pressure = 1.0
        self.pump_rate = pump_rate
        self.usage_factor = usage_factor
        self.leak_rate = leak_rate
        self.failure_chance = failure_chance
        self.pump_on = True

    def update(self, demand: float, dt: float) -> float:
        if self.pump_on:
            if random.random() < self.failure_chance * dt:
                self.pump_on = False
            else:
                self.pressure += self.pump_rate * dt
        self.pressure -= (self.leak_rate + demand * self.usage_factor) * dt
        self.pressure = max(0.0, min(self.pressure, 1.0))
        return self.pressure


class SystemManager:
    """Handles slow actuation of gear, flaps and speedbrake with hydraulics."""

    def __init__(self, fdm, flap_rate=0.5, gear_rate=0.5, speedbrake_rate=0.5, failure_chance=0.0):
        self.fdm = fdm
        self.flap_rate = flap_rate
        self.gear_rate = gear_rate
        self.speedbrake_rate = speedbrake_rate
        self.flap = 0.0
        self.gear = 0.0
        self.speedbrake = 0.0
        self.target_flap = 0.0
        self.target_gear = 0.0
        self.target_speedbrake = 0.0
        self.hydraulics = HydraulicSystem(failure_chance=failure_chance)

    def set_targets(self, gear=None, flap=None, speedbrake=None):
        if gear is not None:
            self.target_gear = max(0.0, min(gear, 1.0))
        if flap is not None:
            self.target_flap = max(0.0, min(flap, 1.0))
        if speedbrake is not None:
            self.target_speedbrake = max(0.0, min(speedbrake, 1.0))

    def update(self, dt):
        d_flap = self.target_flap - self.flap
        d_gear = self.target_gear - self.gear
        d_speedbrake = self.target_speedbrake - self.speedbrake
        demand = abs(d_flap) + abs(d_gear) + abs(d_speedbrake)
        pressure = self.hydraulics.update(demand, dt)

        max_df = self.flap_rate * dt * pressure
        d_flap = max(min(d_flap, max_df), -max_df)
        self.flap += d_flap
        self.fdm['fcs/flap-cmd-norm'] = self.flap

        max_dg = self.gear_rate * dt * pressure
        d_gear = max(min(d_gear, max_dg), -max_dg)
        self.gear += d_gear
        self.fdm['gear/gear-cmd-norm'] = self.gear

        max_ds = self.speedbrake_rate * dt * pressure
        d_speedbrake = max(min(d_speedbrake, max_ds), -max_ds)
        self.speedbrake += d_speedbrake
        self.fdm['fcs/speedbrake-cmd-norm'] = self.speedbrake

        return pressure, demand


class FuelSystem:
    """Track fuel quantity and burn rate for a simple two tank setup.

    A very small crossfeed logic equalises the tanks when the imbalance
    exceeds a threshold. Crossfeed is automatic and stops once the tanks
    are nearly balanced.
    """

    def __init__(
        self,
        fdm,
        engine=None,
        electrics=None,
        apu_flow_pph=150.0,
        crossfeed_rate_pph=1200.0,
        balance_threshold_lbs=1000.0,
    ):
        self.fdm = fdm
        self.engine = engine
        self.electrics = electrics
        self.apu_flow_pph = apu_flow_pph
        self.crossfeed_rate_pph = crossfeed_rate_pph
        self.balance_threshold_lbs = balance_threshold_lbs
        self.crossfeed_on = False
        self.prev_used_0 = fdm.get_property_value('propulsion/engine/fuel-used-lbs')
        self.prev_used_1 = fdm.get_property_value('propulsion/engine[1]/fuel-used-lbs')

    def update(self):
        dt = self.fdm.get_delta_t()
        used_0 = self.fdm.get_property_value('propulsion/engine/fuel-used-lbs')
        used_1 = self.fdm.get_property_value('propulsion/engine[1]/fuel-used-lbs')
        flow_0 = (used_0 - self.prev_used_0) / dt * 3600.0
        flow_1 = (used_1 - self.prev_used_1) / dt * 3600.0
        if self.engine is not None:
            flow_0 *= 1.0 + self.engine.extra_fuel_factor
            flow_1 *= 1.0 + self.engine.extra_fuel_factor
        self.prev_used_0 = used_0
        self.prev_used_1 = used_1

        left = self.fdm.get_property_value('propulsion/tank/contents-lbs')
        right = self.fdm.get_property_value('propulsion/tank[1]/contents-lbs')

        # Automatic crossfeed to keep tanks balanced
        diff = left - right
        if not self.crossfeed_on and abs(diff) > self.balance_threshold_lbs:
            self.crossfeed_on = True
        if self.crossfeed_on:
            xfeed_amt = min(
                abs(diff),
                self.crossfeed_rate_pph / 3600.0 * dt,
            )
            if diff > 0:
                left -= xfeed_amt
                right += xfeed_amt
            else:
                left += xfeed_amt
                right -= xfeed_amt
            if abs(left - right) < self.balance_threshold_lbs * 0.2:
                self.crossfeed_on = False
            self.fdm['propulsion/tank/contents-lbs'] = left
            self.fdm['propulsion/tank[1]/contents-lbs'] = right

        apu_flow = 0.0
        if self.electrics and self.electrics.apu_running:
            apu_flow = self.apu_flow_pph
            burn = apu_flow / 3600.0 * dt
            left = max(left - burn, 0.0)
            self.fdm['propulsion/tank/contents-lbs'] = left

        total = left + right
        return {
            'left_lbs': left,
            'right_lbs': right,
            'total_lbs': total,
            'flow0_pph': flow_0,
            'flow1_pph': flow_1,
            'apu_flow_pph': apu_flow,
            'crossfeed': self.crossfeed_on,
        }


class ElectricSystem:
    """Electrical system with battery, generator and a simple APU."""

    def __init__(
        self,
        charge_rate=0.2,
        discharge_rate=0.05,
        apu_charge_rate=0.1,
        apu_start_time=5.0,
    ):
        self.charge = 1.0
        self.charge_rate = charge_rate
        self.discharge_rate = discharge_rate
        self.apu_charge_rate = apu_charge_rate
        self.apu_start_time = apu_start_time
        self.apu_running = False
        self.apu_timer = 0.0

    def start_apu(self) -> None:
        if not self.apu_running:
            self.apu_running = True
            self.apu_timer = 0.0

    def stop_apu(self) -> None:
        self.apu_running = False

    def update(self, generator_on: bool, demand: float, dt: float) -> float:
        if generator_on:
            self.charge += self.charge_rate * dt
        elif self.apu_running:
            # Allow a small delay before the APU provides power
            self.apu_timer += dt
            if self.apu_timer >= self.apu_start_time:
                self.charge += self.apu_charge_rate * dt

        self.charge -= demand * self.discharge_rate * dt
        self.charge = max(0.0, min(self.charge, 1.0))

        # Automatically start the APU if the battery is low and the
        # generator is not running. Stop it once the battery has recovered.
        if self.charge < 0.3 and not generator_on:
            self.start_apu()
        if self.charge > 0.95 and self.apu_running and generator_on:
            self.stop_apu()

        return self.charge

    def is_powered(self) -> bool:
        return self.charge > 0.05


class BleedAirSystem:
    """Provide bleed air from the engines or APU."""

    def __init__(self, engine, electrics, engine_gain=1.0, apu_gain=0.5):
        self.engine = engine
        self.electrics = electrics
        self.engine_gain = engine_gain
        self.apu_gain = apu_gain
        self.pressure = 0.0

    def update(self) -> float:
        self.pressure = 0.0
        n1 = self.engine.n1()
        if n1 > 0.2:
            self.pressure += n1 * self.engine_gain
        if self.electrics.apu_running:
            self.pressure += self.apu_gain
        self.pressure = max(0.0, min(self.pressure, 1.0))
        return self.pressure


class EngineStartSystem:
    """Start the engines using bleed air with a short delay."""

    def __init__(self, fdm, bleed, start_time=8.0):
        self.fdm = fdm
        self.bleed = bleed
        self.start_time = start_time
        self.timer = 0.0
        self.state = "stopped"

    def request_start(self) -> None:
        if self.state == "stopped":
            self.state = "starting"
            self.timer = 0.0
            self.fdm["propulsion/engine/set-running"] = 0
            self.fdm["propulsion/engine[1]/set-running"] = 0

    def update(self, dt: float) -> bool:
        if self.state == "starting":
            if self.bleed.pressure > 0.3:
                self.timer += dt
            if self.timer >= self.start_time:
                self.fdm["propulsion/engine/set-running"] = 1
                self.fdm["propulsion/engine[1]/set-running"] = 1
                self.state = "running"
        return self.state == "running"


class Environment:
    """Wind model with simple lateral and vertical gusts."""

    def __init__(self, fdm, gust_strength=5.0, vertical_strength=2.0, base_temp=15.0):
        self.fdm = fdm
        self.gust_strength = gust_strength
        self.vertical_strength = vertical_strength
        self.base_temp = base_temp
        self.temperature_c = base_temp
        self.precip = 0.0
        self.t = 0.0

    def update(self, dt):
        self.t += dt
        base = 10.0 * math.sin(self.t / 30.0)
        gust = self.gust_strength * math.sin(self.t * 10.0)
        gust += random.uniform(-self.gust_strength, self.gust_strength) * 0.1
        self.fdm['atmosphere/wind-north-fps'] = 0.0
        self.fdm['atmosphere/wind-east-fps'] = base + gust

        vert_base = 2.0 * math.sin(self.t / 40.0)
        vert_gust = self.vertical_strength * math.sin(self.t * 12.0)
        vert_gust += random.uniform(-self.vertical_strength, self.vertical_strength) * 0.1
        self.fdm['atmosphere/wind-down-fps'] = vert_base + vert_gust

        alt = self.fdm.get_property_value('position/h-sl-ft')
        self.temperature_c = self.base_temp - 0.002 * alt
        if random.random() < 0.01:
            self.precip = random.uniform(0.2, 1.0)
        self.precip *= 0.99

    def is_icing(self):
        return self.temperature_c < 2.0 and self.precip > 0.3


class AntiIceSystem:
    """Minimal anti-ice system that counters engine icing."""

    def __init__(self, environment, engine, bleed=None, melt_rate=0.05, acc_rate=0.1):
        self.environment = environment
        self.engine = engine
        self.bleed = bleed
        self.melt_rate = melt_rate
        self.acc_rate = acc_rate
        self.active = False
        self.ice = 0.0

    def set_active(self, state: bool) -> None:
        self.active = state

    def update(self, dt: float):
        if self.environment.is_icing() and not self.active:
            self.ice += self.acc_rate * dt
        else:
            melt = self.melt_rate
            if self.active and self.bleed is not None:
                melt *= self.bleed.pressure
            self.ice -= melt * dt
        self.ice = max(0.0, min(1.0, self.ice))
        self.engine.efficiency = 1.0 - 0.3 * self.ice
        self.engine.extra_fuel_factor = 0.05 if self.active else 0.0
        return self.active, self.ice


class PressurizationSystem:
    """Track cabin altitude based on a simple pressurization model."""

    def __init__(self, fdm, bleed=None, target_diff=8.0, leak_rate=0.001):
        self.fdm = fdm
        self.bleed = bleed
        self.target_diff = target_diff
        self.leak_rate = leak_rate
        self.cabin_alt = fdm.get_property_value('position/h-sl-ft')

    def _pressure_at_alt(self, alt):
        return 14.7 * math.exp(-alt / 20000.0)

    def update(self, dt):
        alt = self.fdm.get_property_value('position/h-sl-ft')
        amb = self._pressure_at_alt(alt)
        cab = self._pressure_at_alt(self.cabin_alt)
        diff = cab - amb
        bleed = 1.0
        if self.bleed is not None:
            bleed = self.bleed.update()
        cab += (self.target_diff - diff) * 0.1 * dt * bleed
        self.cabin_alt = -20000.0 * math.log(max(cab, 0.01) / 14.7)
        self.cabin_alt += (alt - self.cabin_alt) * self.leak_rate * dt
        return self.cabin_alt, diff, bleed


class OxygenSystem:
    """Monitor oxygen supply based on cabin altitude."""

    def __init__(self, capacity=1.0, usage_rate=0.01, alt_threshold=10000.0):
        self.level = capacity
        self.usage_rate = usage_rate
        self.alt_threshold = alt_threshold

    def update(self, cabin_alt, dt):
        if cabin_alt > self.alt_threshold:
            excess = cabin_alt - self.alt_threshold
            self.level -= self.usage_rate * (excess / 10000.0) * dt
        self.level = max(0.0, min(self.level, 1.0))
        return self.level


class StallWarningSystem:
    """Very simple stall warning based on AoA and airspeed."""

    def __init__(self, fdm, alpha_deg=12.0, speed_kt=120.0):
        self.fdm = fdm
        self.alpha_thresh = math.radians(alpha_deg)
        self.speed_thresh = speed_kt

    def update(self):
        alpha = self.fdm.get_property_value('aero/alpha-rad')
        speed = self.fdm.get_property_value('velocities/vt-fps') / 1.68781
        return alpha > self.alpha_thresh or speed < self.speed_thresh


class GroundProximityWarningSystem:
    """Warn when approaching the ground with a high descent rate."""

    def __init__(self, fdm, alt_ft=200.0, sink_rate_fpm=1500.0):
        self.fdm = fdm
        self.alt_thresh = alt_ft
        self.sink_thresh = -abs(sink_rate_fpm)

    def update(self):
        agl = self.fdm.get_property_value('position/h-agl-ft')
        vs_fpm = self.fdm.get_property_value('velocities/h-dot-fps') * 60.0
        return agl < self.alt_thresh and vs_fpm < self.sink_thresh


class OverspeedWarningSystem:
    """Warn when exceeding a structural speed limit."""

    def __init__(self, fdm, limit_kt=320.0):
        self.fdm = fdm
        self.limit = limit_kt

    def update(self):
        speed = self.fdm.get_property_value('velocities/vt-fps') / 1.68781
        return speed > self.limit


class Autopilot:
    """Heading, altitude and speed hold with basic system automation."""

    def __init__(
        self,
        fdm,
        dt,
        engine,
        systems,
        electrics,
        environment,
        anti_ice,
        capture_window=200.0,
        climb_vs_fpm=1500.0,
        descent_vs_fpm=-1500.0,
    ):
        self.fdm = fdm
        self.dt = dt
        self.engine = engine
        self.systems = systems
        self.electrics = electrics
        self.environment = environment
        self.anti_ice = anti_ice
        self.autothrottle = Autothrottle(fdm, engine)
        self.altitude = 0.0
        self.heading = 0.0
        self.speed = 0.0
        # Altitude control is split into an altitude to vertical speed loop
        # and a vertical speed to pitch loop for better stability
        self.alt_pid = PIDController(0.002)
        self.vs_pid = PIDController(0.005)
        self.hdg_pid = PIDController(0.02)
        self.yaw_pid = PIDController(0.1)
        self.vs_target_fpm = 0.0
        self.capture_window = capture_window
        self.climb_vs_fpm = climb_vs_fpm
        self.descent_vs_fpm = descent_vs_fpm

    def set_targets(self, altitude=None, heading=None, speed=None, vs=None):
        if altitude is not None:
            if altitude != self.altitude:
                self.vs_target_fpm = (
                    self.climb_vs_fpm if altitude > self.altitude else self.descent_vs_fpm
                )
            self.altitude = altitude
        if heading is not None:
            self.heading = heading
        if speed is not None:
            self.speed = speed
            self.autothrottle.set_target(speed)
        if vs is not None:
            self.vs_target_fpm = vs

    def _manage_systems(self, alt, speed):
        """Very naive flap, gear and speedbrake scheduling."""
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

        # Automatically deploy the speedbrake when overspeeding
        if speed > 320:
            speedbrake = 1.0
        elif speed > 280:
            speedbrake = (speed - 280) / 40.0
        else:
            speedbrake = 0.0
        self.systems.set_targets(gear=gear_cmd, flap=flap, speedbrake=speedbrake)

        # Auto anti-ice when icing conditions are detected
        icing = self.environment.is_icing()
        self.anti_ice.set_active(icing)

    def update(self):
        f = self.fdm
        alt = f.get_property_value('position/h-sl-ft')
        psi = f.get_property_value('attitude/psi-deg')
        speed = f.get_property_value('velocities/vt-fps') / 1.68781
        vs = f.get_property_value('velocities/h-dot-fps')  # ft/s

        self._manage_systems(alt, speed)
        pressure, demand = self.systems.update(self.dt)

        powered = self.electrics.is_powered()

        # Altitude capture logic with a constant VS until near target
        alt_error = self.altitude - alt
        if abs(alt_error) > self.capture_window:
            vs_target_fpm = self.vs_target_fpm
        else:
            vs_target_fpm = self.alt_pid.update(alt_error, self.dt) * 60.0
            self.vs_target_fpm = vs_target_fpm
        vs_target_fpm = max(min(vs_target_fpm, 3000.0), -3000.0)
        vs_error = vs_target_fpm / 60.0 - vs
        pitch_cmd = max(min(self.vs_pid.update(vs_error, self.dt), 0.5), -0.5)
        if powered:
            f['fcs/elevator-cmd-norm'] = pitch_cmd * pressure
        else:
            f['fcs/elevator-cmd-norm'] = 0.0

        heading_error = (self.heading - psi + 180) % 360 - 180
        aileron_cmd = max(min(self.hdg_pid.update(heading_error, self.dt), 0.3), -0.3)
        if powered:
            f['fcs/aileron-cmd-norm'] = aileron_cmd * pressure
        else:
            f['fcs/aileron-cmd-norm'] = 0.0

        slip = f.get_property_value('aero/beta-rad')
        rudder_cmd = max(min(self.yaw_pid.update(-slip, self.dt), 0.3), -0.3)
        if powered:
            f['fcs/rudder-cmd-norm'] = rudder_cmd * pressure
        else:
            f['fcs/rudder-cmd-norm'] = 0.0

        throttle_cmd = self.autothrottle.update(self.dt, powered)
        active, ice = self.anti_ice.update(self.dt)

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
            active,
            ice,
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
        self.systems = SystemManager(self.fdm, failure_chance=1e-4)
        self.electrics = ElectricSystem()
        self.fuel = FuelSystem(self.fdm, self.engine, self.electrics)
        self.bleed = BleedAirSystem(self.engine, self.electrics)
        self.starter = EngineStartSystem(self.fdm, self.bleed)
        self.environment = Environment(self.fdm)
        self.anti_ice = AntiIceSystem(self.environment, self.engine, self.bleed)
        self.pressurization = PressurizationSystem(self.fdm, self.bleed)
        self.oxygen = OxygenSystem()
        self.stall_warning = StallWarningSystem(self.fdm)
        self.gpws = GroundProximityWarningSystem(self.fdm)
        self.overspeed = OverspeedWarningSystem(self.fdm)
        self.autopilot = Autopilot(
            self.fdm,
            dt,
            self.engine,
            self.systems,
            self.electrics,
            self.environment,
            self.anti_ice,
        )
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
        f['propulsion/engine/set-running'] = 0
        f['propulsion/engine[1]/set-running'] = 0
        f.run_ic()
        self.electrics.start_apu()
        self.starter.request_start()

    def step(self):
        dt = self.fdm.get_delta_t()
        self.environment.update(dt)
        self.starter.update(dt)
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
            anti_ice_on,
            ice_accum,
        ) = self.autopilot.update()
        elec = self.electrics.update(n1 > 0.5, hyd_demand + 0.1, dt)
        cabin_alt, cabin_diff, bleed_press = self.pressurization.update(dt)
        fuel_data = self.fuel.update()
        oxygen = self.oxygen.update(cabin_alt, dt)
        stall = self.stall_warning.update()
        gpws = self.gpws.update()
        overspeed = self.overspeed.update()
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
            'anti_ice_on': anti_ice_on,
            'ice_accum': ice_accum,
            'cabin_altitude_ft': cabin_alt,
            'cabin_diff_psi': cabin_diff,
            'bleed_press': bleed_press,
            'fuel_lbs': fuel,
            'fuel_flow_lbs_hr_eng1': fuel_data['flow0_pph'],
            'fuel_flow_lbs_hr_eng2': fuel_data['flow1_pph'],
            'apu_flow_lbs_hr': fuel_data['apu_flow_pph'],
            'crossfeed': fuel_data['crossfeed'],
            'oxygen_level': oxygen,
            'stall_warning': stall,
            'gpws_warning': gpws,
            'overspeed_warning': overspeed,
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
                    f"cabin={data['cabin_altitude_ft']:.0f}ft "
                    f"bleed={data['bleed_press']:.2f} "
                    f"ff={data['fuel_flow_lbs_hr_eng1']:.0f}/{data['fuel_flow_lbs_hr_eng2']:.0f} pph "
                    f"apu={data['apu_flow_lbs_hr']:.0f}pph "
                    f"xfeed={'ON' if data['crossfeed'] else 'OFF'} "
                    f"oxy={data['oxygen_level']:.2f} "
                    f"ice={data['ice_accum']:.2f} {'ON' if data['anti_ice_on'] else 'OFF'} "
                    f"stall={'YES' if data['stall_warning'] else 'NO'} "
                    f"gpws={'YES' if data['gpws_warning'] else 'NO'} "
                    f"os={'YES' if data['overspeed_warning'] else 'NO'}"
                )
            time.sleep(self.fdm.get_delta_t())

if __name__ == '__main__':
    sim = A320IFRSim()
    sim.run(300)

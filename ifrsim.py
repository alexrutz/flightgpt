import jsbsim
import time
import math
import random
from tcas import TCASSystem


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
    """Model a single engine with basic failures and dynamics."""

    def __init__(
        self,
        fdm,
        index: int,
        spool_rate=0.2,
        failure_chance=0.0,
        oil=None,
        fire_chance=0.0,
    ):
        self.fdm = fdm
        self.index = index
        self.spool_rate = spool_rate
        self.failure_chance = failure_chance
        self.throttle = 0.0
        self.target = 0.0
        self.efficiency = 1.0
        self.extra_fuel_factor = 0.0
        self.failed = False
        self.oil = oil or OilSystem()
        self.oil_timer = 0.0
        self.fire = False
        self.fire_timer = 0.0
        self.fire_chance = fire_chance
        self.egt = 0.4
        self.egt_timer = 0.0
        self.egt_rise_rate = 0.5
        self.egt_cool_rate = 0.2

    def _prop(self, name: str) -> str:
        if self.index == 0:
            return f"propulsion/engine/{name}"
        return f"propulsion/engine[{self.index}]/{name}"

    def _fcs(self, name: str) -> str:
        if self.index == 0:
            return f"fcs/{name}"
        return f"fcs/{name}[{self.index}]"

    def fail(self) -> None:
        """Fail this engine."""
        self.failed = True
        self.fdm[self._prop("set-running")] = 0

    def restart(self) -> None:
        """Clear failure flag once the engine is running again."""
        self.failed = False

    def set_target(self, throttle: float) -> None:
        """Set desired throttle position."""
        self.target = max(0.0, min(throttle, 1.0))

    def update(self, dt: float) -> None:
        """Update engine throttle command with a simple non-linear lag."""
        if not self.failed:
            diff = self.target - self.throttle
            rate = self.spool_rate * (1.0 + 2.0 * abs(diff))
            max_delta = rate * dt
            diff = max(min(diff, max_delta), -max_delta)
            self.throttle += diff
            if random.random() < self.failure_chance * dt:
                self.fail()
        else:
            self.throttle = 0.0

        cmd = self.throttle * self.efficiency
        self.fdm[self._fcs("throttle-cmd-norm")] = cmd

        # Simple exhaust temperature model with overheat failure
        self.egt += (self.throttle * self.egt_rise_rate - self.egt_cool_rate) * dt
        self.egt = max(0.0, min(self.egt, 1.5))
        if self.egt > 1.2:
            self.egt_timer += dt
        else:
            self.egt_timer = 0.0
        if self.egt_timer > 5.0:
            self.fail()

        oil_p, oil_t = self.oil.update(self.throttle, dt)
        if oil_p < 0.2 or oil_t > 1.2:
            self.oil_timer += dt
        else:
            self.oil_timer = 0.0
        if self.oil_timer > 5.0:
            self.fail()

        if not self.fire and random.random() < self.fire_chance * dt:
            self.fire = True
        if self.fire:
            self.fire_timer += dt
            if self.fire_timer > 10.0:
                self.fail()
        else:
            self.fire_timer = 0.0

    def n1(self) -> float:
        return self.fdm.get_property_value(self._prop("n1"))

    def oil_pressure(self) -> float:
        return self.oil.pressure

    def oil_temperature(self) -> float:
        return self.oil.temperature

    def exhaust_temperature(self) -> float:
        return self.egt

    def extinguish_fire(self) -> None:
        self.fire = False
        self.fire_timer = 0.0


class EngineSystem:
    """Manage a set of engines as a single unit."""

    def __init__(self, engines: list[Engine]):
        self.engines = engines

    @property
    def throttle(self) -> float:
        if not self.engines:
            return 0.0
        return sum(e.throttle for e in self.engines) / len(self.engines)

    @property
    def extra_fuel_factor(self) -> float:
        if not self.engines:
            return 0.0
        return sum(e.extra_fuel_factor for e in self.engines) / len(self.engines)

    @extra_fuel_factor.setter
    def extra_fuel_factor(self, value: float) -> None:
        for e in self.engines:
            e.extra_fuel_factor = value

    @property
    def efficiency(self) -> float:
        if not self.engines:
            return 1.0
        return sum(e.efficiency for e in self.engines) / len(self.engines)

    @efficiency.setter
    def efficiency(self, value: float) -> None:
        for e in self.engines:
            e.efficiency = value

    def set_target(self, throttle: float) -> None:
        for e in self.engines:
            e.set_target(throttle)

    def update(self, dt: float) -> None:
        for e in self.engines:
            e.update(dt)

    def n1(self) -> float:
        if not self.engines:
            return 0.0
        return sum(e.n1() for e in self.engines) / len(self.engines)

    def n1_list(self) -> list[float]:
        return [e.n1() for e in self.engines]

    def oil_pressure(self) -> float:
        if not self.engines:
            return 0.0
        return sum(e.oil_pressure() for e in self.engines) / len(self.engines)

    def oil_temperature(self) -> float:
        if not self.engines:
            return 0.0
        return sum(e.oil_temperature() for e in self.engines) / len(self.engines)

    def exhaust_temperature(self) -> float:
        if not self.engines:
            return 0.0
        return sum(e.exhaust_temperature() for e in self.engines) / len(self.engines)

    def egt_list(self) -> list[float]:
        return [e.exhaust_temperature() for e in self.engines]

    def fail(self, index: int | None = None) -> None:
        if index is None:
            for e in self.engines:
                e.fail()
        elif 0 <= index < len(self.engines):
            self.engines[index].fail()

    def restart(self, index: int | None = None) -> None:
        if index is None:
            for e in self.engines:
                e.restart()
        elif 0 <= index < len(self.engines):
            self.engines[index].restart()

    @property
    def fire(self) -> bool:
        return any(e.fire for e in self.engines)


class Autothrottle:
    """Maintain target airspeed by commanding engine thrust."""

    def __init__(self, fdm, engine: EngineSystem, kp=0.01, ki=0.0, kd=0.0, pitot=None):
        self.fdm = fdm
        self.engine = engine
        self.pid = PIDController(kp, ki, kd)
        self.target_speed = 0.0
        self.pitot = pitot
        self.engaged = True

    def engage(self) -> None:
        """Activate autothrottle control."""
        self.engaged = True

    def disengage(self) -> None:
        """Deactivate autothrottle control."""
        self.engaged = False

    def set_target(self, speed_kt: float) -> None:
        """Set desired airspeed in knots."""
        self.target_speed = speed_kt

    def update(self, dt: float, powered: bool = True) -> float:
        """Update engine throttle to hold the target speed."""
        if not self.engaged or not powered:
            # When disengaged simply update the engine toward its current
            # target without changing it so manual thrust settings remain.
            self.engine.update(dt)
            return self.engine.throttle

        if self.pitot is not None:
            speed = self.pitot.indicated_speed(self.fdm)
        else:
            speed = self.fdm.get_property_value("velocities/vt-fps") / 1.68781
        throttle_cmd = self.pid.update(self.target_speed - speed, dt)
        throttle_cmd = max(0.0, min(throttle_cmd, 1.0))
        self.engine.set_target(throttle_cmd)
        self.engine.update(dt)
        return self.engine.throttle


class HydraulicSystem:
    """Very small hydraulic system model with basic failures."""

    def __init__(
        self, pump_rate=0.3, usage_factor=0.5, leak_rate=0.01, failure_chance=0.0
    ):
        self.pressure = 1.0
        self.pump_rate = pump_rate
        self.usage_factor = usage_factor
        self.leak_rate = leak_rate
        self.failure_chance = failure_chance
        self.pump_on = True

    def update(self, demand: float, dt: float, pump_power: bool = True) -> float:
        if self.pump_on and pump_power:
            if random.random() < self.failure_chance * dt:
                self.pump_on = False
            else:
                self.pressure += self.pump_rate * dt
        self.pressure -= (self.leak_rate + demand * self.usage_factor) * dt
        self.pressure = max(0.0, min(self.pressure, 1.0))
        return self.pressure


class BrakeSystem:
    """Very small wheel brake model with heat buildup."""

    def __init__(self, heat_rate=0.3, cool_rate=0.05):
        self.heat = 0.0
        self.heat_rate = heat_rate
        self.cool_rate = cool_rate
        self.command = 0.0
        self.parking_brake = False

    def set_command(self, cmd: float) -> None:
        self.command = max(0.0, min(cmd, 1.0))

    def set_parking_brake(self, on: bool) -> None:
        """Engage or release the parking brake."""
        self.parking_brake = bool(on)

    def update(self, on_ground: bool, dt: float) -> tuple[float, float]:
        if not on_ground and not self.parking_brake:
            self.command = 0.0
        applied = max(self.command, 1.0 if self.parking_brake else 0.0)
        self.heat += self.heat_rate * applied * dt
        self.heat -= self.cool_rate * dt
        self.heat = max(0.0, min(self.heat, 1.0))
        efficiency = 1.0 - 0.5 * self.heat
        return efficiency, self.heat


class AutobrakeSystem:
    """Simple autobrake logic with selectable levels."""

    LEVELS = {
        "off": 0.0,
        "low": 0.3,
        "med": 0.6,
        "high": 1.0,
    }

    def __init__(
        self,
        brakes: BrakeSystem,
        level: str = "off",
        arm_speed_kt: float = 70.0,
        disarm_speed_kt: float = 20.0,
        throttle_threshold: float = 0.3,
    ):
        self.brakes = brakes
        self.level = level if level in self.LEVELS else "off"
        self.arm_speed = arm_speed_kt
        self.disarm_speed = disarm_speed_kt
        self.throttle_threshold = throttle_threshold
        self.active = False

    def set_level(self, level: str) -> None:
        if level in self.LEVELS:
            self.level = level
            self.active = False

    def update(self, on_ground: bool, speed_kt: float, throttle: float) -> bool:
        if self.level == "off":
            self.active = False
        else:
            if (
                on_ground
                and speed_kt > self.arm_speed
                and throttle < self.throttle_threshold
            ):
                self.active = True
            if (
                not on_ground
                or throttle >= self.throttle_threshold
                or speed_kt < self.disarm_speed
            ):
                self.active = False

        if self.active:
            self.brakes.set_command(self.LEVELS[self.level])
        return self.active


class OilSystem:
    """Track basic oil pressure and temperature."""

    def __init__(
        self,
        pump_rate=0.5,
        leak_rate=0.02,
        heat_rate=0.3,
        cool_rate=0.1,
        failure_chance=0.0,
    ):
        self.pressure = 1.0
        self.temperature = 0.2
        self.pump_rate = pump_rate
        self.leak_rate = leak_rate
        self.heat_rate = heat_rate
        self.cool_rate = cool_rate
        self.failure_chance = failure_chance
        self.pump_on = True

    def update(self, throttle: float, dt: float) -> tuple[float, float]:
        if self.pump_on:
            if random.random() < self.failure_chance * dt:
                self.pump_on = False
            else:
                self.pressure += self.pump_rate * throttle * dt
        self.pressure -= self.leak_rate * dt
        self.pressure = max(0.0, min(self.pressure, 1.0))

        self.temperature += self.heat_rate * throttle * dt
        self.temperature -= self.cool_rate * dt
        self.temperature = max(0.0, self.temperature)
        return self.pressure, self.temperature


class SystemManager:
    """Handles slow actuation of gear, flaps and speedbrake with hydraulics.

    Flaps and gear may become inoperable if moved above their respective
    overspeed limits to model structural damage.
    """

    def __init__(
        self,
        fdm,
        flap_rate=0.5,
        gear_rate=0.5,
        speedbrake_rate=0.5,
        failure_chance=0.0,
        flap_overspeed_kt=220.0,
        gear_overspeed_kt=250.0,
    ):
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
        self.flap_overspeed_kt = flap_overspeed_kt
        self.gear_overspeed_kt = gear_overspeed_kt
        self.flap_operable = True
        self.gear_operable = True

    def set_targets(self, gear=None, flap=None, speedbrake=None):
        if gear is not None:
            self.target_gear = max(0.0, min(gear, 1.0))
        if flap is not None:
            self.target_flap = max(0.0, min(flap, 1.0))
        if speedbrake is not None:
            self.target_speedbrake = max(0.0, min(speedbrake, 1.0))

    def update(self, dt, pump_power: bool = True, speed_kt: float = 0.0):
        d_flap = self.target_flap - self.flap
        d_gear = self.target_gear - self.gear
        d_speedbrake = self.target_speedbrake - self.speedbrake
        demand = abs(d_flap) + abs(d_gear) + abs(d_speedbrake)
        pressure = self.hydraulics.update(demand, dt, pump_power)

        if self.flap_operable:
            if speed_kt > self.flap_overspeed_kt and abs(d_flap) > 0.0:
                self.flap_operable = False
            else:
                max_df = self.flap_rate * dt * pressure
                d_flap = max(min(d_flap, max_df), -max_df)
                self.flap += d_flap
        self.fdm["fcs/flap-cmd-norm"] = self.flap

        if self.gear_operable:
            if speed_kt > self.gear_overspeed_kt and abs(d_gear) > 0.0:
                self.gear_operable = False
            else:
                max_dg = self.gear_rate * dt * pressure
                d_gear = max(min(d_gear, max_dg), -max_dg)
                self.gear += d_gear
        self.fdm["gear/gear-cmd-norm"] = self.gear

        max_ds = self.speedbrake_rate * dt * pressure
        d_speedbrake = max(min(d_speedbrake, max_ds), -max_ds)
        self.speedbrake += d_speedbrake
        self.fdm["fcs/speedbrake-cmd-norm"] = self.speedbrake

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
        engine: EngineSystem | None = None,
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
        self.prev_used_0 = fdm.get_property_value("propulsion/engine/fuel-used-lbs")
        self.prev_used_1 = fdm.get_property_value("propulsion/engine[1]/fuel-used-lbs")

    def update(self):
        dt = self.fdm.get_delta_t()
        used_0 = self.fdm.get_property_value("propulsion/engine/fuel-used-lbs")
        used_1 = self.fdm.get_property_value("propulsion/engine[1]/fuel-used-lbs")
        flow_0 = (used_0 - self.prev_used_0) / dt * 3600.0
        flow_1 = (used_1 - self.prev_used_1) / dt * 3600.0
        if self.engine is not None:
            flow_0 *= 1.0 + self.engine.extra_fuel_factor
            flow_1 *= 1.0 + self.engine.extra_fuel_factor
        self.prev_used_0 = used_0
        self.prev_used_1 = used_1

        left = self.fdm.get_property_value("propulsion/tank/contents-lbs")
        right = self.fdm.get_property_value("propulsion/tank[1]/contents-lbs")

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
            self.fdm["propulsion/tank/contents-lbs"] = left
            self.fdm["propulsion/tank[1]/contents-lbs"] = right

        apu_flow = 0.0
        if self.electrics and self.electrics.apu_running:
            apu_flow = self.apu_flow_pph
            burn = apu_flow / 3600.0 * dt
            left = max(left - burn, 0.0)
            self.fdm["propulsion/tank/contents-lbs"] = left

        total = left + right
        return {
            "left_lbs": left,
            "right_lbs": right,
            "total_lbs": total,
            "flow0_pph": flow_0,
            "flow1_pph": flow_1,
            "apu_flow_pph": apu_flow,
            "crossfeed": self.crossfeed_on,
        }


class RamAirTurbine:
    """Provide emergency electrical power when deployed."""

    def __init__(
        self,
        deploy_threshold=0.1,
        retract_threshold=0.6,
        charge_rate=0.05,
    ):
        self.deploy_threshold = deploy_threshold
        self.retract_threshold = retract_threshold
        self.charge_rate = charge_rate
        self.deployed = False

    def update(self, electrics, generator_available: bool, dt: float) -> bool:
        if (
            not self.deployed
            and electrics.charge < self.deploy_threshold
            and not generator_available
        ):
            self.deployed = True
        if self.deployed and (
            electrics.charge > self.retract_threshold or generator_available
        ):
            self.deployed = False
        if self.deployed:
            electrics.charge = min(1.0, electrics.charge + self.charge_rate * dt)
        return self.deployed


class ElectricSystem:
    """Electrical system with battery, generator and a simple APU."""

    def __init__(
        self,
        charge_rate=0.2,
        discharge_rate=0.05,
        apu_charge_rate=0.1,
        apu_start_time=5.0,
        generator_failure_chance=0.0,
        rat=None,
    ):
        self.charge = 1.0
        self.charge_rate = charge_rate
        self.discharge_rate = discharge_rate
        self.apu_charge_rate = apu_charge_rate
        self.apu_start_time = apu_start_time
        self.apu_running = False
        self.apu_timer = 0.0
        self.generator_failure_chance = generator_failure_chance
        self.generator_failed = False
        self.rat = rat

    def start_apu(self) -> None:
        if not self.apu_running:
            self.apu_running = True
            self.apu_timer = 0.0

    def stop_apu(self) -> None:
        self.apu_running = False

    def update(self, generator_on: bool, demand: float, dt: float) -> float:
        if generator_on and not self.generator_failed:
            if random.random() < self.generator_failure_chance * dt:
                self.generator_failed = True
            else:
                self.charge += self.charge_rate * dt
        elif self.apu_running:
            # Allow a small delay before the APU provides power
            self.apu_timer += dt
            if self.apu_timer >= self.apu_start_time:
                self.charge += self.apu_charge_rate * dt

        self.charge -= demand * self.discharge_rate * dt
        self.charge = max(0.0, min(self.charge, 1.0))

        # Automatically start the APU if the battery is low and the
        # generator is not running or has failed. Stop it once the battery
        # has recovered and the generator is available again.
        gen_available = generator_on and not self.generator_failed
        if self.charge < 0.3 and not gen_available:
            self.start_apu()
        if self.charge > 0.95 and self.apu_running and gen_available:
            self.stop_apu()

        if self.rat is not None:
            self.rat.update(self, gen_available, dt)

        return self.charge

    def is_powered(self) -> bool:
        return self.charge > 0.05

    def rat_deployed(self) -> bool:
        return self.rat.deployed if self.rat is not None else False


class BleedAirSystem:
    """Provide bleed air from the engines or APU."""

    def __init__(self, engine: EngineSystem, electrics, engine_gain=1.0, apu_gain=0.5):
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

    def __init__(self, fdm, bleed, engines: EngineSystem | None = None, start_time=8.0):
        self.fdm = fdm
        self.bleed = bleed
        self.engines = engines
        self.start_time = start_time
        self.timer = 0.0
        self.state = "stopped"

    def request_start(self) -> None:
        if self.state == "stopped":
            self.state = "starting"
            self.timer = 0.0
            if self.engines is not None:
                for i, _ in enumerate(self.engines.engines):
                    key = (
                        "propulsion/engine/set-running"
                        if i == 0
                        else f"propulsion/engine[{i}]/set-running"
                    )
                    self.fdm[key] = 0

    def update(self, dt: float) -> bool:
        if self.state == "starting":
            if self.bleed.pressure > 0.3:
                self.timer += dt
            if self.timer >= self.start_time:
                if self.engines is not None:
                    for i, eng in enumerate(self.engines.engines):
                        key = (
                            "propulsion/engine/set-running"
                            if i == 0
                            else f"propulsion/engine[{i}]/set-running"
                        )
                        self.fdm[key] = 1
                        eng.restart()
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
        self.fdm["atmosphere/wind-north-fps"] = 0.0
        self.fdm["atmosphere/wind-east-fps"] = base + gust

        vert_base = 2.0 * math.sin(self.t / 40.0)
        vert_gust = self.vertical_strength * math.sin(self.t * 12.0)
        vert_gust += (
            random.uniform(-self.vertical_strength, self.vertical_strength) * 0.1
        )
        self.fdm["atmosphere/wind-down-fps"] = vert_base + vert_gust

        alt = self.fdm.get_property_value("position/h-sl-ft")
        self.temperature_c = self.base_temp - 0.002 * alt
        if random.random() < 0.01:
            self.precip = random.uniform(0.2, 1.0)
        self.precip *= 0.99

    def is_icing(self):
        return self.temperature_c < 2.0 and self.precip > 0.3


class AntiIceSystem:
    """Minimal anti-ice system that counters engine icing."""

    def __init__(
        self,
        environment,
        engine: EngineSystem,
        bleed=None,
        melt_rate=0.05,
        acc_rate=0.1,
        failure_chance=0.0,
    ):
        self.environment = environment
        self.engine = engine
        self.bleed = bleed
        self.melt_rate = melt_rate
        self.acc_rate = acc_rate
        self.failure_chance = failure_chance
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
        if self.ice > 0.9 and random.random() < self.failure_chance * dt:
            self.engine.fail()
        return self.active, self.ice


class WingIceSystem:
    """Track wing icing and allow a simple anti-ice function."""

    def __init__(
        self,
        environment,
        bleed=None,
        melt_rate=0.05,
        acc_rate=0.1,
    ):
        self.environment = environment
        self.bleed = bleed
        self.melt_rate = melt_rate
        self.acc_rate = acc_rate
        self.active = False
        self.ice = 0.0

    def set_active(self, state: bool) -> None:
        self.active = state

    def update(self, dt: float) -> tuple[bool, float]:
        if self.environment.is_icing() and not self.active:
            self.ice += self.acc_rate * dt
        else:
            melt = self.melt_rate
            if self.active and self.bleed is not None:
                melt *= self.bleed.pressure
            self.ice -= melt * dt
        self.ice = max(0.0, min(1.0, self.ice))
        return self.active, self.ice


class PitotSystem:
    """Simulate pitot tube icing and heating."""

    def __init__(self, environment, heat_on=True, clog_rate=0.1, clear_rate=0.05):
        self.environment = environment
        self.heat_on = heat_on
        self.clog_rate = clog_rate
        self.clear_rate = clear_rate
        self.clog = 0.0

    def set_heat(self, state: bool) -> None:
        self.heat_on = state

    def update(self, dt: float) -> float:
        if self.environment.is_icing() and not self.heat_on:
            self.clog += self.clog_rate * dt
        else:
            self.clog -= self.clear_rate * dt
        self.clog = max(0.0, min(self.clog, 1.0))
        return self.clog

    def indicated_speed(self, fdm) -> float:
        speed = fdm.get_property_value("velocities/vt-fps") / 1.68781
        return speed * (1.0 - 0.5 * self.clog)


class PressurizationSystem:
    """Track cabin altitude based on a simple pressurization model."""

    def __init__(self, fdm, bleed=None, target_diff=8.0, leak_rate=0.001):
        self.fdm = fdm
        self.bleed = bleed
        self.target_diff = target_diff
        self.leak_rate = leak_rate
        self.cabin_alt = fdm.get_property_value("position/h-sl-ft")

    def _pressure_at_alt(self, alt):
        return 14.7 * math.exp(-alt / 20000.0)

    def update(self, dt):
        alt = self.fdm.get_property_value("position/h-sl-ft")
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


class CabinTemperatureSystem:
    """Simple cabin temperature model using bleed air."""

    def __init__(
        self,
        environment,
        bleed=None,
        target_temp_c=21.0,
        leak_rate=0.002,
        control_gain=0.1,
    ):
        self.environment = environment
        self.bleed = bleed
        self.target_temp = target_temp_c
        self.leak_rate = leak_rate
        self.control_gain = control_gain
        self.cabin_temp = target_temp_c

    def update(self, dt: float) -> float:
        outside = self.environment.temperature_c
        bleed_temp = outside
        if self.bleed is not None:
            bleed_temp += 100.0 * self.bleed.pressure
        # heat/cool towards bleed temperature
        self.cabin_temp += (bleed_temp - self.cabin_temp) * self.control_gain * dt
        # leakage towards outside temperature
        self.cabin_temp += (outside - self.cabin_temp) * self.leak_rate * dt
        return self.cabin_temp


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

    def __init__(self, fdm, alpha_deg=12.0, speed_kt=120.0, wing_ice=None):
        self.fdm = fdm
        self.alpha_thresh = math.radians(alpha_deg)
        self.speed_thresh = speed_kt
        self.wing_ice = wing_ice

    def update(self):
        alpha = self.fdm.get_property_value("aero/alpha-rad")
        speed = self.fdm.get_property_value("velocities/vt-fps") / 1.68781
        speed_thresh = self.speed_thresh
        if self.wing_ice is not None:
            speed_thresh += 40.0 * self.wing_ice.ice
        return alpha > self.alpha_thresh or speed < speed_thresh


class GroundProximityWarningSystem:
    """Warn when approaching the ground with a high descent rate."""

    def __init__(self, fdm, alt_ft=200.0, sink_rate_fpm=1500.0):
        self.fdm = fdm
        self.alt_thresh = alt_ft
        self.sink_thresh = -abs(sink_rate_fpm)

    def update(self):
        agl = self.fdm.get_property_value("position/h-agl-ft")
        vs_fpm = self.fdm.get_property_value("velocities/h-dot-fps") * 60.0
        return agl < self.alt_thresh and vs_fpm < self.sink_thresh


class OverspeedWarningSystem:
    """Warn when exceeding a structural speed limit."""

    def __init__(self, fdm, limit_kt=320.0):
        self.fdm = fdm
        self.limit = limit_kt

    def update(self):
        speed = self.fdm.get_property_value("velocities/vt-fps") / 1.68781
        return speed > self.limit


class WeatherRadarSystem:
    """Detect heavy precipitation along the flight path."""

    def __init__(self, environment, threshold=0.5):
        self.environment = environment
        self.threshold = threshold

    def update(self) -> bool:
        """Return True when precipitation intensity exceeds the threshold."""
        return self.environment.precip >= self.threshold


class FireSuppressionSystem:
    """Detect and extinguish engine fires with limited bottles."""

    def __init__(self, engines: EngineSystem, bottles=2):
        self.engines = engines
        self.bottles = bottles
        self.active = False
        self.timer = 0.0
        self.active_engine: int | None = None

    def update(self, dt):
        if self.active_engine is None:
            for i, eng in enumerate(self.engines.engines):
                if eng.fire and self.bottles > 0:
                    self.bottles -= 1
                    self.active_engine = i
                    self.timer = 0.0
                    break
        if self.active_engine is not None:
            self.timer += dt
            if self.timer > 1.0:
                self.engines.engines[self.active_engine].extinguish_fire()
                self.active_engine = None
        return self.engines.fire

    def bottles_left(self) -> int:
        return self.bottles


class MasterCautionSystem:
    """Aggregate warnings from multiple subsystems."""

    def __init__(self):
        self.warnings: dict[str, bool] = {}

    def set_warning(self, name: str, state: bool) -> None:
        """Register or update a warning condition by name."""
        self.warnings[name] = state

    def is_active(self) -> bool:
        """Return True if any registered warning is active."""
        return any(self.warnings.values())

    def reset(self) -> None:
        """Clear all stored warnings."""
        self.warnings.clear()


class NavigationSystem:
    """Very small waypoint-based navigation model."""

    def __init__(self, fdm, waypoints=None):
        self.fdm = fdm
        self.waypoints = waypoints or []
        self.index = 0

    def add_waypoint(self, lat_deg, lon_deg, alt_ft=None):
        """Append a new waypoint to the route."""
        self.waypoints.append((lat_deg, lon_deg, alt_ft))

    def _bearing_distance(self, lat1, lon1, lat2, lon2):
        lat1 = math.radians(lat1)
        lon1 = math.radians(lon1)
        lat2 = math.radians(lat2)
        lon2 = math.radians(lon2)
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = (
            math.sin(dlat / 2) ** 2
            + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
        )
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        dist_nm = 3440.065 * c
        y = math.sin(dlon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(
            lat2
        ) * math.cos(dlon)
        bearing = (math.degrees(math.atan2(y, x)) + 360) % 360
        return bearing, dist_nm

    def update(self):
        if not self.waypoints or self.index >= len(self.waypoints):
            return None, None, None
        lat = self.fdm.get_property_value("position/lat-gc-deg")
        lon = self.fdm.get_property_value("position/long-gc-deg")
        tgt_lat, tgt_lon, tgt_alt = self.waypoints[self.index]
        bearing, dist = self._bearing_distance(lat, lon, tgt_lat, tgt_lon)
        if dist < 0.3 and self.index < len(self.waypoints) - 1:
            self.index += 1
            tgt_lat, tgt_lon, tgt_alt = self.waypoints[self.index]
        bearing, dist = self._bearing_distance(lat, lon, tgt_lat, tgt_lon)
        return bearing, dist, tgt_alt


class ILSSystem:
    """Very small ILS model providing localizer and glideslope deviation."""

    def __init__(
        self,
        fdm,
        runway_lat_deg,
        runway_lon_deg,
        runway_hdg_deg,
        runway_alt_ft=0.0,
        gs_deg=3.0,
        range_nm=10.0,
    ):
        self.fdm = fdm
        self.lat = runway_lat_deg
        self.lon = runway_lon_deg
        self.hdg = runway_hdg_deg
        self.alt = runway_alt_ft
        self.gs = gs_deg
        self.range = range_nm

    def _bearing_distance(self, lat1, lon1, lat2, lon2):
        lat1 = math.radians(lat1)
        lon1 = math.radians(lon1)
        lat2 = math.radians(lat2)
        lon2 = math.radians(lon2)
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = (
            math.sin(dlat / 2) ** 2
            + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
        )
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        dist_nm = 3440.065 * c
        y = math.sin(dlon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(
            lat2
        ) * math.cos(dlon)
        bearing = (math.degrees(math.atan2(y, x)) + 360) % 360
        return bearing, dist_nm

    def update(self):
        lat = self.fdm.get_property_value("position/lat-gc-deg")
        lon = self.fdm.get_property_value("position/long-gc-deg")
        alt = self.fdm.get_property_value("position/h-sl-ft")
        bearing, dist = self._bearing_distance(lat, lon, self.lat, self.lon)
        dev = (bearing - self.hdg + 180) % 360 - 180
        target_alt = self.alt + math.tan(math.radians(self.gs)) * dist * 6076.12
        gs_dev = alt - target_alt
        if dist > self.range:
            return None, None, dist
        return dev, gs_dev, dist


class Autopilot:
    """Heading, altitude and speed hold with basic system automation.

    A small vertical navigation mode adjusts the target climb or descent
    rate so that altitude constraints at the active waypoint are met
    whenever the navigation system provides both a distance and an
    altitude."""

    def __init__(
        self,
        fdm,
        dt,
        engine,
        systems,
        electrics,
        environment,
        anti_ice,
        wing_ice=None,
        brakes=None,
        nav=None,
        autobrake=None,
        ils=None,
        pitot=None,
        capture_window=200.0,
        climb_vs_fpm=1500.0,
        descent_vs_fpm=-1500.0,
        auto_manage_systems=True,
    ):
        self.fdm = fdm
        self.dt = dt
        self.engine = engine
        self.systems = systems
        self.electrics = electrics
        self.environment = environment
        self.anti_ice = anti_ice
        self.wing_ice = wing_ice
        self.brakes = brakes
        self.nav = nav
        self.autobrake = autobrake
        self.ils = ils
        self.pitot = pitot
        self.last_autobrake_active = False
        self.autothrottle = Autothrottle(fdm, engine, pitot=pitot)
        self.engaged = True
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
        self.auto_manage_systems = auto_manage_systems
        self.vertical_mode = "VS"

    def engage(self) -> None:
        """Activate the autopilot."""
        self.engaged = True

    def disengage(self) -> None:
        """Deactivate the autopilot and release controls."""
        self.engaged = False

    def set_system_automation(self, enabled: bool) -> None:
        """Enable or disable automatic gear and flap management."""
        self.auto_manage_systems = enabled

    def set_targets(self, altitude=None, heading=None, speed=None, vs=None):
        if altitude is not None:
            if altitude != self.altitude:
                self.vs_target_fpm = (
                    self.climb_vs_fpm
                    if altitude > self.altitude
                    else self.descent_vs_fpm
                )
            self.altitude = altitude
        if heading is not None:
            self.heading = heading
        if speed is not None:
            self.speed = speed
            self.autothrottle.set_target(speed)
        if vs is not None:
            self.vs_target_fpm = vs

    def _manage_systems(self, alt, speed, on_ground):
        """Very naive flap, gear and speedbrake scheduling."""
        if not self.auto_manage_systems:
            return
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

        if self.brakes is not None:
            autobrake_active = False
            if self.autobrake is not None:
                autobrake_active = self.autobrake.update(
                    on_ground, speed, self.engine.throttle
                )
            if not autobrake_active:
                if on_ground and speed > 30:
                    self.brakes.set_command(min((speed - 30) / 40.0, 1.0))
                else:
                    self.brakes.set_command(0.0)
            self.last_autobrake_active = autobrake_active

        # Auto anti-ice when icing conditions are detected
        icing = self.environment.is_icing()
        self.anti_ice.set_active(icing)
        if self.wing_ice is not None:
            self.wing_ice.set_active(icing)
        if self.pitot is not None:
            self.pitot.set_heat(icing)

    def update(self):
        f = self.fdm
        if self.pitot is not None:
            self.pitot.update(self.dt)
        alt = f.get_property_value("position/h-sl-ft")
        psi = f.get_property_value("attitude/psi-deg")
        if self.pitot is not None:
            speed = self.pitot.indicated_speed(f)
        else:
            speed = f.get_property_value("velocities/vt-fps") / 1.68781
        vs = f.get_property_value("velocities/h-dot-fps")  # ft/s
        vertical_mode = "VS"

        loc_dev = None
        gs_dev = None
        ils_dist = None
        if self.ils is not None:
            loc_dev, gs_dev, ils_dist = self.ils.update()
            if loc_dev is not None:
                self.heading = self.ils.hdg + loc_dev
                self.altitude = alt - gs_dev
                vertical_mode = "APP"
            else:
                vertical_mode = "VS"
        
        nav_dist = None
        if self.nav is not None:
            nav_bearing, nav_dist, nav_alt = self.nav.update()
            if nav_bearing is not None:
                self.heading = nav_bearing
            if nav_alt is not None:
                self.altitude = nav_alt
            if (
                nav_alt is not None
                and nav_dist is not None
                and nav_dist > 0.1
                and speed > 30.0
            ):
                alt_error_wp = nav_alt - alt
                vs_wp = alt_error_wp * speed / (nav_dist * 60.0)
                self.vs_target_fpm = min(
                    self.climb_vs_fpm,
                    max(self.descent_vs_fpm, vs_wp),
                )
                vertical_mode = "VNAV"

        agl = f.get_property_value("position/h-agl-ft")
        on_ground = agl < 5.0
        self._manage_systems(alt, speed, on_ground)
        pump_power = self.engine.n1() > 0.2 or self.electrics.apu_running
        pressure, demand = self.systems.update(self.dt, pump_power, speed)
        brake_temp = 0.0
        if self.brakes is not None:
            _, brake_temp = self.brakes.update(on_ground, self.dt)

        powered = self.electrics.is_powered()

        # Altitude capture logic with a constant VS until near target
        alt_error = self.altitude - alt
        if abs(alt_error) > self.capture_window:
            vs_target_fpm = self.vs_target_fpm
        else:
            vs_target_fpm = self.alt_pid.update(alt_error, self.dt) * 60.0
            self.vs_target_fpm = vs_target_fpm
            vertical_mode = "ALT"
        vs_target_fpm = max(min(vs_target_fpm, 3000.0), -3000.0)
        vs_error = vs_target_fpm / 60.0 - vs
        pitch_cmd = 0.0
        aileron_cmd = 0.0
        rudder_cmd = 0.0
        if self.engaged:
            pitch_cmd = max(min(self.vs_pid.update(vs_error, self.dt), 0.5), -0.5)
            if powered:
                f["fcs/elevator-cmd-norm"] = pitch_cmd * pressure
            else:
                f["fcs/elevator-cmd-norm"] = 0.0

            heading_error = (self.heading - psi + 180) % 360 - 180
            aileron_cmd = max(
                min(self.hdg_pid.update(heading_error, self.dt), 0.3), -0.3
            )
            if powered:
                f["fcs/aileron-cmd-norm"] = aileron_cmd * pressure
            else:
                f["fcs/aileron-cmd-norm"] = 0.0

            slip = f.get_property_value("aero/beta-rad")
            rudder_cmd = max(min(self.yaw_pid.update(-slip, self.dt), 0.3), -0.3)
            if powered:
                f["fcs/rudder-cmd-norm"] = rudder_cmd * pressure
            else:
                f["fcs/rudder-cmd-norm"] = 0.0
        else:
            if powered:
                f["fcs/elevator-cmd-norm"] = 0.0
                f["fcs/aileron-cmd-norm"] = 0.0
                f["fcs/rudder-cmd-norm"] = 0.0

        throttle_cmd = self.autothrottle.update(self.dt, powered)
        active, ice = self.anti_ice.update(self.dt)
        wing_active = False
        wing_ice = 0.0
        if self.wing_ice is not None:
            wing_active, wing_ice = self.wing_ice.update(self.dt)

        n1_list = self.engine.n1_list()
        egt_list = self.engine.egt_list()
        oil_p = self.engine.oil_pressure()
        oil_t = self.engine.oil_temperature()
        self.vertical_mode = vertical_mode
        return (
            alt,
            speed,
            psi,
            pitch_cmd,
            aileron_cmd,
            self.engine.throttle,
            n1_list,
            pressure,
            demand,
            active,
            ice,
            wing_active,
            wing_ice,
            nav_dist,
            brake_temp,
            self.last_autobrake_active,
            oil_p,
            oil_t,
            egt_list,
            loc_dev,
            gs_dev,
            ils_dist,
            self.systems.flap_operable,
            self.systems.gear_operable,
        )


class A320IFRSim:
    def __init__(self, root_dir="jsbsim-master", dt=0.02):
        self.fdm = jsbsim.FGFDMExec(None, None)
        self.fdm.disable_output()
        self.fdm.set_root_dir(root_dir)
        self.fdm.load_model("A320")
        self.fdm.set_dt(dt)
        self.target_altitude = 4000  # feet
        self.target_psi = 0  # heading degrees
        self.target_speed = 250  # knots
        self.time_s = 0.0
        self.engines = EngineSystem(
            [
                Engine(self.fdm, 0, failure_chance=5e-5, fire_chance=1e-5),
                Engine(self.fdm, 1, failure_chance=5e-5, fire_chance=1e-5),
            ]
        )
        self.systems = SystemManager(self.fdm, failure_chance=1e-4)
        self.rat = RamAirTurbine()
        self.electrics = ElectricSystem(generator_failure_chance=5e-5, rat=self.rat)
        self.fuel = FuelSystem(self.fdm, self.engines, self.electrics)
        self.bleed = BleedAirSystem(self.engines, self.electrics)
        self.starter = EngineStartSystem(self.fdm, self.bleed, self.engines)
        self.environment = Environment(self.fdm)
        self.pitot = PitotSystem(self.environment)
        self.brakes = BrakeSystem()
        self.autobrake = AutobrakeSystem(self.brakes)
        self.tcas = TCASSystem(self.fdm)
        self.anti_ice = AntiIceSystem(
            self.environment, self.engines, self.bleed, failure_chance=1e-4
        )
        self.wing_ice = WingIceSystem(self.environment, self.bleed)
        self.pressurization = PressurizationSystem(self.fdm, self.bleed)
        self.cabin_temp = CabinTemperatureSystem(self.environment, self.bleed)
        self.oxygen = OxygenSystem()
        self.stall_warning = StallWarningSystem(self.fdm, wing_ice=self.wing_ice)
        self.gpws = GroundProximityWarningSystem(self.fdm)
        self.overspeed = OverspeedWarningSystem(self.fdm)
        self.weather_radar = WeatherRadarSystem(self.environment)
        self.fire_suppr = FireSuppressionSystem(self.engines)
        self.master_caution = MasterCautionSystem()
        self.nav = NavigationSystem(
            self.fdm,
            [
                (37.63, -122.02, None),
                (37.60, -121.98, None),
                (37.60, -122.05, None),
            ],
        )
        self.ils = ILSSystem(self.fdm, 37.60, -122.05, 270.0, 10.0)
        self.autopilot = Autopilot(
            self.fdm,
            dt,
            self.engines,
            self.systems,
            self.electrics,
            self.environment,
            self.anti_ice,
            self.wing_ice,
            self.brakes,
            self.nav,
            self.autobrake,
            self.ils,
            self.pitot,
        )
        self.init_conditions()
        self.autopilot.set_targets(
            self.target_altitude, self.target_psi, self.target_speed
        )

    def init_conditions(self):
        f = self.fdm
        f["ic/altitude-ft"] = self.target_altitude
        f["ic/psi-true-deg"] = self.target_psi
        f["ic/u-fps"] = self.target_speed * 1.68781  # convert kts to fps
        f["ic/vt-fps"] = self.target_speed * 1.68781
        f["ic/vc-kts"] = self.target_speed
        f["ic/h-sl-ft"] = self.target_altitude
        f["ic/long-gc-deg"] = -122.0
        f["ic/lat-gc-deg"] = 37.615
        f["ic/weight-lbs"] = 130000
        f["propulsion/engine/set-running"] = 0
        f["propulsion/engine[1]/set-running"] = 0
        f.run_ic()
        self.electrics.start_apu()
        self.starter.request_start()

    def set_parking_brake(self, on: bool) -> None:
        """Engage or release the parking brake."""
        self.brakes.set_parking_brake(on)

    def step(self):
        dt = self.fdm.get_delta_t()
        self.time_s += dt
        self.environment.update(dt)
        self.starter.update(dt)
        if (
            any(e.failed for e in self.engines.engines)
            and self.starter.state == "running"
        ):
            self.starter.request_start()
        (
            alt,
            speed,
            psi,
            pitch_cmd,
            aileron_cmd,
            throttle_cmd,
            n1_list,
            pressure,
            hyd_demand,
            anti_ice_on,
            ice_accum,
            wing_anti_ice_on,
            wing_ice_accum,
            nav_dist,
            brake_temp,
            autobrake_active,
            oil_press,
            oil_temp,
            egt_list,
            loc_dev,
            gs_dev,
            ils_dist,
            flap_ok,
            gear_ok,
        ) = self.autopilot.update()
        pitch_deg = self.fdm.get_property_value("attitude/pitch-deg")
        roll_deg = self.fdm.get_property_value("attitude/roll-deg")
        n1_avg = sum(n1_list) / len(n1_list)
        elec = self.electrics.update(n1_avg > 0.5, hyd_demand + 0.1, dt)
        cabin_alt, cabin_diff, bleed_press = self.pressurization.update(dt)
        cabin_temp = self.cabin_temp.update(dt)
        fuel_data = self.fuel.update()
        left_fuel = fuel_data["left_lbs"]
        right_fuel = fuel_data["right_lbs"]
        oxygen = self.oxygen.update(cabin_alt, dt)
        self.fire_suppr.update(dt)
        fire = self.engines.fire
        bottles = self.fire_suppr.bottles_left()
        stall = self.stall_warning.update()
        gpws = self.gpws.update()
        overspeed = self.overspeed.update()
        radar_alert = self.weather_radar.update()
        tcas_alert = self.tcas.update()
        fuel = fuel_data["total_lbs"]
        flap = self.fdm.get_property_value("fcs/flap-pos-norm")
        gear = self.fdm.get_property_value("gear/gear-pos-norm")
        vs_fpm = self.fdm.get_property_value("velocities/h-dot-fps") * 60.0
        outside_temp = self.environment.temperature_c
        precip_intensity = self.environment.precip

        # Update master caution status
        self.master_caution.set_warning("stall", stall)
        self.master_caution.set_warning("gpws", gpws)
        self.master_caution.set_warning("overspeed", overspeed)
        self.master_caution.set_warning("fire", fire)
        caution = self.master_caution.is_active()
        self.fdm.run()
        return {
            "altitude_ft": alt,
            "speed_kt": speed,
            "heading_deg": psi,
            "vs_fpm": vs_fpm,
            "pitch_cmd": pitch_cmd,
            "aileron_cmd": aileron_cmd,
            "pitch_deg": pitch_deg,
            "roll_deg": roll_deg,
            "throttle_cmd": throttle_cmd,
            "n1": n1_list,
            "flap": flap,
            "gear": gear,
            "speedbrake": self.systems.speedbrake,
            "hyd_press": pressure,
            "elec_charge": elec,
            "anti_ice_on": anti_ice_on,
            "ice_accum": ice_accum,
            "wing_anti_ice_on": wing_anti_ice_on,
            "wing_ice_accum": wing_ice_accum,
            "cabin_altitude_ft": cabin_alt,
            "cabin_diff_psi": cabin_diff,
            "cabin_temp_c": cabin_temp,
            "bleed_press": bleed_press,
            "fuel_lbs": fuel,
            "fuel_left_lbs": left_fuel,
            "fuel_right_lbs": right_fuel,
            "fuel_flow_lbs_hr_eng1": fuel_data["flow0_pph"],
            "fuel_flow_lbs_hr_eng2": fuel_data["flow1_pph"],
            "apu_flow_lbs_hr": fuel_data["apu_flow_pph"],
            "crossfeed": fuel_data["crossfeed"],
            "oxygen_level": oxygen,
            "stall_warning": stall,
            "gpws_warning": gpws,
            "overspeed_warning": overspeed,
            "weather_radar": radar_alert,
            "nav_dist_nm": nav_dist,
            "ils_dist_nm": ils_dist,
            "loc_dev_deg": loc_dev,
            "gs_dev_ft": gs_dev,
            "brake_temp": brake_temp,
            "autobrake_active": autobrake_active,
            "oil_press": oil_press,
            "oil_temp": oil_temp,
            "egt": egt_list,
            "engine_fire": fire,
            "fire_bottles": bottles,
            "rat_deployed": self.electrics.rat_deployed(),
            "flap_operable": self.systems.flap_operable,
            "gear_operable": self.systems.gear_operable,
            "tcas_alert": tcas_alert,
            "master_caution": caution,
            "parking_brake": self.brakes.parking_brake,
            "outside_temp_c": outside_temp,
            "precip_intensity": precip_intensity,
            "time_s": self.time_s,
        }

    def run(self, steps=600):
        for i in range(steps):
            data = self.step()
            if i % 50 == 0:
                tcas_str = "NONE"
                if data["tcas_alert"] is not None:
                    t = data["tcas_alert"]
                    tcas_str = f"{t['bearing_deg']:.0f}deg {t['distance_nm']:.1f}nm"
                print(
                    f"t={i*self.fdm.get_delta_t():.1f}s alt={data['altitude_ft']:.1f}ft "
                    f"spd={data['speed_kt']:.1f}kt hdg={data['heading_deg']:.1f} "
                    f"vs={data['vs_fpm']:.0f}fpm flap={data['flap']:.2f} "
                    f"gear={data['gear']:.0f} thr={data['throttle_cmd']:.2f} "
                    f"n1={'/'.join(f'{n*100:.0f}' if n<=2 else f'{n:.0f}' for n in data['n1'])}% "
                    f"egt={'/'.join(f'{t*100:.0f}' for t in data['egt'])}% "
                    f"hyd={data['hyd_press']:.2f} elec={data['elec_charge']:.2f} "
                    f"fuel={data['fuel_lbs']:.0f}lb "
                    f"cabin={data['cabin_altitude_ft']:.0f}ft "
                    f"bleed={data['bleed_press']:.2f} "
                    f"ff={data['fuel_flow_lbs_hr_eng1']:.0f}/{data['fuel_flow_lbs_hr_eng2']:.0f} pph "
                    f"apu={data['apu_flow_lbs_hr']:.0f}pph "
                    f"xfeed={'ON' if data['crossfeed'] else 'OFF'} "
                    f"oxy={data['oxygen_level']:.2f} "
                    f"ice={data['ice_accum']:.2f} {'ON' if data['anti_ice_on'] else 'OFF'} "
                    f"wingice={data['wing_ice_accum']:.2f} {'ON' if data['wing_anti_ice_on'] else 'OFF'} "
                    f"stall={'YES' if data['stall_warning'] else 'NO'} "
                    f"gpws={'YES' if data['gpws_warning'] else 'NO'} "
                    f"os={'YES' if data['overspeed_warning'] else 'NO'} "
                    f"d2wp={data['nav_dist_nm']:.1f}nm "
                    f"d2ils={data['ils_dist_nm']:.1f}nm "
                    f"loc={data['loc_dev_deg']:.1f} "
                    f"gs={data['gs_dev_ft']:.0f} "
                    f"brk={data['brake_temp']:.2f} "
                    f"autobrk={'ON' if data['autobrake_active'] else 'OFF'} "
                    f"oil={data['oil_press']:.2f}/{data['oil_temp']:.2f} "
                    f"fire={'YES' if data['engine_fire'] else 'NO'} "
                    f"btl={data['fire_bottles']} "
                    f"rat={'DEP' if data['rat_deployed'] else 'STOW'} "
                    f"mc={'ON' if data['master_caution'] else 'OFF'} "
                    f"tcas={tcas_str}"
                )
            time.sleep(self.fdm.get_delta_t())


if __name__ == "__main__":
    sim = A320IFRSim()
    sim.run(300)

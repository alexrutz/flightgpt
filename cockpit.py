# High level cockpit systems for the A320 simulation.
# These classes bundle the individual systems from ifrsim.py to
# represent a simplified A320 cockpit.

from ifrsim import A320IFRSim


class AutopilotPanel:
    """Minimal interface to the underlying autopilot."""

    def __init__(self, ap):
        self.ap = ap

    def engage(self) -> None:
        self.ap.engage()

    def disengage(self) -> None:
        self.ap.disengage()

    def engage_autothrottle(self) -> None:
        self.ap.autothrottle.engage()

    def disengage_autothrottle(self) -> None:
        self.ap.autothrottle.disengage()

    def set_altitude(self, alt_ft: float) -> None:
        self.ap.set_targets(altitude=alt_ft)

    def set_heading(self, hdg_deg: float) -> None:
        self.ap.set_targets(heading=hdg_deg)

    def set_speed(self, speed_kt: float) -> None:
        self.ap.set_targets(speed=speed_kt)

    def set_vs(self, vs_fpm: float) -> None:
        self.ap.set_targets(vs=vs_fpm)


class RadioPanel:
    """Very small radio management panel."""

    def __init__(self):
        self.com1_active = 118.0
        self.com1_standby = 121.5
        self.com2_active = 119.0
        self.com2_standby = 122.5

    def swap_com1(self):
        self.com1_active, self.com1_standby = (
            self.com1_standby,
            self.com1_active,
        )

    def swap_com2(self):
        self.com2_active, self.com2_standby = (
            self.com2_standby,
            self.com2_active,
        )


class Transponder:
    """Simple transponder model."""

    def __init__(self, code: int = 7000, mode: str = "standby"):
        self.code = code
        self.mode = mode  # off, standby, alt, on

    def set_code(self, code: int) -> None:
        self.code = max(0, min(code, 7777))

    def set_mode(self, mode: str) -> None:
        if mode in {"off", "standby", "alt", "on"}:
            self.mode = mode


class AutobrakePanel:
    """Allow setting the autobrake level."""

    def __init__(self, autobrake):
        self.autobrake = autobrake

    def set_level(self, level: str) -> None:
        self.autobrake.set_level(level)


class EnginePanel:
    """Start the engines via the starter system."""

    def __init__(self, starter):
        self.starter = starter

    def start(self) -> None:
        self.starter.request_start()


class APUPanel:
    """Control the auxiliary power unit."""

    def __init__(self, electrics):
        self.electrics = electrics

    def start(self) -> None:
        self.electrics.start_apu()

    def stop(self) -> None:
        self.electrics.stop_apu()


class ElectricalPanel:
    """Monitor aircraft electrical power."""

    def __init__(self, electrics):
        self.electrics = electrics

    def start_apu(self) -> None:
        self.electrics.start_apu()

    def stop_apu(self) -> None:
        self.electrics.stop_apu()

    @property
    def charge(self) -> float:
        return self.electrics.charge

    @property
    def rat_deployed(self) -> bool:
        return self.electrics.rat_deployed()


class FuelPanel:
    """Display fuel quantities and manage crossfeed."""

    def __init__(self, fuel):
        self.fuel = fuel

    def enable_crossfeed(self) -> None:
        self.fuel.crossfeed_on = True

    def disable_crossfeed(self) -> None:
        self.fuel.crossfeed_on = False

    def toggle_crossfeed(self) -> None:
        self.fuel.crossfeed_on = not self.fuel.crossfeed_on


class A320Cockpit:
    """High level interface exposing the main cockpit systems."""

    def __init__(self, root_dir: str = "jsbsim-master"):
        self.sim = A320IFRSim(root_dir=root_dir)
        self.radio = RadioPanel()
        self.transponder = Transponder()
        self.autopilot = AutopilotPanel(self.sim.autopilot)
        self.autobrake = AutobrakePanel(self.sim.autobrake)
        self.engine = EnginePanel(self.sim.starter)
        self.apu = APUPanel(self.sim.electrics)
        self.electrics = ElectricalPanel(self.sim.electrics)
        self.fuel = FuelPanel(self.sim.fuel)

    def step(self):
        """Advance the underlying simulation and return a status snapshot."""
        data = self.sim.step()
        return {
            "pfd": {
                "altitude_ft": data["altitude_ft"],
                "speed_kt": data["speed_kt"],
                "heading_deg": data["heading_deg"],
                "vs_fpm": data["vs_fpm"],
            },
            "ecam": {
                "n1": data["n1"],
                "oil_press": data["oil_press"],
                "oil_temp": data["oil_temp"],
                "egt": data["egt"],
                "fuel_lbs": data["fuel_lbs"],
                "apu_flow_pph": data["apu_flow_lbs_hr"],
                "fire_bottles": data["fire_bottles"],
            },
            "radio": {
                "com1_active": self.radio.com1_active,
                "com1_standby": self.radio.com1_standby,
                "com2_active": self.radio.com2_active,
                "com2_standby": self.radio.com2_standby,
            },
            "transponder": {
                "code": self.transponder.code,
                "mode": self.transponder.mode,
            },
            "autopilot": {
                "engaged": self.sim.autopilot.engaged,
                "autothrottle": self.sim.autopilot.autothrottle.engaged,
                "target_altitude_ft": self.sim.autopilot.altitude,
                "target_heading_deg": self.sim.autopilot.heading,
                "target_speed_kt": self.sim.autopilot.speed,
                "target_vs_fpm": self.sim.autopilot.vs_target_fpm,
                "autobrake_level": self.sim.autobrake.level,
                "autobrake_active": data["autobrake_active"],
            },
            "hydraulics": {"pressure": data["hyd_press"]},
            "electrical": {
                "charge": data["elec_charge"],
                "apu_running": self.sim.electrics.apu_running,
                "rat_deployed": data["rat_deployed"],
            },
            "fuel": {
                "left_lbs": data["fuel_left_lbs"],
                "right_lbs": data["fuel_right_lbs"],
                "total_lbs": data["fuel_lbs"],
                "crossfeed": data["crossfeed"],
            },
            "cabin": {
                "altitude_ft": data["cabin_altitude_ft"],
                "diff_psi": data["cabin_diff_psi"],
                "temperature_c": data["cabin_temp_c"],
            },
            "warnings": {
                "stall": data["stall_warning"],
                "gpws": data["gpws_warning"],
                "overspeed": data["overspeed_warning"],
                "fire": data["engine_fire"],
                "master_caution": data["master_caution"],
            },
        }


if __name__ == "__main__":
    cp = A320Cockpit()
    for i in range(300):
        status = cp.step()
        if i % 50 == 0:
            pfd = status["pfd"]
            print(
                f"ALT {pfd['altitude_ft']:.0f}FT SPD {pfd['speed_kt']:.1f}KT HDG {pfd['heading_deg']:.0f}"
            )


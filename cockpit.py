# High level cockpit systems for the A320 simulation.
# These classes bundle the individual systems from ifrsim.py to
# represent a simplified A320 cockpit.

from ifrsim import A320IFRSim
from a320_systems import (
    PrimaryFlightDisplay,
    EngineDisplay,
    FlightManagementSystem,
    AutopilotPanel,
    RadioPanel,
    Transponder,
    AutobrakePanel,
    EnginePanel,
    APUPanel,
    ElectricalPanel,
    FuelPanel,
)




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
        self.pfd = PrimaryFlightDisplay()
        self.ecam_display = EngineDisplay()
        self.fms = FlightManagementSystem(self.sim.nav)

    def step(self):
        """Advance the underlying simulation and return a status snapshot."""
        data = self.sim.step()
        self.pfd.update(data)
        self.ecam_display.update(data)
        return {
            "pfd": {
                "altitude_ft": self.pfd.altitude_ft,
                "speed_kt": self.pfd.speed_kt,
                "heading_deg": self.pfd.heading_deg,
                "vs_fpm": self.pfd.vs_fpm,
            },
            "ecam": {
                "n1": self.ecam_display.n1,
                "oil_press": self.ecam_display.oil_press,
                "oil_temp": self.ecam_display.oil_temp,
                "egt": self.ecam_display.egt,
                "fuel_lbs": self.ecam_display.fuel_lbs,
                "apu_flow_pph": self.ecam_display.apu_flow_pph,
                "fire_bottles": self.ecam_display.fire_bottles,
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
            "tcas": data["tcas_alert"],
            "navigation": {
                "active_waypoint": self.fms.active_waypoint(),
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
                "tcas": data["tcas_alert"],
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


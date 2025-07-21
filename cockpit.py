# High level cockpit systems for the A320 simulation.
# These classes bundle the individual systems from ifrsim.py to
# represent a simplified A320 cockpit.

from ifrsim import A320IFRSim
from a320_systems import (
    PrimaryFlightDisplay,
    EngineDisplay,
    FlightManagementSystem,
    PressurizationDisplay,
    WarningPanel,
    AutopilotPanel,
    RadioPanel,
    Transponder,
    AutobrakePanel,
    EnginePanel,
    APUPanel,
    ElectricalPanel,
    FuelPanel,
    FlightControlPanel,
    WeatherRadarPanel,
    NavigationDisplay,
    TCASDisplay,
    SystemsStatusPanel,
    OverheadPanel,
    CabinSignsPanel,
    OxygenPanel,
    LightingPanel,
    HydraulicPanel,
    BleedAirPanel,
    EnvironmentPanel,
    AltimeterPanel,
    ParkingBrakePanel,
    BrakesPanel,
    ClockPanel,
    CockpitSystems,
)
from mcdu import MCDU


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
        self.controls = FlightControlPanel(self.sim.systems)
        self.weather_radar = WeatherRadarPanel(self.sim.weather_radar)
        self.nav_display = NavigationDisplay()
        self.tcas_display = TCASDisplay()
        self.system_status = SystemsStatusPanel()
        self.overhead = OverheadPanel()
        self.hydraulic_panel = HydraulicPanel()
        self.bleed_air_panel = BleedAirPanel()
        self.environment_panel = EnvironmentPanel()
        self.cabin_signs = CabinSignsPanel()
        self.oxygen_display = OxygenPanel()
        self.lights = LightingPanel()
        self.parking_brake = ParkingBrakePanel()
        self.brakes_display = BrakesPanel()
        self.clock = ClockPanel()
        self.altimeter = AltimeterPanel()
        self.pfd = PrimaryFlightDisplay()
        self.ecam_display = EngineDisplay()
        self.pressurization = PressurizationDisplay()
        self.warnings_panel = WarningPanel()
        self.fms = FlightManagementSystem(self.sim.nav, self.sim.nav_db)
        self.mcdu = MCDU(self.fms)
        self.cockpit_systems = CockpitSystems()

    def set_seatbelt_sign(self, on: bool) -> None:
        """Toggle the seatbelt sign."""
        self.cabin_signs.seatbelt_on = on

    def set_no_smoking_sign(self, on: bool) -> None:
        """Toggle the no smoking sign."""
        self.cabin_signs.no_smoking_on = on

    def set_landing_light(self, on: bool) -> None:
        """Toggle the landing light."""
        self.lights.set_landing(on)

    def set_taxi_light(self, on: bool) -> None:
        """Toggle the taxi light."""
        self.lights.set_taxi(on)

    def set_nav_light(self, on: bool) -> None:
        """Toggle the navigation lights."""
        self.lights.set_nav(on)

    def set_strobe_light(self, on: bool) -> None:
        """Toggle the strobe light."""
        self.lights.set_strobe(on)

    def set_beacon_light(self, on: bool) -> None:
        """Toggle the beacon light."""
        self.lights.set_beacon(on)

    def set_parking_brake(self, on: bool) -> None:
        """Engage or release the parking brake."""
        self.sim.set_parking_brake(on)
        self.parking_brake.engaged = on

    def set_altimeter(self, pressure_hpa: float) -> None:
        """Set the altimeter pressure setting."""
        self.altimeter.pressure_hpa = pressure_hpa

    def step(self):
        """Advance the underlying simulation and return a status snapshot."""
        data = self.sim.step()
        self.pfd.update(data)
        self.ecam_display.update(data)
        self.weather_radar.update(data)
        self.nav_display.update(data)
        self.tcas_display.update(data)
        self.system_status.update(data)
        self.overhead.update(data)
        self.hydraulic_panel.update(data)
        self.bleed_air_panel.update(data)
        self.environment_panel.update(data)
        self.fuel.update(data)
        self.cabin_signs.update(data)
        self.parking_brake.update(data)
        self.brakes_display.update(data)
        self.oxygen_display.update(data)
        self.pressurization.update(data)
        self.altimeter.update({"pressure_hpa": self.altimeter.pressure_hpa})
        self.clock.update(data)
        warnings = {
            "stall": data["stall_warning"],
            "gpws": data["gpws_warning"],
            "overspeed": data["overspeed_warning"],
            "fire": data["engine_fire"],
            "tcas": data["tcas_alert"],
            "master_caution": data["master_caution"],
        }
        self.warnings_panel.update({"warnings": warnings})
        autopilot_info = {
            "engaged": self.sim.autopilot.engaged,
            "autothrottle": self.sim.autopilot.autothrottle.engaged,
            "target_altitude_ft": self.sim.autopilot.altitude,
            "target_heading_deg": self.sim.autopilot.heading,
            "target_speed_kt": self.sim.autopilot.speed,
            "target_vs_fpm": self.sim.autopilot.vs_target_fpm,
            "autobrake_level": self.sim.autobrake.level,
            "autobrake_active": data["autobrake_active"],
            "automation": self.sim.autopilot.auto_manage_systems,
            "vertical_mode": self.sim.autopilot.vertical_mode,
            "lateral_mode": self.sim.autopilot.lateral_mode,
        }
        mcdu_pages = self.mcdu.all_pages()
        cockpit_data = {
            **data,
            "warnings": warnings,
            "apu_running": self.sim.electrics.apu_running,
            "generator_failed": self.sim.electrics.generator_failed,
            "autopilot": autopilot_info,
            "parking_brake": self.sim.brakes.parking_brake,
            "pressure_hpa": self.altimeter.pressure_hpa,
            "mcdu": {
                "flight_plan": [tuple(wp) for wp in self.fms.waypoints],
                "active_index": self.fms.nav.index,
                "pages": mcdu_pages,
            },
        }
        self.cockpit_systems.update(cockpit_data)
        return {
            "pfd": {
                "altitude_ft": self.pfd.altitude_ft,
                "speed_kt": self.pfd.speed_kt,
                "heading_deg": self.pfd.heading_deg,
                "vs_fpm": self.pfd.vs_fpm,
                "pitch_deg": self.pfd.pitch_deg,
                "roll_deg": self.pfd.roll_deg,
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
            "autopilot": autopilot_info,
            "nav_display": {
                "distance_nm": self.nav_display.distance_nm,
                "ils_distance_nm": self.nav_display.ils_distance_nm,
                "loc_dev_deg": self.nav_display.loc_dev_deg,
                "gs_dev_ft": self.nav_display.gs_dev_ft,
                "tcas": self.nav_display.tcas_alert,
            },
            "hydraulics": {"pressure": data["hyd_press"]},
            "hydraulic_panel": {"pressure": self.hydraulic_panel.pressure},
            "electrical": {
                "charge": data["elec_charge"],
                "apu_running": self.sim.electrics.apu_running,
                "rat_deployed": data["rat_deployed"],
                "generator_failed": self.sim.electrics.generator_failed,
            },
            "systems": {
                "hydraulic_pressure": self.system_status.hydraulic_pressure,
                "electrical_charge": self.system_status.electrical_charge,
                "bleed_pressure": self.system_status.bleed_pressure,
            },
            "overhead": {
                "apu_running": self.overhead.apu_running,
                "crossfeed": self.overhead.crossfeed,
            },
            "bleed_air": {
                "pressure": self.bleed_air_panel.pressure,
                "anti_ice_on": self.bleed_air_panel.anti_ice_on,
                "wing_anti_ice_on": self.bleed_air_panel.wing_anti_ice_on,
            },
            "environment": {
                "temperature_c": data["outside_temp_c"],
                "precip_intensity": data["precip_intensity"],
            },
            "weather_radar": self.weather_radar.detecting,
            "tcas_display": {
                "alert": self.tcas_display.alert,
                "bearing_deg": self.tcas_display.bearing_deg,
                "distance_nm": self.tcas_display.distance_nm,
                "alt_diff_ft": self.tcas_display.alt_diff_ft,
            },
            "navigation": {
                "active_waypoint": self.fms.active_waypoint(),
            },
            "mcdu": {
                "flight_plan": [tuple(wp) for wp in self.fms.waypoints],
                "active_index": self.fms.nav.index,
                "pages": mcdu_pages,
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
            "altimeter": {"pressure_hpa": self.altimeter.pressure_hpa},
            "oxygen": {"level": data["oxygen_level"]},
            "cabin_signs": {
                "seatbelt": self.cabin_signs.seatbelt_on,
                "no_smoking": self.cabin_signs.no_smoking_on,
            },
            "lights": {
                "landing": self.lights.landing_on,
                "taxi": self.lights.taxi_on,
                "nav": self.lights.nav_on,
                "strobe": self.lights.strobe_on,
                "beacon": self.lights.beacon_on,
            },
            "controls": {
                "flap": data["flap"],
                "gear": data["gear"],
                "speedbrake": self.sim.systems.speedbrake,
                "parking_brake": self.sim.brakes.parking_brake,
            },
            "brakes": {
                "temperature": self.brakes_display.temperature,
                "autobrake_active": self.brakes_display.autobrake_active,
            },
            "clock": {"time": self.clock.time_hms},
            "warnings": warnings,
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

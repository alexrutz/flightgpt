"""Simplified A320 cockpit system models."""

from dataclasses import dataclass, field
from typing import List, Optional

from ifrsim import NavigationSystem


@dataclass
class PrimaryFlightDisplay:
    """Minimal primary flight display state."""

    altitude_ft: float = 0.0
    speed_kt: float = 0.0
    heading_deg: float = 0.0
    vs_fpm: float = 0.0
    fd_pitch: float = 0.0
    fd_roll: float = 0.0

    def update(self, data: dict) -> None:
        self.altitude_ft = data.get("altitude_ft", 0.0)
        self.speed_kt = data.get("speed_kt", 0.0)
        self.heading_deg = data.get("heading_deg", 0.0)
        self.vs_fpm = data.get("vs_fpm", 0.0)
        self.fd_pitch = data.get("pitch_cmd", 0.0)
        self.fd_roll = data.get("aileron_cmd", 0.0)


@dataclass
class EngineDisplay:
    """Basic engine and system parameters shown on the ECAM."""

    n1: List[float] = field(default_factory=list)
    oil_press: float = 0.0
    oil_temp: float = 0.0
    egt: List[float] = field(default_factory=list)
    fuel_lbs: float = 0.0
    apu_flow_pph: float = 0.0
    fire_bottles: int = 0

    def update(self, data: dict) -> None:
        self.n1 = data.get("n1", [])
        self.oil_press = data.get("oil_press", 0.0)
        self.oil_temp = data.get("oil_temp", 0.0)
        self.egt = data.get("egt", [])
        self.fuel_lbs = data.get("fuel_lbs", 0.0)
        self.apu_flow_pph = data.get("apu_flow_lbs_hr", 0.0)
        self.fire_bottles = data.get("fire_bottles", 0)


@dataclass
class PressurizationDisplay:
    """Show basic cabin pressurization information."""

    cabin_alt_ft: float = 0.0
    diff_psi: float = 0.0
    temperature_c: float = 0.0

    def update(self, data: dict) -> None:
        self.cabin_alt_ft = data.get("cabin_altitude_ft", 0.0)
        self.diff_psi = data.get("cabin_diff_psi", 0.0)
        self.temperature_c = data.get("cabin_temp_c", 0.0)


@dataclass
class WarningPanel:
    """Aggregate important warning flags."""

    master_caution: bool = False
    stall: bool = False
    gpws: bool = False
    overspeed: bool = False
    fire: bool = False
    tcas: bool = False

    def update(self, data: dict) -> None:
        warnings = data.get("warnings", {})
        self.master_caution = warnings.get("master_caution", False)
        self.stall = warnings.get("stall", False)
        self.gpws = warnings.get("gpws", False)
        self.overspeed = warnings.get("overspeed", False)
        self.fire = warnings.get("fire", False)
        self.tcas = warnings.get("tcas", False)


@dataclass
class FlightManagementSystem:
    """Very small FMS handling a route of waypoints."""

    nav: NavigationSystem

    def __init__(self, nav: NavigationSystem) -> None:
        self.nav = nav

    @property
    def waypoints(self) -> List[tuple]:
        return self.nav.waypoints

    def load_route(self, wpts: List[tuple]) -> None:
        """Load an entirely new route."""
        self.nav.waypoints = list(wpts)
        self.nav.index = 0

    def add_waypoint(
        self, lat_deg: float, lon_deg: float, alt_ft: Optional[float] = None
    ) -> None:
        self.nav.add_waypoint(lat_deg, lon_deg, alt_ft)

    def active_waypoint(self) -> Optional[tuple]:
        if self.nav.index < len(self.nav.waypoints):
            return self.nav.waypoints[self.nav.index]
        return None

    def advance_waypoint(self) -> None:
        if self.nav.index < len(self.nav.waypoints) - 1:
            self.nav.index += 1


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


@dataclass
class AutopilotDisplay:
    """Show current autopilot and autobrake status."""

    engaged: bool = False
    autothrottle: bool = False
    target_altitude_ft: float = 0.0
    target_heading_deg: float = 0.0
    target_speed_kt: float = 0.0
    target_vs_fpm: float = 0.0
    autobrake_level: str = "off"
    autobrake_active: bool = False
    automation: bool = False

    def update(self, data: dict) -> None:
        self.engaged = data.get("engaged", False)
        self.autothrottle = data.get("autothrottle", False)
        self.target_altitude_ft = data.get("target_altitude_ft", 0.0)
        self.target_heading_deg = data.get("target_heading_deg", 0.0)
        self.target_speed_kt = data.get("target_speed_kt", 0.0)
        self.target_vs_fpm = data.get("target_vs_fpm", 0.0)
        self.autobrake_level = data.get("autobrake_level", "off")
        self.autobrake_active = data.get("autobrake_active", False)
        self.automation = data.get("automation", False)


class RadioPanel:
    """Very small radio management panel."""

    def __init__(self):
        self.com1_active = 118.0
        self.com1_standby = 121.5
        self.com2_active = 119.0
        self.com2_standby = 122.5

    def swap_com1(self) -> None:
        self.com1_active, self.com1_standby = (
            self.com1_standby,
            self.com1_active,
        )

    def swap_com2(self) -> None:
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


class FlightControlPanel:
    """Manually command gear, flap and speedbrake positions."""

    def __init__(self, systems):
        self.systems = systems

    def set_gear(self, position: str) -> None:
        """Set landing gear to 'up' or 'down'."""
        if position.lower() == "down":
            self.systems.set_targets(gear=1.0)
        elif position.lower() == "up":
            self.systems.set_targets(gear=0.0)

    def set_flap(self, setting: float) -> None:
        """Set the flap lever between 0.0 and 1.0."""
        self.systems.set_targets(flap=max(0.0, min(setting, 1.0)))

    def set_speedbrake(self, setting: float) -> None:
        """Set the speedbrake lever between 0.0 and 1.0."""
        self.systems.set_targets(speedbrake=max(0.0, min(setting, 1.0)))


class WeatherRadarPanel:
    """Display simple weather radar indications."""

    def __init__(self, radar):
        self.radar = radar
        self.detecting = False

    def update(self, data: dict) -> None:
        self.detecting = data.get("weather_radar", False)


@dataclass
class NavigationDisplay:
    """Show navigation and ILS information on the ND."""

    distance_nm: float = 0.0
    ils_distance_nm: float = 0.0
    loc_dev_deg: float = 0.0
    gs_dev_ft: float = 0.0
    tcas_alert: bool = False

    def update(self, data: dict) -> None:
        self.distance_nm = data.get("nav_dist_nm", 0.0)
        self.ils_distance_nm = data.get("ils_dist_nm", 0.0)
        self.loc_dev_deg = data.get("loc_dev_deg", 0.0)
        self.gs_dev_ft = data.get("gs_dev_ft", 0.0)
        self.tcas_alert = data.get("tcas_alert", False)


@dataclass
class TCASDisplay:
    """Show TCAS traffic alert information."""

    bearing_deg: float = 0.0
    distance_nm: float = 0.0
    alt_diff_ft: float = 0.0
    alert: bool = False

    def update(self, data: dict) -> None:
        alert = data.get("tcas_alert")
        if alert:
            self.bearing_deg = alert.get("bearing_deg", 0.0)
            self.distance_nm = alert.get("distance_nm", 0.0)
            self.alt_diff_ft = alert.get("alt_diff_ft", 0.0)
            self.alert = True
        else:
            self.alert = False


@dataclass
class SystemsStatusPanel:
    """Display hydraulic, electrical and bleed air status."""

    hydraulic_pressure: float = 0.0
    electrical_charge: float = 0.0
    bleed_pressure: float = 0.0

    def update(self, data: dict) -> None:
        self.hydraulic_pressure = data.get("hyd_press", 0.0)
        self.electrical_charge = data.get("elec_charge", 0.0)
        self.bleed_pressure = data.get("bleed_press", 0.0)


@dataclass
class OverheadPanel:
    """Monitor high level aircraft system states."""

    apu_running: bool = False
    crossfeed: bool = False

    def update(self, data: dict) -> None:
        self.apu_running = data.get("apu_running", False)
        self.crossfeed = data.get("crossfeed", False)


@dataclass
class OxygenPanel:
    """Display remaining oxygen supply."""

    level: float = 0.0

    def update(self, data: dict) -> None:
        self.level = data.get("oxygen_level", 0.0)


@dataclass
class CabinSignsPanel:
    """Manage seatbelt and no smoking signs."""

    seatbelt_on: bool = False
    no_smoking_on: bool = False

    def update(self, data: dict) -> None:
        if "seatbelt_on" in data:
            self.seatbelt_on = data["seatbelt_on"]
        if "no_smoking_on" in data:
            self.no_smoking_on = data["no_smoking_on"]


@dataclass
class ParkingBrakePanel:
    """Indicate parking brake state."""

    engaged: bool = False

    def update(self, data: dict) -> None:
        if "parking_brake" in data:
            self.engaged = data["parking_brake"]


@dataclass
class LightingPanel:
    """Manage exterior light switches."""

    landing_on: bool = False
    taxi_on: bool = False
    nav_on: bool = False
    strobe_on: bool = False
    beacon_on: bool = False

    def set_landing(self, on: bool) -> None:
        self.landing_on = on

    def set_taxi(self, on: bool) -> None:
        self.taxi_on = on

    def set_nav(self, on: bool) -> None:
        self.nav_on = on

    def set_strobe(self, on: bool) -> None:
        self.strobe_on = on

    def set_beacon(self, on: bool) -> None:
        self.beacon_on = on


@dataclass
class ClockPanel:
    """Simple chronometer showing elapsed simulation time."""

    time_s: float = 0.0

    def update(self, data: dict) -> None:
        self.time_s = data.get("time_s", self.time_s)

    @property
    def time_hms(self) -> str:
        h = int(self.time_s // 3600)
        m = int(self.time_s % 3600 // 60)
        s = int(self.time_s % 60)
        return f"{h:02}:{m:02}:{s:02}"


@dataclass
class CockpitSystems:
    """Aggregate all major cockpit panels for convenience."""

    pfd: PrimaryFlightDisplay = field(default_factory=PrimaryFlightDisplay)
    engine: EngineDisplay = field(default_factory=EngineDisplay)
    pressurization: PressurizationDisplay = field(default_factory=PressurizationDisplay)
    warnings: WarningPanel = field(default_factory=WarningPanel)
    navigation: NavigationDisplay = field(default_factory=NavigationDisplay)
    tcas: TCASDisplay = field(default_factory=TCASDisplay)
    autopilot: AutopilotDisplay = field(default_factory=AutopilotDisplay)
    systems: SystemsStatusPanel = field(default_factory=SystemsStatusPanel)
    overhead: OverheadPanel = field(default_factory=OverheadPanel)
    oxygen: 'OxygenPanel' = field(default_factory=lambda: OxygenPanel())
    cabin: CabinSignsPanel = field(default_factory=CabinSignsPanel)
    lights: LightingPanel = field(default_factory=LightingPanel)
    parking_brake: ParkingBrakePanel = field(default_factory=ParkingBrakePanel)
    clock: ClockPanel = field(default_factory=ClockPanel)

    def update(self, data: dict) -> None:
        """Update all panels from a simulation snapshot."""
        self.pfd.update(data)
        self.engine.update(data)
        self.pressurization.update(data)
        self.warnings.update({"warnings": data.get("warnings", {})})
        self.navigation.update(data)
        self.tcas.update(data)
        self.autopilot.update(data.get("autopilot", {}))
        self.systems.update(data)
        self.overhead.update(data)
        self.oxygen.update(data)
        self.cabin.update(data)
        self.parking_brake.update(data)
        self.clock.update(data)
        # Light states are stored in the panel itself, so no update needed


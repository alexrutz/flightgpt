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

    def update(self, data: dict) -> None:
        self.altitude_ft = data.get("altitude_ft", 0.0)
        self.speed_kt = data.get("speed_kt", 0.0)
        self.heading_deg = data.get("heading_deg", 0.0)
        self.vs_fpm = data.get("vs_fpm", 0.0)


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

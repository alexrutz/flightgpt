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

    def add_waypoint(self, lat_deg: float, lon_deg: float, alt_ft: Optional[float] = None) -> None:
        self.nav.add_waypoint(lat_deg, lon_deg, alt_ft)

    def active_waypoint(self) -> Optional[tuple]:
        if self.nav.index < len(self.nav.waypoints):
            return self.nav.waypoints[self.nav.index]
        return None

    def advance_waypoint(self) -> None:
        if self.nav.index < len(self.nav.waypoints) - 1:
            self.nav.index += 1


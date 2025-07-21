"""Very simple MCDU interface for route management."""

from __future__ import annotations

from typing import List, Optional

from a320_systems import FlightManagementSystem


class MCDU:
    """Lightweight wrapper around the :class:`FlightManagementSystem`."""

    def __init__(self, fms: FlightManagementSystem) -> None:
        self.fms = fms

    def load_route(self, idents: List[str]) -> None:
        """Load a new flight plan by waypoint identifiers."""
        self.fms.load_route_by_idents(idents)

    def add_waypoint(self, ident: str) -> None:
        """Append a waypoint to the current flight plan."""
        if not self.fms.nav_db:
            raise ValueError("Navigation database not configured")
        coords = self.fms.nav_db.lookup(ident)
        if not coords:
            raise ValueError(f"Unknown fix identifier: {ident}")
        self.fms.add_waypoint(coords[0], coords[1])

    def clear_route(self) -> None:
        """Remove all waypoints from the flight plan."""
        self.fms.load_route([])

    def flight_plan(self) -> List[tuple]:
        """Return the current flight plan waypoints."""
        return list(self.fms.waypoints)

    def active_waypoint(self) -> Optional[tuple]:
        """Return the active waypoint if any."""
        return self.fms.active_waypoint()

    def direct_to(self, index: int) -> None:
        """Skip ahead to a waypoint by index."""
        if not 0 <= index < len(self.fms.waypoints):
            raise IndexError("Waypoint index out of range")
        self.fms.nav.index = index

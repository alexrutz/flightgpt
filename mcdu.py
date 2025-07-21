"""Very simple MCDU interface for route management."""

from __future__ import annotations

from typing import List, Optional

from a320_systems import FlightManagementSystem


class MCDU:
    """Lightweight wrapper around the :class:`FlightManagementSystem`."""

    def __init__(self, fms: FlightManagementSystem) -> None:
        self.fms = fms
        self.pages = ["idx", "f-plan", "prog", "init"]

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
        self.fms.add_waypoint(coords[0], coords[1], ident=ident)

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

    def remove_waypoint(self, index: int) -> None:
        """Delete a waypoint from the flight plan."""
        self.fms.remove_waypoint(index)

    def set_altitude(self, index: int, alt_ft: Optional[float]) -> None:
        """Set the altitude constraint for a waypoint."""
        self.fms.set_altitude_constraint(index, alt_ft)

    def list_pages(self) -> List[str]:
        """Return available MCDU pages."""
        return list(self.pages)

    def all_pages(self) -> dict[str, List[str]]:
        """Return a mapping of page names to their textual representation."""
        return {name: self.get_page(name) for name in self.pages}

    def get_page(self, name: str) -> List[str]:
        """Return a simple textual representation of an MCDU page."""
        lname = name.lower()
        if lname not in self.pages:
            raise ValueError(f"Unknown page: {name}")
        if lname == "idx":
            lines = ["INDEX"]
            for p in self.pages:
                lines.append(p.upper())
            return lines
        if lname == "f-plan":
            lines = ["F-PLAN"]
            for i, wp in enumerate(self.fms.waypoints):
                lat, lon, alt, *rest = wp
                ident = rest[0] if rest else f"WP{i+1}"
                prefix = ">" if i == self.fms.nav.index else " "
                alt_str = f" {alt:.0f}ft" if alt is not None else ""
                lines.append(f"{prefix} {ident} {lat:.4f} {lon:.4f}{alt_str}")
            return lines
        if lname == "prog":
            dist = self.fms.nav.distance_to_waypoint()
            active = self.fms.active_waypoint()
            if not active:
                return ["PROGRESS", "NO ACTIVE WP"]
            lat, lon, alt, *rest = active
            ident = rest[0] if rest else "---"
            alt_str = f"{alt:.0f}ft" if alt is not None else "----"
            dist_str = f"{dist:.1f}nm" if dist is not None else "----"
            return [
                "PROGRESS",
                f"TO {ident} {lat:.4f} {lon:.4f}",
                f"DIST {dist_str} ALT {alt_str}",
            ]
        if lname == "init":
            lines = ["INIT"]
            if self.fms.waypoints:
                o = self.fms.waypoints[0]
                lines.append(f"ORIG {o[0]:.4f} {o[1]:.4f}")
            if len(self.fms.waypoints) > 1:
                d = self.fms.waypoints[-1]
                lines.append(f"DEST {d[0]:.4f} {d[1]:.4f}")
            rem = self.fms.nav.remaining_distance()
            if rem is not None:
                lines.append(f"DIST {rem:.1f}NM")
            return lines
        return []

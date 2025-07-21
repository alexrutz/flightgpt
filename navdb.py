"""Simple navigation database loader."""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Dict, Tuple


class NavDatabase:
    """Load airport and waypoint data from CSV files."""

    def __init__(self, airports_file: str | Path, waypoints_file: str | Path) -> None:
        self.airports: Dict[str, Tuple[float, float]] = {}
        self.waypoints: Dict[str, Tuple[float, float]] = {}
        self._load_airports(airports_file)
        self._load_waypoints(waypoints_file)

    def _load_airports(self, path: str | Path) -> None:
        with open(path, newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    lat = float(
                        row.get("lat_deg")
                        or row.get("latitude_deg")
                        or row.get("LAT_DEG")
                    )
                    lon = float(
                        row.get("lon_deg")
                        or row.get("longitude_deg")
                        or row.get("LON_DEG")
                    )
                except (TypeError, ValueError):
                    continue
                ident = row.get("ident", "").strip().upper()
                if ident:
                    self.airports[ident] = (lat, lon)

    def _load_waypoints(self, path: str | Path) -> None:
        with open(path, newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    lat = float(
                        row.get("lat_deg")
                        or row.get("latitude_deg")
                        or row.get("LAT_DEG")
                    )
                    lon = float(
                        row.get("lon_deg")
                        or row.get("longitude_deg")
                        or row.get("LON_DEG")
                    )
                except (TypeError, ValueError):
                    continue
                ident = row.get("ident", "").strip().upper()
                if ident:
                    self.waypoints[ident] = (lat, lon)

    def lookup(self, ident: str) -> Tuple[float, float] | None:
        """Return (lat, lon) for airport or waypoint identifier."""
        ident = ident.strip().upper()
        if ident in self.airports:
            return self.airports[ident]
        if ident in self.waypoints:
            return self.waypoints[ident]
        return None

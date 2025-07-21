#!/usr/bin/env python3
"""Download and convert the OurAirports database for the simulator."""

from __future__ import annotations

import csv
from pathlib import Path
import io
import urllib.request

AIRPORTS_URL = "https://davidmegginson.github.io/ourairports-data/airports.csv"
NAVAIDS_URL = "https://davidmegginson.github.io/ourairports-data/navaids.csv"


def download_airports() -> str:
    """Return raw CSV data from the OurAirports dataset."""
    with urllib.request.urlopen(AIRPORTS_URL) as resp:
        return resp.read().decode("utf-8", errors="ignore")


def download_navaids() -> str:
    """Return raw CSV data for world navaids."""
    with urllib.request.urlopen(NAVAIDS_URL) as resp:
        return resp.read().decode("utf-8", errors="ignore")


def convert_to_navdb(data: str, output: Path) -> None:
    """Write airports in navdb CSV format."""
    reader = csv.DictReader(io.StringIO(data))
    with open(output, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["ident", "name", "lat_deg", "lon_deg"])
        for row in reader:
            ident = row.get("ident", "").strip().upper()
            lat = row.get("latitude_deg") or row.get("lat_deg")
            lon = row.get("longitude_deg") or row.get("lon_deg")
            name = row.get("name", "").strip()
            if not ident or lat is None or lon is None:
                continue
            try:
                lat_f = float(lat)
                lon_f = float(lon)
            except ValueError:
                continue
            writer.writerow([ident, name, lat_f, lon_f])


def convert_navaids_to_waypoints(data: str, output: Path) -> None:
    """Write navaids as navdb waypoints."""
    reader = csv.DictReader(io.StringIO(data))
    with open(output, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["ident", "lat_deg", "lon_deg"])
        for row in reader:
            ident = row.get("ident", "").strip().upper()
            lat = row.get("latitude_deg") or row.get("lat_deg")
            lon = row.get("longitude_deg") or row.get("lon_deg")
            if not ident or lat is None or lon is None:
                continue
            try:
                lat_f = float(lat)
                lon_f = float(lon)
            except ValueError:
                continue
            writer.writerow([ident, lat_f, lon_f])


def main() -> None:
    data = download_airports()
    target_dir = Path(__file__).resolve().parents[1] / "data" / "navdb"
    convert_to_navdb(data, target_dir / "airports.csv")
    print(f"Wrote {target_dir / 'airports.csv'}")

    navaid_data = download_navaids()
    convert_navaids_to_waypoints(navaid_data, target_dir / "waypoints.csv")
    print(f"Wrote {target_dir / 'waypoints.csv'}")


if __name__ == "__main__":
    main()

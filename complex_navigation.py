
# -*- coding: utf-8 -*-
"""Advanced waypoint navigation model with cross-track guidance."""

from __future__ import annotations

import math


class ComplexNavigationSystem:
    """Manage a route of waypoints with lateral and vertical guidance."""

    def __init__(self, fdm, waypoints: list[tuple] | None = None, xtrack_gain: float = 5.0) -> None:
        self.fdm = fdm
        self.waypoints = waypoints or []
        self.index = 0
        self.xtrack_gain = xtrack_gain

    def add_waypoint(self, lat_deg: float, lon_deg: float, alt_ft: float | None = None) -> None:
        self.waypoints.append((lat_deg, lon_deg, alt_ft))

    def _bearing_distance(self, lat1: float, lon1: float, lat2: float, lon2: float) -> tuple[float, float]:
        lat1 = math.radians(lat1)
        lon1 = math.radians(lon1)
        lat2 = math.radians(lat2)
        lon2 = math.radians(lon2)
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        dist_nm = 3440.065 * c
        y = math.sin(dlon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        bearing = (math.degrees(math.atan2(y, x)) + 360) % 360
        return bearing, dist_nm

    def _cross_track(self, lat: float, lon: float, lat1: float, lon1: float, lat2: float, lon2: float) -> tuple[float, float]:
        course, dist12 = self._bearing_distance(lat1, lon1, lat2, lon2)
        bearing13, dist13 = self._bearing_distance(lat1, lon1, lat, lon)
        course_rad = math.radians(course)
        bearing_rad = math.radians(bearing13)
        dist13_rad = dist13 / 3440.065
        xt_rad = math.asin(math.sin(dist13_rad) * math.sin(bearing_rad - course_rad))
        at_rad = math.acos(min(max(math.cos(dist13_rad) / math.cos(xt_rad), -1.0), 1.0))
        return xt_rad * 3440.065, at_rad * 3440.065

    def update(self) -> tuple[float | None, float | None, float | None]:
        if len(self.waypoints) < 2 or self.index >= len(self.waypoints) - 1:
            return None, None, None

        lat = self.fdm.get_property_value("position/lat-gc-deg")
        lon = self.fdm.get_property_value("position/long-gc-deg")

        wp1 = self.waypoints[self.index]
        wp2 = self.waypoints[self.index + 1]

        course, leg_dist = self._bearing_distance(wp1[0], wp1[1], wp2[0], wp2[1])
        xtrack, along = self._cross_track(lat, lon, wp1[0], wp1[1], wp2[0], wp2[1])

        # Advance to next leg when past current one
        if along >= leg_dist and self.index < len(self.waypoints) - 2:
            self.index += 1
            wp1 = self.waypoints[self.index]
            wp2 = self.waypoints[self.index + 1]
            course, leg_dist = self._bearing_distance(wp1[0], wp1[1], wp2[0], wp2[1])
            xtrack, along = self._cross_track(lat, lon, wp1[0], wp1[1], wp2[0], wp2[1])

        heading = (course - self.xtrack_gain * xtrack) % 360
        distance = max(leg_dist - along, 0.0)

        alt = None
        alt1 = wp1[2]
        alt2 = wp2[2]
        if alt1 is not None and alt2 is not None and leg_dist > 0.0:
            alt = alt1 + (alt2 - alt1) * min(along, leg_dist) / leg_dist
        elif alt2 is not None:
            alt = alt2
        elif alt1 is not None:
            alt = alt1

        return heading, distance, alt

class TCASSystem:
    """Very small traffic collision avoidance system."""

    def __init__(self, fdm, traffic=None, alert_distance_nm=5.0, alert_alt_ft=1000.0):
        self.fdm = fdm
        self.traffic = traffic or []
        self.alert_distance = alert_distance_nm
        self.alert_alt = alert_alt_ft

    def set_traffic(self, traffic):
        self.traffic = traffic

    def add_target(self, lat_deg, lon_deg, alt_ft):
        self.traffic.append({"lat": lat_deg, "lon": lon_deg, "alt": alt_ft})

    def _bearing_distance(self, lat1, lon1, lat2, lon2):
        import math
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

    def update(self):
        if not self.traffic:
            return None
        lat = self.fdm.get_property_value("position/lat-gc-deg")
        lon = self.fdm.get_property_value("position/long-gc-deg")
        alt = self.fdm.get_property_value("position/h-sl-ft")
        for t in self.traffic:
            bearing, dist = self._bearing_distance(lat, lon, t["lat"], t["lon"])
            alt_diff = abs(t["alt"] - alt)
            if dist <= self.alert_distance and alt_diff <= self.alert_alt:
                return {"bearing_deg": bearing, "distance_nm": dist, "alt_diff_ft": alt_diff}
        return None

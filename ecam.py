from __future__ import annotations

from dataclasses import dataclass
from typing import List, Dict

from a320_systems import CockpitSystems


@dataclass
class ECAM:
    """Generate simple textual ECAM pages from cockpit system data."""

    systems: CockpitSystems
    pages: List[str]

    def __init__(self, systems: CockpitSystems) -> None:
        self.systems = systems
        self.pages = [
            "idx",
            "eng",
            "bleed",
            "hyd",
            "elec",
            "fuel",
            "status",
        ]

    def list_pages(self) -> List[str]:
        return list(self.pages)

    def all_pages(self) -> Dict[str, List[str]]:
        return {name: self.get_page(name) for name in self.pages}

    def get_page(self, name: str) -> List[str]:
        lname = name.lower()
        if lname not in self.pages:
            raise ValueError(f"Unknown page: {name}")
        if lname == "idx":
            lines = ["INDEX"]
            for p in self.pages:
                lines.append(p.upper())
            return lines
        if lname == "eng":
            e = self.systems.engine
            lines = ["ENG"]
            for i, n1 in enumerate(e.n1, 1):
                egt = e.egt[i - 1] if i - 1 < len(e.egt) else 0.0
                lines.append(f"ENG{i} N1 {n1:.0f}% EGT {egt:.0f}")
            lines.append(f"OIL {e.oil_press:.0f}psi {e.oil_temp:.0f}C")
            lines.append(
                f"FUEL {e.fuel_lbs:.0f}LB APU {e.apu_flow_pph:.0f}PPH"
            )
            return lines
        if lname == "bleed":
            b = self.systems.bleed_air
            p = self.systems.pressurization
            lines = ["BLEED"]
            lines.append(f"PRES {b.pressure:.0f}psi")
            lines.append(f"A-ICE {'ON' if b.anti_ice_on else 'OFF'}")
            lines.append(f"WING {'ON' if b.wing_anti_ice_on else 'OFF'}")
            lines.append(f"CAB ALT {p.cabin_alt_ft:.0f}FT")
            lines.append(f"CAB DIFF {p.diff_psi:.1f} PSI")
            return lines
        if lname == "hyd":
            h = self.systems.hydraulics
            return ["HYD", f"PRES {h.pressure:.0f}psi"]
        if lname == "elec":
            e = self.systems.electrical
            lines = ["ELEC"]
            lines.append(f"CHARGE {e.charge:.0f}%")
            lines.append(f"APU {'ON' if e.apu_running else 'OFF'}")
            lines.append(f"GEN FAIL {'YES' if e.generator_failed else 'NO'}")
            lines.append(f"RAT {'DEPLOYED' if e.rat_deployed else 'STOWED'}")
            return lines
        if lname == "fuel":
            f = self.systems.fuel
            lines = ["FUEL"]
            lines.append(f"L {f.left_lbs:.0f} R {f.right_lbs:.0f}")
            lines.append(f"TOTAL {f.total_lbs:.0f}LB")
            lines.append(f"XFEED {'ON' if f.crossfeed else 'OFF'}")
            return lines
        if lname == "status":
            w = self.systems.warnings
            lines = ["STATUS"]
            if w.fire:
                lines.append("ENG FIRE")
            if w.stall:
                lines.append("STALL")
            if w.gpws:
                lines.append("TERRAIN")
            if w.overspeed:
                lines.append("OVERSPEED")
            if w.tcas:
                lines.append("TCAS")
            if len(lines) == 1:
                lines.append("NORMAL")
            return lines
        return []

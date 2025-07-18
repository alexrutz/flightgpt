# A320 Cockpit Systems

This document provides a brief overview of the simplified cockpit systems
implemented in this project. They are designed to expose key aircraft data and
controls for a non-graphical cockpit environment.

## Primary Flight Display (PFD)
Tracks altitude, airspeed, heading, vertical speed and basic flight director
commands for external indicators.

## Engine Display / ECAM
Reports N1 thrust levels, oil pressure and temperature, exhaust gas
temperature, remaining fuel and APU bleed flow. It also shows the number of
available fire bottles.

## Pressurization Display
Shows cabin altitude, differential pressure and temperature.

## Warning Panel
Aggregates master caution, stall, ground proximity, overspeed, fire and TCAS
warnings.

## Flight Management System (FMS)
A small wrapper around the navigation system that stores waypoints and allows
basic route management.

## Autopilot Panel & Display
Interfaces with the autopilot and autothrottle. The display shows current
engagement state, targets and autobrake information.

## Radio Panel
Provides simple COM1/COM2 frequency management.

## Transponder
Allows setting of transponder code and operating mode.

## Autobrake Panel
Controls the autobrake system level for landing.

## Engine & APU Panels
Handle engine start requests and APU start/stop commands.

## Electrical Panel
Monitors battery charge, APU status and RAT deployment.

## Fuel Panel
Displays tank quantities and manages fuel crossfeed.

## Flight Control Panel
Manually commands landing gear, flap and speedbrake positions.

## Weather Radar Panel
Indicates heavy precipitation detected ahead of the aircraft.

## Navigation Display
Shows distance to the active waypoint, ILS deviations and TCAS alert state.

## TCAS Display
Reports the bearing, range and altitude difference of conflicting traffic.

## Systems Status Panel
Summarizes hydraulic, electrical and bleed air pressures.

## Overhead Panel
Indicates APU running state and fuel crossfeed status.

## Cabin Signs Panel
Tracks the seatbelt and no smoking signs.

## Parking Brake Panel
Indicates whether the parking brake is engaged.

## Oxygen Panel
Shows the remaining oxygen supply for reference.

## Clock Panel
Displays the elapsed simulation time.

## Lighting Panel
Controls exterior lighting such as landing, taxi, navigation, strobe and beacon
lights.

## CockpitSystems Helper
The `CockpitSystems` dataclass aggregates all of the above panels so they can
be updated from a single simulation snapshot.

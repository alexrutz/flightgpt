# A320 Cockpit Systems

This document provides a brief overview of the simplified cockpit systems
implemented in this project. They are designed to expose key aircraft data and
controls for a non-graphical cockpit environment. Each section now includes a
bit more detail so custom hardware or software can better integrate with the
simulation.

## Primary Flight Display (PFD)
Tracks altitude, airspeed, heading, vertical speed and basic flight director
commands for external indicators. The PFD also reports the actual aircraft
pitch and roll angles so a standby attitude indicator can be driven from the
snapshot data.  Autopilot flight director cues are exposed so external displays
can mimic the real Airbus layout.

## Engine Display / ECAM
Reports N1 thrust levels, oil pressure and temperature, exhaust gas
temperature, remaining fuel and APU bleed flow. It also shows the number of
available fire bottles.  Each engine is tracked individually and a simplified
fuel flow indicator helps monitor consumption and diagnose failures.

## Pressurization Display
Shows cabin altitude, differential pressure and temperature.  The display is
fed by the bleed air model and reflects how well the cabin is being maintained
during climbs or descents.

## Warning Panel
Aggregates master caution, stall, ground proximity, overspeed, fire and TCAS
warnings.  When multiple warnings are active the highest priority alert can be
driven to external annunciators.

## Flight Management System (FMS)
A small wrapper around the navigation system that stores waypoints and allows
basic route management.  Waypoints may include altitude constraints which the
autopilot uses for simple vertical navigation.

## Autopilot Panel & Display
Interfaces with the autopilot and autothrottle. The display shows current
engagement state, targets and autobrake information.  The autopilot can
automatically manage gear, flaps and speedbrake as well as track altitude
constraints provided by the FMS. It also reports the active vertical mode
(VS, ALT, VNAV or APP) so external displays can mimic the Airbus flight mode
annunciations.

## Radio Panel
Provides simple COM1/COM2 frequency management.  Active and standby
frequencies can be swapped to simulate standard Airbus radio operation.

## Transponder
Allows setting of the transponder code and operating mode.  Selecting
"ALT" mode enables altitude reporting to the TCAS display.

## Autobrake Panel
Controls the autobrake system level for landing.  Wheel brakes are applied
automatically on touchdown and brake temperature can be monitored on the brake
panel.

## Engine & APU Panels
Handle engine start requests and APU start/stop commands.  Engine starts rely on
bleed air from the APU and the panel also allows shutting the APU down once the
generators are online.

## Electrical Panel
Monitors battery charge, generator output and APU status. It now also reports
when a generator has failed and whether the emergency RAT is deployed. The RAT
automatically deploys if both generators and the APU fail, keeping essential
systems powered.

## Fuel Panel
Displays left, right and total tank quantities and manages fuel crossfeed.  A
simple automatic crossfeed keeps the tanks balanced when uneven consumption is
detected.

## Flight Control Panel
Manually commands landing gear, flap and speedbrake positions.  Manual inputs
override the autopilot's automatic system scheduling.

## Flight Controls Display
Reports the current flap, gear and speedbrake positions. It also indicates if
the flap or gear mechanisms are inoperable due to overspeed damage, helping to
diagnose system failures.

## Weather Radar Panel
Indicates heavy precipitation detected ahead of the aircraft.  The information
can be used to anticipate turbulence or icing conditions.

## Navigation Display
Shows distance to the active waypoint, ILS deviations and TCAS alert state.  The
localizer deviation is reported in degrees and the glideslope deviation in feet
for easy integration with external displays.

## TCAS Display
Reports the bearing, range and altitude difference of conflicting traffic and
highlights the alert state when a collision avoidance manoeuvre is required.

## Systems Status Panel
Summarizes hydraulic, electrical and bleed air pressures so the overall health
of the major systems can be assessed at a glance.

## Overhead Panel
Indicates APU running state and fuel crossfeed status similar to the annunciator
lights on a real overhead panel.

## Hydraulic Panel
Shows the current hydraulic system pressure.  Loss of pressure reduces control
surface authority and slows gear or flap operation.

## Bleed Air Panel
Displays bleed air pressure along with engine and wing anti-ice status.  Both
the pressurization and anti-ice systems depend on sufficient bleed supply.

## Environment Panel
Reports the outside air temperature and current precipitation intensity.  These
values feed the icing and weather radar models.

## Cabin Signs Panel
Tracks the seatbelt and no smoking signs.  The booleans can be used to trigger
audio chimes or indicator lights.

## Parking Brake Panel
Indicates whether the parking brake is engaged.

## Brake Panel
Shows current brake temperature and whether the autobrake system is active.
Excessive heat may lead to brake failures during the simulation.

## Oxygen Panel
Shows the remaining oxygen supply for reference and warns when the level becomes
critically low during a depressurisation event.

## Altimeter Panel
Stores the current barometric setting used by the primary flight display so the
PFD altitude reference is consistent.

## Clock Panel
Displays the elapsed simulation time in HH:MM:SS format for easy reference.

## Lighting Panel
Controls exterior lighting such as landing, taxi, navigation, strobe and beacon
lights.  While purely cosmetic, the states are exposed for completeness when
building a full hardware cockpit.

## CockpitSystems Helper
The `CockpitSystems` dataclass aggregates all of the above panels so they can be
updated from a single simulation snapshot. The snapshot also includes the
current radio and transponder settings for external consumers.  Brake
temperature information is included and every panel's state can be retrieved via
the `snapshot()` method for integration with other software or hardware.


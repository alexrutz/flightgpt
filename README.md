# flightgpt

Simple command line A320 flight simulator for instrument flying. The
simulator uses the [JSBSim](https://github.com/JSBSim-Team/jsbsim) flight
dynamics engine and now features a lightweight autopilot with PID controllers
for heading, altitude and speed.  Basic automatic flap and gear scheduling as
well as a yaw damper add a little system depth. Engines are started
automatically and the remaining fuel is displayed as the flight progresses.
Recent updates include a simple fuel system that reports per-engine fuel flow
and throttle commands with a short spool lag for more realistic behaviour.
A basic hydraulic model now limits flap and gear motion when pressure is low
and prints the current pressure for reference. The electrical system has been
extended with a small APU that automatically starts when the battery runs low
and supplements the engine-driven generator. Control surface authority is now
scaled with hydraulic pressure for more realistic failures. The wind model
adds vertical gusts and the pressurization system tracks cabin altitude for
additional realism. A basic anti-ice system now reacts to icing conditions
based on temperature and precipitation, degrading engine performance when ice
builds up. The latest update introduces a stall warning and ground proximity
warning system for more realistic aircraft handling cues. A small update adds a
simple altitude capture mode that uses a constant climb or descent rate until
the target is nearly reached. The fuel system now includes an automatic
crossfeed to balance the tanks and engine spool dynamics respond quicker when
large throttle changes are commanded.
An automatic autothrottle system now maintains the commanded airspeed for
more realistic power management. The latest build adds an overspeed warning
with automatic speedbrake deployment and introduces random hydraulic pump
failures for additional system depth.
A simple weather radar now detects heavy precipitation and reports it via
the cockpit interface.
A simple navigation display shows distance to the active waypoint and ILS
deviations, while a small systems page tracks hydraulic, electrical and
bleed air pressure. An overhead panel monitors the APU and fuel crossfeed
state.
A new bleed air model now ties engine and APU performance to cabin
pressurization and anti-ice efficiency for greater realism.
Hydraulic pumps now depend on engine or APU power, so losing all sources
quickly reduces control surface authority.
Engines now start using bleed air from the APU and take a few seconds to
reach idle for more authentic startup behaviour.
A small twin-engine model now tracks each engine separately, allowing
individual failures and more realistic system behaviour.
A small brake model tracks heat build-up on the ground for a touch more
system depth.
A basic autobrake system now manages wheel braking after touchdown for
more realistic landings.
A small cockpit interface exposes autopilot, radio, transponder,
autobrake, engine start and APU controls. Manual gear, flap and
speedbrake levers are available alongside an option to disable the
automatic system scheduling.
A small navigation system lets the autopilot track a short series of
waypoints for basic route following.
Random generator failures may require the APU to power the aircraft and
severe icing can now lead to engine flameouts that need a restart.
An emergency ram air turbine automatically deploys when both the generator
and APU fail, keeping essential systems powered until normal power is
restored.
An oil system tracks pressure and temperature and may cause engine
failures when overheating or losing lubrication.
A simple fire detection and suppression system automatically
extinguishes engine fires using two bottles for added emergency depth.
An exhaust temperature model now tracks engine heat and can cause
failures when limits are exceeded for even more realism.
A simple master caution system now highlights active warnings from
multiple subsystems.
A simple vertical navigation mode adjusts climb and descent rates to
meet waypoint altitude constraints when following a route.
An approach mode now tracks an ILS localizer and glideslope for hands-off
landings.
Wing icing is now simulated and increases stall speed unless the wing
anti-ice system is active.
Flap and landing gear mechanisms can now jam when moved above their
overspeed limits and a small cabin temperature model uses bleed air to
keep the cabin comfortable.
Seatbelt and no smoking signs can be toggled from the cockpit interface
to simulate passenger announcements.
Exterior lights such as landing, taxi and strobe lights can be
controlled via the CLI for basic lighting management.
The new `CockpitSystems` helper class aggregates all panels so they can be
updated from a single simulation data snapshot.
No graphics are provided – the goal is to use external hardware like LED
displays or buttons for cockpit interaction.

## Quick start

1. Install Python 3.12+ and the `jsbsim` package:

```bash
pip install jsbsim
```

2. Run the example simulator:

```bash
python ifrsim.py
```

3. Launch the interactive cockpit CLI:

```bash
python cockpit_cli.py
```

The output shows altitude, airspeed, heading and remaining fuel every
few seconds.  The aircraft model definition is stored in `data/A320`.

This is only a minimal starting point for a larger non-graphical IFR
trainer.  You can extend it with your own controls and connect external
hardware for displays and switches.

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
additional realism.
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

The output shows altitude, airspeed, heading and remaining fuel every
few seconds.  The aircraft model definition is stored in `data/A320`.

This is only a minimal starting point for a larger non-graphical IFR
trainer.  You can extend it with your own controls and connect external
hardware for displays and switches.

# flightgpt

Simple command line A320 flight simulator for instrument flying. The
simulator uses the [JSBSim](https://github.com/JSBSim-Team/jsbsim) flight
dynamics engine and now features a tiny autopilot with PID controllers for
heading, altitude and speed. Engines are started automatically and the
remaining fuel is displayed as the flight progresses. No graphics are
provided – the goal is to use external hardware like LED displays or buttons
for cockpit interaction.

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

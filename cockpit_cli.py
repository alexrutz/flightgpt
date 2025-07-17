"""Simple CLI to interact with the A320 cockpit systems."""

from cockpit import A320Cockpit


HELP_TEXT = """Available commands:
  step                - advance one simulation step
  run N               - run N simulation steps
  status              - show primary flight display info
  ap on|off           - engage or disengage the autopilot
  athr on|off         - engage or disengage the autothrottle
  alt VALUE           - set target altitude in ft
  hdg VALUE           - set target heading in deg
  spd VALUE           - set target speed in kt
  vs VALUE            - set target vertical speed in fpm
  xfeed on|off        - enable or disable fuel crossfeed
  apu start|stop      - start or stop the APU
  engines start       - start the engines
  quit                - exit the program"""


def print_status(status: dict) -> None:
    pfd = status["pfd"]
    ecam = status["ecam"]
    print(
        f"ALT {pfd['altitude_ft']:.0f}FT "
        f"SPD {pfd['speed_kt']:.1f}KT "
        f"HDG {pfd['heading_deg']:.0f} "
        f"VS {pfd['vs_fpm']:.0f}FPM "
        f"FUEL {ecam['fuel_lbs']:.0f}LB"
    )


def main() -> None:
    cp = A320Cockpit()
    print("A320 cockpit CLI. Type 'help' for commands.")
    while True:
        try:
            line = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not line:
            continue
        if line == "help":
            print(HELP_TEXT)
            continue
        if line == "quit":
            break
        parts = line.split()
        cmd = parts[0]
        args = parts[1:]

        if cmd == "step":
            status = cp.step()
            print_status(status)
            continue
        if cmd == "run" and args:
            try:
                count = int(args[0])
            except ValueError:
                print("Invalid step count")
                continue
            for _ in range(count):
                status = cp.step()
            print_status(status)
            continue
        if cmd == "status":
            status = cp.step()
            print_status(status)
            continue
        if cmd == "ap" and args:
            if args[0] == "on":
                cp.autopilot.engage()
            elif args[0] == "off":
                cp.autopilot.disengage()
            else:
                print("Usage: ap on|off")
            continue
        if cmd == "athr" and args:
            if args[0] == "on":
                cp.autopilot.engage_autothrottle()
            elif args[0] == "off":
                cp.autopilot.disengage_autothrottle()
            else:
                print("Usage: athr on|off")
            continue
        if cmd == "alt" and args:
            try:
                cp.autopilot.set_altitude(float(args[0]))
            except ValueError:
                print("Invalid altitude")
            continue
        if cmd == "hdg" and args:
            try:
                cp.autopilot.set_heading(float(args[0]))
            except ValueError:
                print("Invalid heading")
            continue
        if cmd == "spd" and args:
            try:
                cp.autopilot.set_speed(float(args[0]))
            except ValueError:
                print("Invalid speed")
            continue
        if cmd == "vs" and args:
            try:
                cp.autopilot.set_vs(float(args[0]))
            except ValueError:
                print("Invalid vertical speed")
            continue
        if cmd == "xfeed" and args:
            if args[0] == "on":
                cp.fuel.enable_crossfeed()
            elif args[0] == "off":
                cp.fuel.disable_crossfeed()
            else:
                print("Usage: xfeed on|off")
            continue
        if cmd == "apu" and args:
            if args[0] == "start":
                cp.apu.start()
            elif args[0] == "stop":
                cp.apu.stop()
            else:
                print("Usage: apu start|stop")
            continue
        if cmd == "engines" and args and args[0] == "start":
            cp.engine.start()
            continue
        print("Unknown command. Type 'help' for a list of commands.")


if __name__ == "__main__":
    main()

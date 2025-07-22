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
  baro VALUE          - set altimeter pressure in hPa
  xfeed on|off        - enable or disable fuel crossfeed
  gear up|down        - move the landing gear
  flap VALUE          - set flaps (0.0-1.0)
  spbrake VALUE       - set speedbrake (0.0-1.0)
  auto_sys on|off     - toggle automatic system management
  apu start|stop      - start or stop the APU
  engines start       - start the engines
  seatbelt on|off     - toggle the seatbelt sign
  nosmoke on|off      - toggle the no smoking sign
  landlight on|off    - toggle the landing light
  taxilight on|off    - toggle the taxi light
  navlight on|off     - toggle the navigation lights
  strobelight on|off  - toggle the strobe light
  beacon on|off       - toggle the beacon light
  pbrake on|off       - set the parking brake
  plan                - show current flight plan
  route A B C         - load a new route by waypoint idents
  direct INDEX        - skip to flight plan waypoint
  delwp INDEX         - delete waypoint at INDEX
  wpalt INDEX ALT     - set altitude constraint for waypoint
  mcdu pages          - list available MCDU pages
  mcdu PAGE           - show a textual MCDU page
  ecam pages          - list available ECAM pages
  ecam PAGE           - show a textual ECAM page
  ewd                 - show the engine warning/system display
  quit                - exit the program"""


def print_status(status: dict) -> None:
    pfd = status["pfd"]
    ecam = status["ecam"]
    ewd = status.get("ewd", {})
    line = (
        f"ALT {pfd['altitude_ft']:.0f}FT "
        f"SPD {pfd['speed_kt']:.1f}KT "
        f"HDG {pfd['heading_deg']:.0f} "
        f"VS {pfd['vs_fpm']:.0f}FPM "
        f"P {pfd['pitch_deg']:.1f} "
        f"R {pfd['roll_deg']:.1f} "
        f"FUEL {ecam['fuel_lbs']:.0f}LB "
        f"BELT {'ON' if status['cabin_signs']['seatbelt'] else 'OFF'} "
        f"SMOKE {'ON' if status['cabin_signs']['no_smoking'] else 'OFF'}"
        f" OXY {status['oxygen']['level']:.2f}"
        f" TIME {status['clock']['time']}"
        f" BARO {status['altimeter']['pressure_hpa']:.1f}hPa"
        f" PBRK {'ON' if status['controls']['parking_brake'] else 'OFF'}"
    )
    tcas = status.get("tcas_display", {})
    if tcas.get("alert"):
        line += (
            f" TCAS {tcas['bearing_deg']:.0f}deg"
            f" {tcas['distance_nm']:.1f}NM"
            f" {tcas['alt_diff_ft']:.0f}FT"
        )
    active = [name.upper() for name, on in ewd.get("warnings", {}).items() if on]
    if active:
        line += " WARN " + " ".join(active)
    print(line)


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
        if cmd == "baro" and args:
            try:
                cp.set_altimeter(float(args[0]))
            except ValueError:
                print("Invalid pressure")
            continue
        if cmd == "gear" and args:
            if args[0] in {"up", "down"}:
                cp.controls.set_gear(args[0])
            else:
                print("Usage: gear up|down")
            continue
        if cmd == "flap" and args:
            try:
                cp.controls.set_flap(float(args[0]))
            except ValueError:
                print("Invalid flap setting")
            continue
        if cmd == "spbrake" and args:
            try:
                cp.controls.set_speedbrake(float(args[0]))
            except ValueError:
                print("Invalid speedbrake setting")
            continue
        if cmd == "auto_sys" and args:
            if args[0] == "on":
                cp.sim.autopilot.set_system_automation(True)
            elif args[0] == "off":
                cp.sim.autopilot.set_system_automation(False)
            else:
                print("Usage: auto_sys on|off")
            continue
        if cmd == "xfeed" and args:
            if args[0] == "on":
                cp.enable_crossfeed()
            elif args[0] == "off":
                cp.disable_crossfeed()
            else:
                print("Usage: xfeed on|off")
            continue
        if cmd == "seatbelt" and args:
            if args[0] == "on":
                cp.set_seatbelt_sign(True)
            elif args[0] == "off":
                cp.set_seatbelt_sign(False)
            else:
                print("Usage: seatbelt on|off")
            continue
        if cmd == "nosmoke" and args:
            if args[0] == "on":
                cp.set_no_smoking_sign(True)
            elif args[0] == "off":
                cp.set_no_smoking_sign(False)
            else:
                print("Usage: nosmoke on|off")
            continue
        if cmd == "landlight" and args:
            if args[0] == "on":
                cp.set_landing_light(True)
            elif args[0] == "off":
                cp.set_landing_light(False)
            else:
                print("Usage: landlight on|off")
            continue
        if cmd == "taxilight" and args:
            if args[0] == "on":
                cp.set_taxi_light(True)
            elif args[0] == "off":
                cp.set_taxi_light(False)
            else:
                print("Usage: taxilight on|off")
            continue
        if cmd == "navlight" and args:
            if args[0] == "on":
                cp.set_nav_light(True)
            elif args[0] == "off":
                cp.set_nav_light(False)
            else:
                print("Usage: navlight on|off")
            continue
        if cmd == "strobelight" and args:
            if args[0] == "on":
                cp.set_strobe_light(True)
            elif args[0] == "off":
                cp.set_strobe_light(False)
            else:
                print("Usage: strobelight on|off")
            continue
        if cmd == "beacon" and args:
            if args[0] == "on":
                cp.set_beacon_light(True)
            elif args[0] == "off":
                cp.set_beacon_light(False)
            else:
                print("Usage: beacon on|off")
            continue
        if cmd == "pbrake" and args:
            if args[0] == "on":
                cp.set_parking_brake(True)
            elif args[0] == "off":
                cp.set_parking_brake(False)
            else:
                print("Usage: pbrake on|off")
            continue
        if cmd == "plan":
            plan = cp.mcdu.flight_plan()
            for i, wp in enumerate(plan):
                lat, lon, alt = wp
                alt_str = f" {alt:.0f}ft" if alt is not None else ""
                print(f"{i}: {lat:.4f}, {lon:.4f}{alt_str}")
            continue
        if cmd == "route" and args:
            try:
                cp.mcdu.load_route(args)
            except Exception as exc:
                print(f"Error: {exc}")
            continue
        if cmd == "direct" and args:
            try:
                idx = int(args[0])
            except ValueError:
                print("Invalid index")
                continue
            try:
                cp.mcdu.direct_to(idx)
            except Exception as exc:
                print(f"Error: {exc}")
            continue
        if cmd == "delwp" and args:
            try:
                idx = int(args[0])
            except ValueError:
                print("Invalid index")
                continue
            try:
                cp.mcdu.remove_waypoint(idx)
            except Exception as exc:
                print(f"Error: {exc}")
            continue
        if cmd == "wpalt" and len(args) >= 2:
            try:
                idx = int(args[0])
            except ValueError:
                print("Invalid index")
                continue
            alt_arg = args[1]
            alt = None
            if alt_arg.lower() != "none":
                try:
                    alt = float(alt_arg)
                except ValueError:
                    print("Invalid altitude")
                    continue
            try:
                cp.mcdu.set_altitude(idx, alt)
            except Exception as exc:
                print(f"Error: {exc}")
            continue
        if cmd == "mcdu" and args:
            sub = args[0]
            if sub == "pages":
                for name in cp.mcdu.list_pages():
                    print(name)
                continue
            page = sub
            try:
                lines = cp.mcdu.get_page(page)
            except Exception as exc:
                print(f"Error: {exc}")
            else:
                for line in lines:
                    print(line)
            continue
        if cmd == "ecam" and args:
            sub = args[0]
            if sub == "pages":
                for name in cp.ecam.list_pages():
                    print(name)
                continue
            page = sub
            try:
                lines = cp.ecam.get_page(page)
            except Exception as exc:
                print(f"Error: {exc}")
            else:
                for line in lines:
                    print(line)
            continue
        if cmd == "ewd":
            snapshot = cp.cockpit_systems.snapshot()
            ewd = snapshot.get("ewd", {})
            n1 = "/".join(f"{n:.0f}" for n in ewd.get("n1", []))
            egt = "/".join(f"{e:.0f}" for e in ewd.get("egt", []))
            line = (
                f"N1 {n1} EGT {egt} OIL {ewd.get('oil_press', 0):.0f}psi "
                f"FUEL {ewd.get('fuel_lbs', 0):.0f}LB"
            )
            active = [name.upper() for name, on in ewd.get("warnings", {}).items() if on]
            if active:
                line += " WARN " + " ".join(active)
            print(line)
            continue
        if cmd == "apu" and args:
            if args[0] == "start":
                cp.start_apu()
            elif args[0] == "stop":
                cp.stop_apu()
            else:
                print("Usage: apu start|stop")
            continue
        if cmd == "engines" and args and args[0] == "start":
            cp.engine.start()
            continue
        print("Unknown command. Type 'help' for a list of commands.")


if __name__ == "__main__":
    main()

from cockpit import A320Cockpit


def main(steps: int = 10) -> None:
    """Run the A320 cockpit simulation for a few steps and print a snapshot."""
    cp = A320Cockpit()
    for _ in range(steps):
        cp.step()
        snapshot = cp.cockpit_systems.snapshot()
        # Print a couple of key values from different panels
        pfd = snapshot["pfd"]
        ewd = snapshot["ewd"]
        warnings = snapshot["warnings"]
        active = [name.upper() for name, on in ewd.get("warnings", {}).items() if on]
        warn_text = " WARN " + " ".join(active) if active else ""
        print(
            f"ALT {pfd['altitude_ft']:.0f}FT SPD {pfd['speed_kt']:.1f}KT "
            f"HDG {pfd['heading_deg']:.0f} VS {pfd['vs_fpm']:.0f}FPM "
            f"N1 {ewd['n1'][0]:.2f}/{ewd['n1'][1]:.2f} "
            f"EGT {ewd['egt'][0]:.0f}/{ewd['egt'][1]:.0f} "
            f"MC {'ON' if warnings['master_caution'] else 'OFF'}" + warn_text
        )


if __name__ == "__main__":
    main()

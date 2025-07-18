import json
from cockpit import A320Cockpit


def main(steps: int = 5) -> None:
    cp = A320Cockpit()
    for _ in range(steps):
        cp.step()
        snapshot = cp.cockpit_systems.snapshot()
        print(json.dumps(snapshot, indent=2))


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Scenario: multiple atmos free-flyers."""

from px4_sitl_launcher import launch, Vehicle

WORLD = "kthspacelab"  # Alternative: "default"

VEHICLES = [
    Vehicle(name="atmos_1", model="gz_atmos", pose=(1, 0, 1, 0, 0, 0)),
    Vehicle(name="atmos_2", model="gz_atmos", pose=(2, 0, 1, 0, 0, 0)),
    Vehicle(name="atmos_3", model="gz_atmos", pose=(3, 0, 1, 0, 0, 0)),
]

SESSION = "atmos"  # tmux session name

if __name__ == "__main__":
    launch(vehicles=VEHICLES, world=WORLD, session=SESSION)

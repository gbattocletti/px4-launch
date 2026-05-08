"""
Shared launcher for PX4 SITL multi-vehicle scenarios.

Simulations can be launched from external scripts by calling the launch function, with
the desired configuration specifics passed as arguments, including number of vehicles,
models, poses, namespaces, world, terminal multiplexer backend, and gazebo settings.
"""

import argparse
import os
import shutil
import subprocess
import sys
import time
import shlex
from dataclasses import dataclass
from pathlib import Path

# Settings defaults --------------------------------------------------------------------
# Multiplexer backend used to host the per-vehicle windows.
#   "tmux"       -> plain tmux
#   "byobu-tmux" -> tmux wrapped by byobu (gives byobu's status bar / keys)
BACKEND = "byobu-tmux"

# Run Gazebo without the GUI.
HEADLESS = False

# Vehicle models -----------------------------------------------------------------------

# Autostart numbers for each model, used to select the correct airframe and settings
# from the PX4 ROMFS at runtime.
MODEL_AUTOSTART = {
    "gz_atmos": 70000,
    "gz_atmos_dual": 70001,
    "gz_uuv_bluerov2_heavy": 60002,
}
MODEL_BUILD = {
    "gz_atmos": "px4_sitl_spacecraft",
    "gz_atmos_dual": "px4_sitl_spacecraft",
    "gz_uuv_bluerov2_heavy": "px4_sitl_uuv",
}

# Env variables ------------------------------------------------------------------------

# Collection of all the env vars that are passed to the PX4 command line
PROPAGATED_KEYS = [
    "PX4_SYS_AUTOSTART",
    "PX4_SIM_MODEL",
    "PX4_UXRCE_DDS_NS",
    "PX4_GZ_MODEL_NAME",
    "PX4_GZ_MODEL_POSE",
    "PX4_GZ_WORLD",
    "PX4_GZ_STANDALONE",
    "PX4_GZ_NO_FOLLOW",
    "HEADLESS",
]


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


@dataclass
class Vehicle:
    """
    Vehicle dataclass.
    """

    name: str
    model: str
    pose: tuple = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def launch(
    vehicles: list[Vehicle],
    world: str = "default",
    px4_dir: str | None = None,
    session: str = "px4sitl",
    startup_delay: float = 6.0,
) -> None:
    """
    Launch the given list of Vehicles.

    Args:
        vehicles (list[Vehicle]): list of Vehicle dataclasses, each defining a single
            simulated vehicle. A PX4 instance is launched for each vehicle, with the
            appropriate environment variables, initial pose, and namespace.
        world (str, optional): name of the Gazebo world to load.
        px4_dir (str | None, optional): path to the PX4-Autopilot source dir
        session (str, optional): name of the tmux session to create
        startup_delay (float, optional): seconds to wait after launching the first
            instance before launching the rest. This is to give gz-server time to start
            up and be detected by the script before launching more instances that will
            try to connect to it.

    Returns:
        None
    """
    # Parse CLI args
    p = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        description="Launch the PX4 SITL scenario defined in this file.",
    )
    p.add_argument(
        "--startup-delay",
        type=float,
        default=startup_delay,
    )
    p.add_argument(
        "--kill",
        action="store_true",
        help="kill the tmux session and all sim processes, then exit",
    )
    p.add_argument(
        "--backend",
        choices=["tmux", "byobu-tmux"],
        default=BACKEND,
        help="multiplexer backend to host the per-vehicle windows",
    )
    p.add_argument(
        "--headless",
        action=argparse.BooleanOptionalAction,
        default=HEADLESS,
        help="run Gazebo without the GUI",
    )
    args = p.parse_args()

    # Resolve arguments
    if args.kill:
        _kill_everything(session)
        return
    if not vehicles:
        sys.exit("No vehicles defined in scenario.")
    if not shutil.which(args.backend):
        sys.exit(
            f"'{args.backend}' not found on PATH. "
            f"Install it first (e.g. `sudo apt install {args.backend.split('-')[0]}`)."
        )
    if px4_dir is None:
        px4_dir = os.environ.get("PX4_Autopilot_Dir")
    if not px4_dir:
        sys.exit("PX4 dir not set: export $PX4_Autopilot_Dir or pass px4_dir=")
    px4_dir: Path = Path(px4_dir).expanduser().resolve()

    # Validate models and check each required build is present.
    for v in vehicles:
        if v.model not in MODEL_BUILD:
            sys.exit(
                f"No build target known for model {v.model!r}. "
                f"Add it to MODEL_BUILD in px4_sitl_launcher.py."
            )
    needed_builds = {MODEL_BUILD[v.model] for v in vehicles}
    for build in needed_builds:
        px4_bin: Path = px4_dir / "build" / build / "bin" / "px4"
        if not px4_bin.is_file():
            sys.exit(
                f"PX4 binary not found at {px4_bin}.\n"
                f"-> run `make {build}` in {px4_dir} first."
            )

    # Build launch commands for each vehicle
    commands = []
    for i, v in enumerate(vehicles):
        env = _build_env(v, world=world, standalone=(i > 0), headless=args.headless)
        commands.append(
            (
                v.name,
                i,
                _build_command(px4_dir, MODEL_BUILD[v.model], i, env),
            )
        )

    # Summary
    print(f"backend : {args.backend}")
    print(f"PX4 dir : {px4_dir}")
    print(f"world   : {world}")
    print(f"headless: {args.headless}")
    print(f"vehicles: {len(vehicles)}")
    for name, instance, cmd in commands:
        print(f"  - {name:<10} i={instance}  {cmd}")
    print()

    # Execute commands to launch the session and spawn gz and the vehicle processes
    _launch_session(commands, session, args.startup_delay, args.backend)


def _build_env(
    vehicle: Vehicle,
    world: str,
    standalone: bool,
    headless: bool,
) -> dict:
    if vehicle.model not in MODEL_AUTOSTART:
        raise SystemExit(
            f"Unknown model {vehicle.model!r}. "
            f"Known: {list(MODEL_AUTOSTART)}. "
            f"Add it to MODEL_AUTOSTART in px4_sitl_launcher.py."
        )
    env = {
        "PX4_SYS_AUTOSTART": str(MODEL_AUTOSTART[vehicle.model]),
        "PX4_SIM_MODEL": vehicle.model,
        "PX4_UXRCE_DDS_NS": vehicle.name,
        "PX4_GZ_MODEL_POSE": ",".join(f"{v:g}" for v in vehicle.pose),
        "PX4_GZ_NO_FOLLOW": "1",  # always use to prevent camera from following robots
    }
    if world and world.lower() != "default":
        env["PX4_GZ_WORLD"] = world
    if standalone:  # use existing gz-server instead of spawning a new one
        env["PX4_GZ_STANDALONE"] = "1"
    if headless and not standalone:  # standalone instances don't spawn gz-server anyway
        env["HEADLESS"] = "1"
    return env


def _env_prefix(env: dict) -> str:
    ordered = [(k, env[k]) for k in PROPAGATED_KEYS if k in env]
    return " ".join(f"{k}={v}" for k, v in ordered)


def _build_command(
    px4_dir: Path,
    build: str,
    instance: int,
    env: dict,
) -> str:
    px4_bin: Path = px4_dir / "build" / build / "bin" / "px4"
    return f"cd {px4_dir} && {_env_prefix(env)} {px4_bin} -i {instance}"


def _launch_session(
    commands: list[tuple[str, int, str]],  # name, instance, command
    session: str,
    delay: float,
    backend: str,
) -> None:
    forward_keys = [
        "DISPLAY",
        "WAYLAND_DISPLAY",
        "XDG_RUNTIME_DIR",
        "LD_LIBRARY_PATH",
        "GZ_SIM_RESOURCE_PATH",
        "GZ_SIM_SYSTEM_PLUGIN_PATH",
        "GZ_VERSION",
        "HOME",
        "PATH",
        "GZ_IP",
        "GZ_RELAY",
    ]

    env_exports = "; ".join(
        f"export {k}={shlex.quote(os.environ[k])}"
        for k in forward_keys
        if k in os.environ
    )

    # Start the first instance, which will spawn gz-server.
    name, instance, cmd = commands[0]
    subprocess.run(
        [
            backend,
            "new-session",
            "-d",
            "-s",
            session,
            "-n",
            name,
            "bash",
            "-c",
            f"{env_exports}; {cmd}",
        ],
        check=True,
    )

    # Server now exists — sync env into it
    for key in forward_keys:
        if key in os.environ:
            subprocess.run([backend, "set-environment", "-g", key, os.environ[key]])
    print(f"  [{name}] PX4 instance {instance}  (gz-server host)")

    # Wait a bit for gz-server to start up and be detectable before launching more
    # instances that will try to connect to it.
    if len(commands) > 1:
        _wait_for_gz_server(delay)

    # Launch the rest of the instances in new windows.
    for name, instance, cmd in commands[1:]:
        subprocess.run(
            [
                backend,
                "new-window",
                "-a",
                "-t",
                session,
                "-n",
                name,
                "bash",
                "-c",
                f"{env_exports}; {cmd}",
            ],
            check=True,
        )
        print(f"  [{name}] PX4 instance {instance}  (standalone, attaching)")

    # Print completion message and instruction to attach to the session and kill it.
    attach_cmd = "byobu" if backend == "byobu-tmux" else "tmux"
    print()
    print(f"session '{session}' is up.")
    print(f"  attach :  {attach_cmd} attach -t {session}")
    if backend == "byobu-tmux":
        print(f"  windows:  F3 / F4   (or   Ctrl-b n / Ctrl-b p)")
    else:
        print(f"  windows:  Ctrl-b 1/2/3   or   Ctrl-b n / Ctrl-b p")
    print(f"  kill   :  rerun this script with --kill")


def _wait_for_gz_server(timeout: float):
    deadline = time.monotonic() + timeout
    print(f"  waiting for gz-server readiness (timeout {timeout:g}s) ...")

    while time.monotonic() < deadline:
        if _gz_server_is_running():
            print("  gz-server is up")
            return
        time.sleep(0.5)

    print("  warning: gz-server was not detected before timeout; continuing anyway")


def _gz_server_is_running() -> bool:
    probes = (
        ["pgrep", "-f", r"gz sim"],
        ["pgrep", "-f", r"gz server"],
    )
    for probe in probes:
        result = subprocess.run(
            probe, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        if result.returncode == 0:
            return True
    return False


def _kill_everything(session: str):
    """
    Kill the session (under either backend) plus any stray sim processes.
    """
    # Try both backends regardless of what started the session.
    for backend in ("byobu-tmux", "tmux"):
        if shutil.which(backend):
            subprocess.run(
                [backend, "kill-session", "-t", session],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
    print(f"killed session '{session}' (if it existed)")

    # Kill leftover sim processes the multiplexer wouldn't have caught. In particular,
    # gz sim runs in its own process group spawned by PX4, so it can survive the tmux
    # session being killed.
    patterns = ["ign gazebo", "gz sim", "gz server", "px4 -i"]
    for pat in patterns:
        subprocess.run(
            ["pkill", "-f", pat],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    time.sleep(1.0)
    for pat in patterns:
        subprocess.run(
            ["pkill", "-9", "-f", pat],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    print(f"killed any leftover sim processes ({', '.join(patterns)})")

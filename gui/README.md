# Calibration GUI (frontend)

A single-window Tkinter front end for the calibration pipeline. It exposes one button per
pipeline stage — **Preprocess → SuperGlue Matching → Initial Guess → Calibrate** — runs whichever
one you click as a supervised subprocess, and streams its log output live. It does not implement
any calibration logic itself; it only shells out to the `ros2 run direct_visual_lidar_calibration
...` executables built by the [backend](../src) and tails their output.

See the [User Guide](../docs/user_guide.md) for the end-user walkthrough with screenshots.

## Requirements

- Python 3 with Tk bindings (`sudo apt install python3-tk` on Debian/Ubuntu — everything else the
  script uses, `os`/`queue`/`shlex`/`signal`/`subprocess`/`threading`/`time`, is in the standard
  library)
- A working install of the [backend](../src) — either a sourced ROS 2 workspace, or running
  inside the packaged AppImage

## Running

Outside an AppImage, from a terminal with your ROS 2 + workspace environment sourced:

```bash
python3 gui/calibration_gui.py
```

Inside the AppImage, `AppRun` launches this script directly with `$APPDIR` already set, which
switches command execution to run against the bundled environment instead of sourcing a
workspace `setup.bash` (see `RUNNING_IN_APPIMAGE` in `calibration_gui.py`).

## Configuration (environment variables)

| Variable | Default | Purpose |
|:---------|:--------|:--------|
| `VENV` | `~/venvs/dvcalib` | Python virtualenv to source (native/dev runs only) |
| `ROS_SETUP` | `/opt/ros/jazzy/setup.bash` | ROS 2 environment to source (native/dev runs only) |
| `WS_SETUP` | `~/ros2_ws/install/setup.bash` | Workspace overlay to source (native/dev runs only) |
| `BAGS_DIR` / `DATASET_DIR` | `~/calibration_bags_AMR` | Default dataset directory shown in the GUI |
| `OUTPUT_DIR` | `<dataset>/bag_processed` | Default output directory shown in the GUI |
| `KILL_GRACE` | `5` | Seconds between SIGTERM and SIGKILL when a step is stopped |

Inside the AppImage, `AppRun` sets `BAGS_DIR`/`OUTPUT_DIR` relative to the `.AppImage` file's own
location, so a `calibration_bags/` folder placed next to it is picked up automatically.

## Notes

- Only one pipeline step runs at a time (single log pane, single process supervisor), but any
  step can be started in any order — the GUI does not enforce running them 1→2→3→4.
- Steps that open their own native GLFW/Iridescence viewer (`preprocess -v`, `calibrate`) pop up
  as separate OS windows, exactly as they would from a terminal — this is expected, not an error.

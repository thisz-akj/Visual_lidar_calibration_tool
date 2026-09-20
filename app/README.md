# Pipeline runner (`run_pipeline.sh`)

A headless, dependency-free driver for the four calibration stages — no GUI, no manual
terminal-tab juggling. It runs `preprocess → find_matches_superglue.py → initial_guess_auto →
calibrate` in sequence, each as its own supervised step with its own timeout (these executables
don't exit on their own), logging every step to `<output>/logs/0N_<step>.log`, and finally checks
that `calib.json` was produced.

This is the same script the [GUI](../gui/README.md) and an AppImage-style bundle would use as
their non-interactive entrypoint; it can equally be run standalone against a native ROS 2
workspace build.

## Usage

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

./app/run_pipeline.sh --dataset /path/to/calibration_bags --output /path/to/output_dir
```

| Option | Default | Description |
|:-------|:--------|:-------------|
| `--dataset <dir>` | `$BAGS_DIR` | Directory containing `bag1/`, `bag2/`, ... |
| `--output <dir>` | `$OUTPUT_DIR` | Directory to write processed output, logs, and `calib.json` |
| `--preprocess-wait <sec>` | 480 | Timeout for the preprocess step |
| `--superglue-wait <sec>` | 30 | Timeout for the SuperGlue matching step |
| `--initial-guess-wait <sec>` | 30 | Timeout for the initial guess step |
| `--calibration-wait <sec>` | 300 | Timeout for the fine-registration step |
| `--kill-grace <sec>` | 5 | Grace period between SIGTERM and SIGKILL when a step's timeout expires |
| `--visualize` | off | Enable preprocess's live GLFW viewer (`-v`); needs a real X11 display |
| `--status-interval <sec>` | 10 | How often to print/refresh progress while a step is running |

`--dataset`/`--output` fall back to the `$BAGS_DIR`/`$OUTPUT_DIR` environment variables when
omitted — inside an AppImage-style bundle these are resolved relative to the bundle's own
location so a `calibration_bags/` folder placed next to it is picked up with no flags needed.

## Exit behavior

On success, `<output>/calib.json` exists and contains the final `T_lidar_camera` transform (see
[Pipeline Stages](../README.md#pipeline-stages) in the main README). If any step exits non-zero,
the script stops immediately and points you at that step's log; if all steps run to completion but
`calib.json` is still missing, the script reports the calibration as incomplete.

A live status line is also written to `<output>/.pipeline_status` (`STATE|step|total|title|elapsed|window`),
so another terminal can `cat` it to check progress without reading the full log.

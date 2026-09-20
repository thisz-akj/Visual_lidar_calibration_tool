#!/usr/bin/env python3
############################################################
# Direct Visual LiDAR Calibration — step-by-step GUI
#
# One button per pipeline step (Preprocess, SuperGlue Matching,
# Initial Guess, Calibrate). Every button stays clickable regardless
# of the others' status — run any step, any time, in any order — the
# only constraint is that just one step runs at a time (single log
# pane, single process supervisor). Each step's dot is grey until
# clicked, green while running or once it succeeds, red if it
# failed/was stopped.
#
# This GUI window only shows status + live logs. Steps that open
# their own native GLFW/Iridescence viewer (preprocess -v, calibrate)
# pop up as their own separate OS window, same as running from a
# terminal — embedding them into this window was tried and reverted
# (the window manager kept reclaiming the reparented window).
#
# AppImage note: when $APPDIR is set (always true inside the AppImage —
# AppRun exports it before launching this script), steps are run as
# plain subprocesses with no extra sourcing. AppRun already fully sealed
# PYTHONHOME/PYTHONPATH/LD_LIBRARY_PATH/AMENT_PREFIX_PATH/PATH to the
# bundle before this GUI process even started, and every subprocess it
# spawns inherits that same environment automatically — there is no
# venv/ROS/workspace setup.bash to source inside the bundle at all.
#
# Author : Azad Kumar Jha
# ROS    : Jazzy
############################################################

import os
import queue
import shlex
import signal
import subprocess
import threading
import time
import tkinter as tk
from tkinter import filedialog, scrolledtext, ttk

HOME = os.path.expanduser("~")

RUNNING_IN_APPIMAGE = "APPDIR" in os.environ

VENV = os.environ.get("VENV", os.path.join(HOME, "venvs", "dvcalib"))
ROS_SETUP = os.environ.get("ROS_SETUP", "/opt/ros/jazzy/setup.bash")
WS_SETUP = os.environ.get("WS_SETUP", os.path.join(HOME, "ros2_ws", "install", "setup.bash"))

GTSAM_LD_LIBRARY_PATH = (
    f"{HOME}/gtsam/build:{HOME}/gtsam/build/gtsam:"
    f"{HOME}/gtsam/build/gtsam/3rdparty/metis/libmetis"
)

KILL_GRACE = int(os.environ.get("KILL_GRACE", 5))

# AppRun exports BAGS_DIR/OUTPUT_DIR resolved relative to the .AppImage
# file's own location; DATASET_DIR is the native-dev-machine convention
# used outside the AppImage.
DEFAULT_DATASET_DIR = os.environ.get("BAGS_DIR", os.environ.get("DATASET_DIR", os.path.join(HOME, "calibration_bags_AMR")))
DEFAULT_OUTPUT_DIR = os.environ.get("OUTPUT_DIR", os.path.join(DEFAULT_DATASET_DIR, "bag_processed"))

# Status dot color: grey until clicked, green while running/once succeeded,
# red if it failed or was stopped.
DOT_COLORS = {
    "PENDING": "#888888",
    "RUNNING": "#2e7d32",
    "SUCCESS": "#2e7d32",
    "FAILED": "#c62828",
    "STOPPED": "#c62828",
}


def build_wrapped_command(cmd):
    """Wrap a ros2 command in a bash -c string that sources the venv,
    ROS, and workspace exactly like run_calibration_1.sh / run_pipeline.sh
    do. Python can't source bash env files directly, so bash still does
    that part; process supervision + log tailing is done in Python so
    the GUI can update live.

    Inside the AppImage there is nothing to source — AppRun already sealed
    the whole environment before this process started — so just exec the
    command directly and let it inherit that environment as-is."""
    quoted_cmd = " ".join(shlex.quote(c) for c in cmd)

    if RUNNING_IN_APPIMAGE:
        return f"exec {quoted_cmd}\n"

    return f"""
set +u
source {shlex.quote(os.path.join(VENV, 'bin', 'activate'))}
source {shlex.quote(ROS_SETUP)}
source {shlex.quote(WS_SETUP)}
set -u
export LD_LIBRARY_PATH="{GTSAM_LD_LIBRARY_PATH}${{LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}}"
exec {quoted_cmd}
"""


class StepRunner:
    """Runs a single pipeline step in a background thread, posting
    (event, payload) messages onto a queue the Tk main loop polls."""

    def __init__(self, event_queue):
        self.q = event_queue
        self._proc = None
        self._stop_requested = False

    def start(self, idx, title, cmd, logpath):
        self._stop_requested = False
        thread = threading.Thread(target=self._run, args=(idx, title, cmd, logpath), daemon=True)
        thread.start()

    def stop(self):
        self._stop_requested = True
        if self._proc is not None and self._proc.poll() is None:
            self._terminate(self._proc)

    def _terminate(self, proc):
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except ProcessLookupError:
            return
        time.sleep(KILL_GRACE)
        if proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass

    def _run(self, idx, title, cmd, logpath):
        self.q.put(("log_reset", f"=== {title} ===\n$ {' '.join(cmd)}\n\n"))

        wrapped = build_wrapped_command(cmd)
        with open(logpath, "wb") as logfh:
            proc = subprocess.Popen(
                ["bash", "-c", wrapped],
                stdout=logfh,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,
            )
            self._proc = proc

            tail_thread = threading.Thread(target=self._tail_log, args=(logpath, proc), daemon=True)
            tail_thread.start()

            ret = proc.wait()
            tail_thread.join(timeout=2)

        if self._stop_requested:
            self.q.put(("step_done", (idx, "STOPPED")))
        elif ret == 0:
            self.q.put(("step_done", (idx, "SUCCESS")))
        else:
            self.q.put(("log", f"\n{title} FAILED (exit {ret}) — see {logpath}\n"))
            self.q.put(("step_done", (idx, "FAILED")))

    def _tail_log(self, path, proc):
        for _ in range(50):
            if os.path.exists(path):
                break
            time.sleep(0.1)
        try:
            with open(path, "r", errors="replace") as f:
                while True:
                    line = f.readline()
                    if line:
                        self.q.put(("log", line))
                        continue
                    if proc.poll() is not None:
                        remainder = f.read()
                        if remainder:
                            self.q.put(("log", remainder))
                        break
                    time.sleep(0.2)
        except FileNotFoundError:
            pass


class CalibrationGUI:
    def __init__(self, root):
        self.root = root
        root.title("Direct LiDAR Camera Calibration")
        root.geometry("900x680")
        root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.event_queue = queue.Queue()
        self.runner = StepRunner(self.event_queue)
        self.active_idx = None

        self._build_paths_frame()
        self._build_steps_frame()
        self._build_log_frame()

        self._update_gating()
        self.root.after(150, self.poll_queue)

    # ---- layout -------------------------------------------------

    def _build_paths_frame(self):
        frame = ttk.Frame(self.root, padding=8)
        frame.pack(fill="x")

        self.dataset_var = tk.StringVar(value=DEFAULT_DATASET_DIR)
        self.output_var = tk.StringVar(value=DEFAULT_OUTPUT_DIR)
        self.dataset_var.trace_add("write", lambda *_: self._on_paths_changed())
        self.output_var.trace_add("write", lambda *_: self._on_paths_changed())

        ttk.Label(frame, text="Dataset dir:").grid(row=0, column=0, sticky="w")
        ttk.Entry(frame, textvariable=self.dataset_var, width=60).grid(row=0, column=1, sticky="we", padx=4)
        ttk.Button(frame, text="Browse...", command=self._browse_dataset).grid(row=0, column=2)

        ttk.Label(frame, text="Output dir:").grid(row=1, column=0, sticky="w")
        ttk.Entry(frame, textvariable=self.output_var, width=60).grid(row=1, column=1, sticky="we", padx=4)
        ttk.Button(frame, text="Browse...", command=self._browse_output).grid(row=1, column=2)

        frame.columnconfigure(1, weight=1)

    def _build_steps_frame(self):
        frame = ttk.Frame(self.root, padding=8)
        frame.pack(fill="x")

        self.visualize_preprocess = tk.BooleanVar(value=True)
        self.visualize_calibrate = tk.BooleanVar(value=True)

        self.steps = [
            {
                "title": "1. Preprocess",
                "logfile": "01_preprocess.log",
                "build_cmd": lambda: [
                    "ros2", "run", "direct_visual_lidar_calibration", "preprocess",
                    self.dataset_var.get(), self.output_var.get(), "-a", "-d",
                ] + (["-v", "--auto_quit"] if self.visualize_preprocess.get() else []),
                "extra_widget": lambda row: ttk.Checkbutton(
                    row, text="show live viewer (-v)", variable=self.visualize_preprocess
                ),
            },
            {
                "title": "2. SuperGlue Matching",
                "logfile": "02_superglue.log",
                "build_cmd": lambda: [
                    "ros2", "run", "direct_visual_lidar_calibration",
                    "find_matches_superglue.py", self.output_var.get(),
                ],
                "extra_widget": None,
            },
            {
                "title": "3. Initial Guess",
                "logfile": "03_initial_guess.log",
                "build_cmd": lambda: [
                    "ros2", "run", "direct_visual_lidar_calibration",
                    "initial_guess_auto", self.output_var.get(),
                ],
                "extra_widget": None,
            },
            {
                "title": "4. Calibrate",
                "logfile": "04_calibration.log",
                "build_cmd": lambda: [
                    "ros2", "run", "direct_visual_lidar_calibration", "calibrate", self.output_var.get(), "--auto_quit",
                ] + ([] if self.visualize_calibrate.get() else ["--background"]),
                "extra_widget": lambda row: ttk.Checkbutton(
                    row, text="show viewer", variable=self.visualize_calibrate
                ),
            },
        ]

        self.status = ["PENDING"] * len(self.steps)
        self.start_buttons = []
        self.status_labels = []

        for i, step in enumerate(self.steps):
            row = ttk.Frame(frame)
            row.pack(fill="x", pady=2)

            ttk.Label(row, text=step["title"], width=22, anchor="w").pack(side="left")

            dot_lbl = tk.Label(row, text="●", fg=DOT_COLORS["PENDING"], width=2, anchor="w")
            dot_lbl.pack(side="left")
            self.status_labels.append(dot_lbl)

            btn = ttk.Button(row, text="Start", command=lambda i=i: self.on_start(i))
            btn.pack(side="left", padx=4)
            self.start_buttons.append(btn)

            if step["extra_widget"] is not None:
                step["extra_widget"](row).pack(side="left", padx=8)

        ctrl = ttk.Frame(frame)
        ctrl.pack(fill="x", pady=(8, 0))
        self.stop_btn = ttk.Button(ctrl, text="Stop running step", command=self.on_stop, state="disabled")
        self.stop_btn.pack(side="left")
        self.overall_status = ttk.Label(ctrl, text="")
        self.overall_status.pack(side="left", padx=12)

    def _build_log_frame(self):
        frame = ttk.Frame(self.root, padding=8)
        frame.pack(fill="both", expand=True)
        ttk.Label(frame, text="Log (currently selected/running step):").pack(anchor="w")
        self.log_text = scrolledtext.ScrolledText(frame, wrap="word", height=28, state="disabled")
        self.log_text.pack(fill="both", expand=True)

    # ---- helpers --------------------------------------------------

    def _browse_dataset(self):
        d = filedialog.askdirectory(initialdir=self.dataset_var.get() or HOME)
        if d:
            self.dataset_var.set(d)

    def _browse_output(self):
        d = filedialog.askdirectory(initialdir=self.output_var.get() or HOME)
        if d:
            self.output_var.set(d)

    def _on_paths_changed(self):
        # Editing dataset/output invalidates any downstream progress —
        # re-running against different data shouldn't look "already done".
        if self.active_idx is None:
            self.status = ["PENDING"] * len(self.steps)
            for lbl in self.status_labels:
                lbl.configure(fg=DOT_COLORS["PENDING"])
            self._update_gating()

    def _update_gating(self):
        # Every step is independently runnable at any time, in any order —
        # the only real constraint is that just one step can run at once
        # (single log pane, single process supervisor).
        paths_ready = bool(self.dataset_var.get()) and bool(self.output_var.get())
        for btn in self.start_buttons:
            btn.configure(state="normal" if (paths_ready and self.active_idx is None) else "disabled")

    def append_log(self, text):
        self.log_text.configure(state="normal")
        self.log_text.insert("end", text)
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def reset_log(self, header):
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.insert("end", header)
        self.log_text.configure(state="disabled")

    def _set_status(self, idx, status):
        self.status[idx] = status
        self.status_labels[idx].configure(fg=DOT_COLORS.get(status, "#000000"))

    # ---- actions ----------------------------------------------------

    def on_start(self, idx):
        dataset_dir = self.dataset_var.get()
        output_dir = self.output_var.get()
        if not dataset_dir or not output_dir or self.active_idx is not None:
            return

        self._launch(idx)
        self._update_gating()

    def _launch(self, idx):
        output_dir = self.output_var.get()
        log_dir = os.path.join(output_dir, "logs")
        os.makedirs(output_dir, exist_ok=True)
        os.makedirs(log_dir, exist_ok=True)

        step = self.steps[idx]
        cmd = step["build_cmd"]()
        logpath = os.path.join(log_dir, step["logfile"])

        self.active_idx = idx
        self._set_status(idx, "RUNNING")
        self.overall_status.configure(text=f"Running: {step['title']}")
        self.stop_btn.configure(state="normal")

        self._update_gating()
        self.runner.start(idx, step["title"], cmd, logpath)

    def on_stop(self):
        self.runner.stop()
        self.overall_status.configure(text="Stopping...")

    def on_close(self):
        self.runner.stop()
        self.root.after(300, self.root.destroy)

    def poll_queue(self):
        try:
            while True:
                kind, payload = self.event_queue.get_nowait()
                if kind == "log":
                    self.append_log(payload)
                elif kind == "log_reset":
                    self.reset_log(payload)
                elif kind == "step_done":
                    idx, status = payload
                    self._set_status(idx, status)
                    self.active_idx = None
                    self.stop_btn.configure(state="disabled")
                    self.overall_status.configure(text=f"{self.steps[idx]['title']}: {status}")
                    self._update_gating()
        except queue.Empty:
            pass
        self.root.after(150, self.poll_queue)


def main():
    root = tk.Tk()
    CalibrationGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()

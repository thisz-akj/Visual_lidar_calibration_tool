# Visual LiDAR Calibration Tool

## Overview
This repository implements a targetless camera–LiDAR extrinsic calibration pipeline designed for real-world robotic and autonomous systems. The system estimates the rigid transformation between a monocular camera and a 3D LiDAR sensor using synchronized ROS2 bag data, eliminating the need for calibration targets.

The pipeline integrates learned feature matching, geometric optimization, and statistical refinement to produce robust and accurate calibration under challenging conditions such as noise, sparsity, dynamic scenes, and varying illumination.

Two ways to run it ship in this repository:
- A **desktop GUI** (`gui/`) — one button per pipeline stage, live logs, no terminal required. See the [User Guide](docs/user_guide.md).
- A **headless CLI runner** (`app/run_pipeline.sh`) — drives all four stages end-to-end for scripted/batch use.

---

## Repository Layout

```
Visual_lidar_calibration_tool/
├── src/, include/                 # Backend: C++ calibration engine (camera models, vlcal core, CLI entrypoints)
├── direct_visual_lidar_calibration/ # Backend: ROS 2 Python package namespace
├── scripts/                       # Backend: Python helper (SuperGlue feature matching)
├── cmake/                         # Backend: CMake find-modules (FindGTSAM.cmake)
├── thirdparty/                    # Backend: vendored deps (nlohmann/json, nanoflann, Sophus)
├── CMakeLists.txt                 # Backend: build definition (ament_cmake / catkin)
├── docker/jazzy/                  # Backend: containerized ROS 2 (jazzy) build
│
├── gui/                           # Frontend: Tkinter desktop GUI (calibration_gui.py)
├── app/                           # App layer: headless pipeline entrypoint (run_pipeline.sh)
│
├── docs/                          # User guide (Markdown + PDF) and screenshots
├── LICENSE
└── README.md
```

| Layer | Directory | Role |
|:------|:----------|:-----|
| Backend | `src/`, `include/`, `cmake/`, `thirdparty/`, `scripts/`, `direct_visual_lidar_calibration/`, `CMakeLists.txt` | The calibration engine itself: camera models, preprocessing, matching, optimization, and the `preprocess` / `find_matches_superglue.py` / `initial_guess_auto` / `calibrate` / `viewer` executables it builds. Packaged as a single ROS 2 (`ament_cmake`) / ROS 1 (`catkin`) package. |
| Frontend | `gui/` | Desktop GUI that drives the backend executables as subprocesses and streams their logs. See [`gui/README.md`](gui/README.md). |
| App | `app/` | Headless orchestration script that runs the same four stages non-interactively, with per-step timeouts and logging. See [`app/README.md`](app/README.md). |
| Docs | `docs/` | End-user guide with screenshots, in both Markdown and PDF form. |

> **Build note:** this repository does not currently include a ROS 2 `package.xml` manifest. `ament_auto_find_build_dependencies()` in `CMakeLists.txt` reads that manifest to resolve dependencies, so add a `package.xml` (declaring `pcl`, `opencv`, `gtsam`, `ceres`, `cv_bridge`, `sensor_msgs`, `rclcpp`, etc. as dependencies) before running `colcon build`.

---

## Problem Statement
Accurate extrinsic calibration between camera and LiDAR is critical for multi-sensor perception systems used in robotics and ADAS. Traditional calibration methods rely on calibration targets, which are impractical in production environments.

This project addresses:
- Targetless calibration in unstructured environments
- Robust alignment under noisy and sparse LiDAR data
- Real-world deployment using ROS2 sensor pipelines
- Making the pipeline approachable for non-experts via a point-and-click GUI, on top of the scriptable CLI stages

Formally, the objective is to recover the rigid-body transform $T_{lidar}^{camera} \in SE(3)$ relating the LiDAR frame $L$ and camera frame $C$, such that a LiDAR point $p_L \in \mathbb{R}^3$ maps to the camera frame as $p_C = T_{camera}^{lidar} \, p_L$, and its pixel projection is $u = \pi(p_C)$ under the camera's intrinsic model $\pi(\cdot)$ (pinhole, fisheye, or omnidirectional). $T_{camera}^{lidar}$ has 6 degrees of freedom (3 rotation + 3 translation) and is estimated in three successive stages of increasing accuracy and decreasing convergence basin, described below.

---

## Method

### 1. Correspondence generation
For each synchronized LiDAR/camera pair, the accumulated LiDAR sweep is rendered into a range/intensity image via spherical projection, and 2D keypoints are matched between this LiDAR-intensity image and the camera image using **SuperGlue**, a graph-neural-network feature matcher trained for wide-baseline, low-texture correspondence. Each matched keypoint pair $(u_{cam}, u_{lidar})$ is back-projected through the stored LiDAR pixel index map to recover a 2D–3D correspondence set $\mathcal{C} = \{(u_i, p_i)\}_{i=1}^{N}$, $u_i \in \mathbb{R}^2$, $p_i \in \mathbb{R}^3$.

### 2. Initial pose estimation
The initial estimate of $T_{camera}^{lidar}$ is computed in two passes over $\mathcal{C}$:

1. **RANSAC rotation search.** For each of $K$ iterations (default $K = 8192$), a minimal sample of 2 correspondences is drawn, and a closed-form least-squares rotation $R \in SO(3)$ between the corresponding camera bearing vectors and LiDAR direction vectors is recovered via the Umeyama/Kabsch SVD solution:
   $$R = U \, \mathrm{diag}(1, 1, \det(UV^\top)) \, V^\top, \qquad U \Sigma V^\top = \mathrm{SVD}(A B^\top)$$
   where $A, B$ are the stacked unit bearing/direction vectors. The rotation with the largest reprojection-error inlier count (default threshold: 10 px) is kept.
2. **Nonlinear reprojection refinement.** The full $SE(3)$ pose is then refined by minimizing total reprojection error over all correspondences with a Cauchy robust loss ($c = 10$ px) via Ceres Solver, parameterizing the pose on the $SE(3)$ manifold (Sophus):
   $$T^\star = \arg\min_{T \in SE(3)} \sum_{i=1}^{N} \rho\!\left(\lVert \pi(T \, p_i) - u_i \rVert^2\right)$$

### 3. Fine registration via Normalized Information Distance (NID)
The initial estimate is refined by directly maximizing the statistical dependency between camera image intensity and LiDAR reflectivity/intensity, without requiring further point correspondences. For a candidate pose $T$, camera pixel intensities $r$ and re-projected LiDAR intensities $s$ are jointly binned into a $B \times B$ histogram (default $B = 16$) to estimate the joint and marginal entropies $H(r,s)$, $H(r)$, $H(s)$, and hence the mutual information $I(r;s) = H(r) + H(s) - H(r,s)$. The registration objective is the **Normalized Information Distance**:
$$\mathrm{NID}(T) = \frac{H(r,s) - I(r;s)}{H(r,s)} = 1 - \frac{I(r;s)}{H(r,s)}$$
$\mathrm{NID} \in [0, 1]$ is minimized over $T \in SE(3)$ using either BFGS or Nelder-Mead (`--registration_type`), with bicubic B-spline interpolation of the image intensity field so the histogram — and therefore the cost — remains differentiable with respect to the projected pixel location.

### 4. Cross-modal validation
The resulting $T_{lidar}^{camera}$ is validated by re-projecting the LiDAR point cloud into the image plane and checking geometric consistency against independent visual cues (instance/segmentation masks): point-cloud-to-mask overlap ratio and centroid displacement, reported in [Results](#results) below.

---

## Key Features

### Targetless Calibration
- No checkerboards or fiducial markers required
- Works directly with natural scene geometry

### ROS2-Based Sensor Pipeline
- Uses ROS2 bag files as primary input
- Supports standard topics:
  - `/camera/image_raw`
  - `/lidar/points`
- Designed for real robotic deployments

### Learned Feature Matching
- SuperGlue-based matching for robust correspondence
- Handles viewpoint variation and low-texture regions

### Multi-Stage Optimization Pipeline
- RANSAC for outlier rejection
- SVD-based rigid transformation initialization
- Non-linear optimization using Ceres Solver
- Final refinement using Normalized Information Distance (NID)

### Cross-Modal Validation
- Projects LiDAR point clouds into image space
- Evaluates calibration using:
  - segmentation alignment
  - centroid consistency

### Point-and-Click Desktop GUI
- One button per pipeline stage, run any stage in any order
- Live log tailing and per-step status indicators
- Works against a native ROS 2 workspace or a bundled/AppImage-style environment (`$APPDIR`-aware)

---

## Results

<p align="center">
  <img src="https://github.com/user-attachments/assets/1cfce0d2-1bd8-4906-b6da-4f01d737f1c5" width="45%" />
  <img src="https://github.com/user-attachments/assets/fa01da7c-16a8-4230-9721-47ea01779778" width="45%" />
</p>

### Reprojection Testing:

**Achieved >95% point-cloud-to-mask overlap with centroid proximity across multiple sensor configurations**
<p align="center">
  <img src="https://github.com/user-attachments/assets/04d8a4e1-3fb3-4cde-a564-4612fea27c93" width="45%" />
  <img src="https://github.com/user-attachments/assets/fa7cd41a-ec2d-4144-b555-cdaf4a3ddca4" width="45%" />
</p>

---

## Pipeline Stages

The calibration pipeline runs as a sequence of CLI stages, each consuming the output of the previous one:

| Stage | Command | Purpose |
|:------|:--------|:--------|
| 1. Preprocess | `preprocess <bag_dir> <out_dir>` | Extracts/accumulates dense point clouds and images from a ROS2 bag |
| 2. Feature matching | `find_matches_superglue.py <out_dir>` | Finds 2D–3D correspondences between LiDAR intensity images and camera images via SuperGlue |
| 3. Initial guess | `initial_guess_auto <out_dir>` (or `initial_guess_manual`) | RANSAC + SVD-based initial extrinsic estimate from the matches |
| 4. Fine registration | `calibrate <out_dir>` | Ceres-based NID optimization to refine the extrinsic transform |
| 5. Inspection | `viewer <out_dir>` | Visual/cross-modal validation of the final calibration |

Each stage writes into a shared `calib.json` in the output directory, ending with the final `T_lidar_camera` transform.

Stages 1–4 are exactly what [`app/run_pipeline.sh`](app/README.md) and the [desktop GUI](gui/README.md) drive end-to-end. Stage 5 (`viewer`) is a separate manual inspection step, not part of that automated run.

---

## Running the Pipeline

### Option A — Desktop GUI
The friendliest entry point: one window, one button per stage, live logs, no terminal commands to remember. Full walkthrough with screenshots in the [User Guide](docs/user_guide.md).

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
python3 gui/calibration_gui.py
```

### Option B — Headless CLI runner
For scripted/batch runs (CI, remote machines, no display):

```bash
./app/run_pipeline.sh --dataset /path/to/calibration_bags --output /path/to/output_dir
```

See [`app/README.md`](app/README.md) for all options (per-step timeouts, `--visualize`, etc.).

Both entry points are written to also work when bundled into a self-contained AppImage-style
distribution: they detect an `$APPDIR` environment variable and, when set, skip sourcing any
workspace `setup.bash` and run directly against whatever environment the bundle already sealed in
place. Building such a bundle is not part of this repository — `docker/jazzy/` covers building a
containerized ROS 2 backend instead.

---

## Dependencies (native / source build)

- [ROS1/ROS2](https://www.ros.org/)
- [PCL](https://pointclouds.org/)
- [OpenCV](https://opencv.org/)
- [GTSAM](https://gtsam.org/)
- [Ceres](http://ceres-solver.org/)
- [Iridescence](https://github.com/koide3/iridescence)
- [SuperGlue](https://github.com/magicleap/SuperGluePretrainedNetwork) [optional, non-commercial use only]
- `python3-tk` — required only for the [desktop GUI](gui/README.md)

See [`docker/jazzy/Dockerfile`](docker/jazzy/Dockerfile) (and the `..._with_superglue` variant) for a known-working dependency set and build sequence on ROS 2 jazzy.

---

## Attribution

The core calibration algorithm (preprocessing, SuperGlue-based matching, RANSAC/SVD initial guess, Ceres/NID fine registration) originates from [koide3/direct_visual_lidar_calibration](https://github.com/koide3/direct_visual_lidar_calibration) by Kenji Koide (AIST), released under the MIT license:

> Koide et al., *General, Single-shot, Target-less, and Automatic LiDAR-Camera Extrinsic Calibration Toolbox*, ICRA2023. [[PDF]](https://staff.aist.go.jp/k.koide/assets/pdf/icra2023.pdf)

The desktop GUI (`gui/`), headless pipeline runner (`app/`), and this documentation are additions on top of that project to make the tool usable end-to-end without hand-typing ROS commands.

## References

1. K. Koide, S. Oishi, M. Yokozuka, A. Banno. *General, Single-shot, Target-less, and Automatic LiDAR-Camera Extrinsic Calibration Toolbox.* ICRA 2023. [[PDF]](https://staff.aist.go.jp/k.koide/assets/pdf/icra2023.pdf)
2. P.-E. Sarlin, D. DeTone, T. Malisiewicz, A. Rabinovich. *SuperGlue: Learning Feature Matching with Graph Neural Networks.* CVPR 2020. [[arXiv:1911.11763]](https://arxiv.org/abs/1911.11763)
3. S. Umeyama. *Least-Squares Estimation of Transformation Parameters Between Two Point Patterns.* IEEE TPAMI, 1991.

## License

MIT — see [`LICENSE`](LICENSE). SuperGlue is bundled/used optionally under its own non-commercial research license; review its terms before commercial use.

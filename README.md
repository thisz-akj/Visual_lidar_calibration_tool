# Visual LiDAR Calibration Tool

## Overview
This repository implements a targetless camera–LiDAR extrinsic calibration pipeline designed for real-world robotic and autonomous systems. The system estimates the rigid transformation between a monocular camera and a 3D LiDAR sensor using synchronized ROS2 bag data, eliminating the need for calibration targets.

The pipeline integrates learned feature matching, geometric optimization, and statistical refinement to produce robust and accurate calibration under challenging conditions such as noise, sparsity, dynamic scenes, and varying illumination.

To make the tool usable outside of a full ROS2 development environment, this repository also includes a **packaging pipeline** (`packaging/`) that builds the entire stack — ROS2, GTSAM, Ceres, Iridescence, SuperGlue, and the calibration workspace — into a single self-contained **AppImage**, so it can be run on any Ubuntu 24.04 (or compatible) machine without a ROS2 install.

---

## Problem Statement
Accurate extrinsic calibration between camera and LiDAR is critical for multi-sensor perception systems used in robotics and ADAS. Traditional calibration methods rely on calibration targets, which are impractical in production environments.

This project addresses:
- Targetless calibration in unstructured environments
- Robust alignment under noisy and sparse LiDAR data
- Real-world deployment using ROS2 sensor pipelines
- Distributing the pipeline as a single portable binary instead of a multi-hour ROS2/GTSAM/Ceres source build

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

### Portable AppImage Packaging
- One-command Docker-based build turns the full pipeline into a single `.AppImage`
- No system-wide ROS2, GTSAM, Ceres, or Python environment required on the target machine
- Bundled CPU-only PyTorch stack for SuperGlue inference (no CUDA dependency)

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

Each stage writes into a shared `calib.json` in the output directory, ending with the final `T_lidar_camera` transform. See [`docs/programs.md`](docs/programs.md) and [`docs/example.md`](docs/example.md) for the full CLI options and a worked example.

Stages 1–4 are exactly what the [AppImage's automated runner](#appimage-packaging) drives end-to-end. Stage 5 (`viewer`) is a separate manual inspection step, not part of that automated run.

---

## AppImage Packaging

The `packaging/` directory contains a self-contained Docker-based build system that produces a portable `dvcalib-pipeline-<target>-x86_64.AppImage`, bundling everything the pipeline needs — ROS2 (jazzy), GTSAM, Ceres, Iridescence, SuperGlue, and a CPU-only PyTorch/OpenCV/NumPy stack — so the calibration tool can be run on a target machine with no ROS2 or build toolchain installed.

### How it works

| File | Role |
|:-----|:-----|
| `Dockerfile.jazzy` | Builds a "donor" image: ROS2 jazzy base + GTSAM 4.2a9, Ceres, Iridescence built from source, the calibration workspace (`colcon build`), and SuperGlue cloned alongside it |
| `build_appdir.sh` | Runs inside the donor container. Stages an `AppDir` by resolving the apt dependency closure (`dpkg -L`), copying ROS2 + the built workspace, copying the source-built libraries, bundling SuperGlue and a CPU PyTorch install, then runs an in-build sanity check before packaging with `appimagetool` |
| `AppRun` | The AppImage entrypoint. Seals `PYTHONHOME`/`PYTHONPATH`/`LD_LIBRARY_PATH`/`AMENT_PREFIX_PATH` to the bundled ROS2/Python/GTSAM/Ceres/torch so nothing from the host environment leaks in, then hands off to `app/run_pipeline.sh` |
| `dvcalib-pipeline.desktop` | Desktop entry metadata for the AppImage |
| `build.sh` | Host-side driver: builds the donor image (cached after the first run) and runs the container to stage + package the AppImage into `../dist/` |

`app/run_pipeline.sh` (bundled into the AppImage and copied into it during staging) drives the pipeline end-to-end: it runs each stage — `preprocess` → `find_matches_superglue.py` → `initial_guess_auto` → `calibrate` — as a supervised step with its own timeout, logging each step to `<output>/logs/`, then verifies that `calib.json` was produced.

### Building the AppImage

```bash
cd packaging
./build.sh jazzy
```

This produces `dist/dvcalib-pipeline-jazzy-x86_64.AppImage`. The donor Docker image is cached, so subsequent builds only re-run the staging/packaging step unless the Dockerfile changes.

### Running the AppImage

The AppImage runs the whole calibration pipeline end-to-end from a single command — point it at a directory of calibration bags and an output directory, and it drives `preprocess` → SuperGlue matching → `initial_guess_auto` → `calibrate` automatically, stopping each step once it finishes (or once its timeout elapses):

```bash
chmod +x dvcalib-pipeline-jazzy-x86_64.AppImage

./dvcalib-pipeline-jazzy-x86_64.AppImage --dataset livox --output livox_preprocessed
```

| Option | Description |
|:-------|:-------------|
| `--dataset <dir>` | **Required.** Directory containing the calibration bags (e.g. `bag1`, `bag2`, ...) |
| `--output <dir>` | **Required.** Directory to write processed output, logs, and the final `calib.json` |
| `--preprocess-wait <sec>` | Timeout for the preprocess step (default 180) |
| `--superglue-wait <sec>` | Timeout for the SuperGlue matching step (default 180) |
| `--initial-guess-wait <sec>` | Timeout for the initial guess step (default 180) |
| `--calibration-wait <sec>` | Timeout for the fine-registration step (default 180) |
| `--kill-grace <sec>` | Grace period between SIGTERM and SIGKILL when a step's timeout expires (default 5) |
| `--visualize` | Enable preprocess's live GLFW viewer (`-v`). Needs a real X11 display — off by default so the pipeline can run headless/in containers |

Each step's output is logged to `<output>/logs/0N_<step>.log`. On success, the run finishes with `<output>/calib.json` containing the final `T_lidar_camera` transform (see [Pipeline Stages](#pipeline-stages) above); if `calib.json` is missing at the end, the run reports the calibration as incomplete and points you at the logs.

A few debug sub-commands are also built into `AppRun` itself:

```bash
./dvcalib-pipeline-jazzy-x86_64.AppImage --version   # print bundled ROS distro / Python version / build date
./dvcalib-pipeline-jazzy-x86_64.AppImage --python    # run the bundled Python interpreter directly
./dvcalib-pipeline-jazzy-x86_64.AppImage --shell     # drop into a shell with the bundled environment sourced
```

### Requirements on the target machine

- x86_64 Linux with FUSE (or run with `--appimage-extract-and-run`)
- No ROS2, GTSAM, Ceres, or Python environment needs to be pre-installed — everything is bundled

---

## Dependencies (native / source build)

- [ROS1/ROS2](https://www.ros.org/)
- [PCL](https://pointclouds.org/)
- [OpenCV](https://opencv.org/)
- [GTSAM](https://gtsam.org/)
- [Ceres](http://ceres-solver.org/)
- [Iridescence](https://github.com/koide3/iridescence)
- [SuperGlue](https://github.com/magicleap/SuperGluePretrainedNetwork) [optional, non-commercial use only]

If you don't want to build these from source, use the [AppImage](#appimage-packaging) instead.

---

## Attribution

The core calibration algorithm (preprocessing, SuperGlue-based matching, RANSAC/SVD initial guess, Ceres/NID fine registration) originates from [koide3/direct_visual_lidar_calibration](https://github.com/koide3/direct_visual_lidar_calibration) by Kenji Koide (AIST), released under the MIT license:

> Koide et al., *General, Single-shot, Target-less, and Automatic LiDAR-Camera Extrinsic Calibration Toolbox*, ICRA2023. [[PDF]](https://staff.aist.go.jp/k.koide/assets/pdf/icra2023.pdf)

The AppImage packaging pipeline (`packaging/`) is an addition on top of that project to make the tool distributable as a single portable binary.

## References

1. K. Koide, S. Oishi, M. Yokozuka, A. Banno. *General, Single-shot, Target-less, and Automatic LiDAR-Camera Extrinsic Calibration Toolbox.* ICRA 2023. [[PDF]](https://staff.aist.go.jp/k.koide/assets/pdf/icra2023.pdf)
2. P.-E. Sarlin, D. DeTone, T. Malisiewicz, A. Rabinovich. *SuperGlue: Learning Feature Matching with Graph Neural Networks.* CVPR 2020. [[arXiv:1911.11763]](https://arxiv.org/abs/1911.11763)
3. S. Umeyama. *Least-Squares Estimation of Transformation Parameters Between Two Point Patterns.* IEEE TPAMI, 1991.

## License

MIT — see the original project's license terms.

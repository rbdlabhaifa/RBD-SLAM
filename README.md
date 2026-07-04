# RBD-SLAM

> **Repository:** [rbdlabhaifa/RBD-SLAM](https://github.com/rbdlabhaifa/RBD-SLAM)

## What is this project?

**RBD-SLAM** is a **C++ autonomous drone navigation system** from the [RBD Lab, University of Haifa](https://github.com/rbdlabhaifa). It combines **ORB-SLAM3** (monocular visual SLAM) with **PCL point-cloud processing**, **GEOS polygon geometry**, and **RRT path planning** to fly a **DJI Tello**, build a 3D map of a room, detect exit points, and navigate toward unexplored areas.

The project targets **indoor autonomous exploration** — finding doorways / points of interest and planning collision-aware paths — using a live camera stream from the Tello (or a webcam for dry runs).

## What does it do?

1. **Stream video** — Tello camera or webcam at 640×480 via a lock-free frame queue (`streamer`).
2. **Track & map** — ORB-SLAM3 builds a sparse 3D map and estimates drone pose in real time.
3. **Cluster obstacles** — KMeans + GEOS convex hulls turn sparse SLAM points into 2D polygon obstacles.
4. **Find exits / goals** — `goal_finder` heuristics pick candidate doorway points from the map.
5. **Plan paths** — RRT on a fitted flight plane, with LEMON graph DFS for path extraction (`path_builder`).
6. **Fly the plan** — `navigator` sends ctello commands to follow waypoints; saves scan artifacts to timestamped directories.
7. **Visualize offline** — `show_rrt` (PCL OpenGL), `plot.py` (Plotly 3D), MeshLab for `tree.xyz`.

**Typical workflow for a new developer:**

```bash
git clone --recursive https://github.com/rbdlabhaifa/RBD-SLAM.git
cd RBD-SLAM
cp config.json.example config.json          # edit paths
cd Configuration/Vocabulary && tar xvfz ORBvoc.txt.tar.gz && cd ../..
./install_dependencies.sh                 # Ubuntu deps (partial — see §3)
cd external/ORB_SLAM3 && ./build.sh && cd ../..
mkdir build && cd build && cmake .. && cmake --build . -j$(nproc)
./auto_navigation                         # run from build/
```

See **[demos/README.md](demos/README.md)** for six step-by-step demos (webcam dry-run → offline RRT viz).

---

# 1. Project Overview

**Problem solved:** Autonomously explore an indoor space with a low-cost Tello, using SLAM-derived geometry instead of pre-built floor plans.

**Honest project state:** Exit-path detection is **not production-ready**. The drone can circle or get stuck when map noise creates false openings. **Low-level navigation** (following computed waypoints) works better than **exit/goal selection**. Several **magic numbers** (KMeans `k`, minimum cluster size, RRT radii) need per-environment tuning.

**Primary users:**

| Audience | How they use it |
|---|---|
| **Lab students / researchers** | Tune clustering/RRT params, extend `goal_finder` / `navigator` |
| **Algorithm developers** | Run `show_rrt` / `test_runner` offline on saved `.xyz`/`.pcd` scans |
| **Hardware operators** | Live Tello runs with `fake_drone: false` in `config.json` |

**What it is not:** a polished product, ROS node, or cloud service. It is a research codebase with hardcoded mission snippets in `auto_navigation.cc` and known SLAM-viewer quirks.

---

# 2. Architecture & Tech Stack

## Languages & build

| Layer | Technology |
|---|---|
| Language | C++17 |
| Build | CMake 3.5+, Release `-O3` |
| Submodule | `external/ORB_SLAM3` (fork: [rbdlabhaifa/ORB_SLAM3](https://github.com/rbdlabhaifa/ORB_SLAM3)) |
| Analysis | Python 3 (`plot.py` — Plotly) |
| Config | JSON (`config.json`) |
| Lint/format | `.clang-format`, `.clang-tidy` (local IDE use; no CI enforcement) |

## Core libraries

| Library | Role |
|---|---|
| **ORB-SLAM3** | Monocular SLAM, Pangolin viewer |
| **PCL** | Point clouds, PCD I/O, visualization |
| **Eigen 3.3** (< 3.4 required) | Linear algebra |
| **GEOS** | Convex hulls, polygon obstacles |
| **LEMON** | Directed graphs, DFS for path extraction |
| **OpenCV 4** | Image matrices, calibration YAML |
| **Boost** | Lock-free SPSC queue for camera frames |
| **ctello-fork** | DJI Tello UDP control |
| **nlohmann/json** | Runtime configuration |

## Component diagram

```
┌─────────────────────────────────────────────────────────────┐
│  auto_navigation (main)                                     │
│    Streamer → ORB_SLAM3 → Navigator → Explorer              │
│         │                        │                          │
│         ▼                        ├─ goal_finder (KMeans+GEOS)
│    Tello / Webcam                └─ path_builder (RRT+LEMON) │
│         ▲                                    │               │
│         └──────────── Drone (ctello) ◄───────┘               │
└─────────────────────────────────────────────────────────────┘
         │ writes scans/DD.MM.YY/HH:MM:SS/*.xyz
         ▼
   show_rrt / test_runner / plot.py  (offline analysis)
```

**Libraries (CMake targets):** `tello_stream`, `slam_utils`, `goal_finder`, `path_builder`, `pcl_operations`, `eigen_operations`.

**Executables:** `auto_navigation`, `map_builder`, `show_rrt`, `test_runner`.

---

# 3. Prerequisites

## Platform

| Requirement | Notes |
|---|---|
| **OS** | Ubuntu 20.04 / 22.04 (documented); other Linux may work |
| **CPU** | Multi-core recommended for ORB-SLAM3 + PCL |
| **GPU / display** | OpenGL for Pangolin (ORB viewer) and PCL visualizer |
| **Git** | `--recursive` clone for ORB_SLAM3 submodule |

## System packages (`install_dependencies.sh` covers part)

- PCL, LEMON, OpenCV 4, GEOS, Boost
- Eigen **3.3.9** (built from source by script — **must be < 3.4**)

## Manual installs (NOT in `install_dependencies.sh`)

| Dependency | Install |
|---|---|
| **Pangolin** | Build from [stevenlovegrove/Pangolin](https://github.com/stevenlovegrove/Pangolin); deps: `libgl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols libegl1-mesa-dev libglew-dev cmake g++ ninja-build` |
| **ctello-fork** | [mipdableep/ctello_fork](https://github.com/mipdableep/ctello_fork.git) — build & install to system path |
| **nlohmann-json** | `sudo apt install nlohmann-json3-dev` or header-only install |

## Pinned dependency versions (from original docs)

| Package | Version / commit |
|---|---|
| LEMON | 1.3.1 |
| GEOS | 3.13.0 @ `15185e65b3e7bc671cf693c97cf103125fb4a171` |
| Eigen | 3.3.9 |
| ctello-fork | see GitHub link above |

## External data

| Asset | Location |
|---|---|
| ORB vocabulary | `Configuration/Vocabulary/ORBvoc.txt.tar.gz` → extract to `ORBvoc.txt` |
| Tello camera calibration | `Configuration/CalibrationFiles/tello_9F5EC2_640.yaml` |
| Demo screenshots | `images/` (committed) |
| Scan artifacts | Generated at runtime under `scans/` (gitignored) |

---

# 4. Environment Setup

## Clone with submodule

```bash
git clone --recursive https://github.com/rbdlabhaifa/RBD-SLAM.git
cd RBD-SLAM
git submodule update --init --recursive   # if cloned without --recursive
```

## First-run configuration

No `.env` file. Settings live in **`config.json`** at the repo root.

```bash
cp config.json.example config.json
nano config.json
mkdir -p scans
```

| Key | Purpose |
|---|---|
| `Vocabulary_Path` | Absolute path to `ORBvoc.txt` |
| `calibration_path` | Camera YAML (Tello 640×480 calibration) |
| `map_path` | `"--use-drone"` for live SLAM map; or path to saved map |
| `data_save_dir` | Writable directory for timestamped scan output |
| `fake_drone` | `true` = no Tello motor commands (safe testing) |
| `use_webcam` | `true` = webcam instead of Tello video stream |
| `offline_mode` | Present in schema; **not wired** in current `auto_navigation` |

**Config resolution:** `auto_navigation` reads `../config.json` relative to **`build/`** (or pass a custom path as argv[1]).

Extract vocabulary once:

```bash
cd Configuration/Vocabulary
tar xvfz ORBvoc.txt.tar.gz
cd ../..
```

> **Note:** `config.json` is local config (`.gitignore`). The `.example` file is the committed template.

---

# 5. Build & Run Instructions

## Install dependencies

```bash
./install_dependencies.sh
# Then manually: Pangolin, ctello-fork, nlohmann-json (see §3)
```

## Build ORB-SLAM3 + project

```bash
cd external/ORB_SLAM3
./build.sh
cd ../..
mkdir -p build && cd build
cmake ..
cmake --build . -j$(nproc)
```

Details: [COMPILATION.md](COMPILATION.md)

## Run — main application

```bash
cd build
./auto_navigation
# Optional custom config path:
./auto_navigation /path/to/config.json
```

**Recommended first run** (no drone):

```json
{ "fake_drone": true, "use_webcam": true }
```

## Run — other executables

| Binary | Command |
|---|---|
| SLAM map recorder | `./map_builder ../Configuration/Vocabulary/ORBvoc.txt ../Configuration/CalibrationFiles/tello_9F5EC2_640.yaml` |
| Offline RRT viz | `./show_rrt map.pcd plane_points.pcd start.pcd 0.1` |
| Goal finder test | `./test_runner map.xyz plane.pcd start.pcd` |
| Plotly analysis | `python3 ../plot.py` (edit paths in script) |

## Convert `.xyz` → `.pcd`

```bash
pcl_xyz2pcd file.xyz file.pcd
```

## Demos

Full walkthrough with expected outputs and screenshot references:

**[demos/README.md](demos/README.md)**

### Visual demo results (committed screenshots)

Offline RRT visualization should resemble:

![RRT path on map](images/Screenshot%20from%202023-08-01%2014-12-30.png)

![RRT alternate view](images/Screenshot%20from%202023-08-01%2014-13-17.png)

Full RRT search tree (`tree.xyz`) in MeshLab:

![RRT tree in MeshLab](images/Screenshot%20from%202023-08-01%2014-14-21.png)

## Tests & linters

**No automated test suite or CI.** `.clang-format` / `.clang-tidy` exist for local use.

Manual validation:

```bash
cd build
./show_rrt <your_map.pcd> <plane.pcd> <start.pcd> 0.1
./test_runner map.xyz plane.pcd start.pcd
```

---

# 6. Repository Structure

```
RBD-SLAM/
├── CMakeLists.txt
├── config.json.example          # ★ Config template — copy to config.json
├── COMPILATION.md
├── install_dependencies.sh
├── plot.py                      # Plotly 3D offline visualization
├── demos/README.md              # ★ Step-by-step demo guide
├── images/                      # ★ Committed RRT demo screenshots
├── Configuration/
│   ├── CalibrationFiles/tello_9F5EC2_640.yaml
│   └── Vocabulary/ORBvoc.txt.tar.gz
├── external/ORB_SLAM3/          # ★ Git submodule
├── include/                     # Headers (navigator, explorer, path_builder, …)
└── src/
    ├── apps/auto_navigation.cc  # ★ Main entry point
    ├── navigator.cc             # Orchestration + drone commands
    ├── explorer.cc              # RRT + exit exploration facade
    ├── goal_finder.cc           # KMeans + exit heuristics
    ├── path_builder.cc          # RRT + LEMON graph
    ├── drone.cc / streamer.cc   # Tello + frame queue
    └── utils/
        ├── map_builder.cc
        ├── show_rrt.cc          # Offline PCL visualizer
        └── test_runner.cc
```

---

# 7. Core Workflows & Data Flow

## Workflow A — Live autonomous navigation

```
config.json
     │
     ▼
auto_navigation.cc
     ├─ Streamer.start_stream()     ← Tello or webcam frames
     ├─ Navigator.start_navigation()
     │    ├─ ORB_SLAM3 TrackMonocular
     │    ├─ slam_utils alignment
     │    ├─ Explorer.get_path_to_the_unknown()
     │    │    ├─ goal_finder::Find_Goal()
     │    │    └─ path_builder RRT
     │    └─ goto_point() → Drone commands
     └─ saves scans/<timestamp>/*.xyz
```

**Read first:** `src/apps/auto_navigation.cc` → `src/navigator.cc` → `src/explorer.cc`.

## Workflow B — Offline RRT visualization

```
Live run produces map_for_path1.xyz, plane_points.xyz, start.xyz
     │
     ▼
pcl_xyz2pcd → map.pcd, plane_points.pcd, start.pcd
     │
     ▼
show_rrt map.pcd plane_points.pcd start.pcd 0.1
     └─ PCL visualizer (see images/)
```

## Scan output artifacts

| File | Description |
|---|---|
| `aligned_points.xyz` | SLAM map points in aligned frame |
| `plane_points.xyz` | 3 points defining flight plane |
| `map_for_pathN.xyz` | Obstacle cloud for planning iteration N |
| `start.xyz` / `end.xyz` | RRT endpoints |
| `pathN.xyz` | Selected navigation path |
| `tree.xyz` | Full RRT tree |
| `exitN.xyz` | Goal finder output |

---

# 8. Deployment & CI/CD

## Deployment

**Lab workstation + Tello only.** No Docker, Kubernetes, or cloud deployment in this repo (a `docker_fixes` branch exists historically but no root Dockerfile on `master`).

Typical setup: Ubuntu PC on same Wi-Fi as Tello, built binaries in `build/`, config pointing at local paths.

## CI/CD

**None configured.** No GitHub Actions or automated build pipeline.

---

# 9. Known Quirks & Technical Debt

| Issue | Context | Impact |
|---|---|---|
| **Exit finding unreliable** | Map noise, magic KMeans params | Drone circles / stuck (documented in original README) |
| **`offline_mode` unused** | In JSON but not read in main loop | Misleading config key |
| **Mission hardcoding in `auto_navigation`** | `forward 110`, RC spins after scan 1 | Lab-specific; must edit for new environments |
| **ORB viewer Stop broken** | ORB-SLAM3 Pangolin UI | Use **Ctrl-C** to exit |
| **No scan data in repo** | `scans/` gitignored | Must generate your own for offline demos |
| **ctello / Pangolin not in install script** | Manual install required | Build fails if missing |
| **Eigen version sensitivity** | Requires < 3.4 | System Eigen 3.4 breaks build |
| **`offlineRun.cc` not in CMake** | Dead stub | Ignore |
| **Large debug logs** | Historical commits included huge `logger.txt`-style artifacts | Bloated history on some branches |

---

# 10. Troubleshooting / FAQ

### 1. `libORB_SLAM3.so` not found / ORB build failure

**Fix:**

```bash
cd external/ORB_SLAM3
git submodule update --init --recursive
./build.sh
ls lib/libORB_SLAM3.so    # must exist before cmake ..
```

---

### 2. `Failed to open config.json`

**Fix:**

```bash
cp config.json.example config.json
# Edit absolute paths for Vocabulary_Path, calibration_path, data_save_dir
cd build && ./auto_navigation    # expects ../config.json
```

---

### 3. `show_rrt` cannot read PCD files

**Fix:**

```bash
# Generate from a prior scan session
pcl_xyz2pcd map_for_path1.xyz map.pcd
pcl_xyz2pcd plane_points.xyz plane_points.pcd
pcl_xyz2pcd start.xyz start.pcd
./show_rrt map.pcd plane_points.pcd start.pcd 0.1
```

See [demos/README.md](demos/README.md) Demo 3.

---

## Quick reference

| Resource | URL |
|---|---|
| Repository | https://github.com/rbdlabhaifa/RBD-SLAM |
| ORB-SLAM3 submodule | https://github.com/rbdlabhaifa/ORB_SLAM3 |
| ctello-fork | https://github.com/mipdableep/ctello_fork |
| Demos guide | [demos/README.md](demos/README.md) |

**Stop button:** ORB-SLAM viewer Stop is unreliable — use **Ctrl-C**.

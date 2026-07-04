# RBD-SLAM Demos

Step-by-step demos for validating the stack without prior lab knowledge. Screenshots of expected output live in [`../images/`](../images/).

| Screenshot | What it shows |
|---|---|
| [`Screenshot from 2023-08-01 14-12-30.png`](../images/Screenshot%20from%202023-08-01%2014-12-30.png) | PCL OpenGL viewer — RRT path on SLAM point cloud |
| [`Screenshot from 2023-08-01 14-13-17.png`](../images/Screenshot%20from%202023-08-01%2014-13-17.png) | Alternate RRT visualization angle |
| [`Screenshot from 2023-08-01 14-14-21.png`](../images/Screenshot%20from%202023-08-01%2014-14-21.png) | Full RRT tree (`tree.xyz`) viewed in MeshLab |

---

## Prerequisites (all demos)

```bash
# From repo root
cp config.json.example config.json    # edit paths
cd Configuration/Vocabulary && tar xvfz ORBvoc.txt.tar.gz && cd ../..
mkdir -p scans

# Build (see COMPILATION.md)
cd external/ORB_SLAM3 && ./build.sh && cd ../..
mkdir -p build && cd build && cmake .. && cmake --build . -j$(nproc)
```

Run all binaries from **`build/`** so `../config.json` resolves correctly.

---

## Demo 1 — Webcam + fake drone (safest first run)

**Goal:** Exercise SLAM + navigation loop without flying a Tello.

Edit `config.json`:

```json
{
  "fake_drone": true,
  "use_webcam": true,
  "map_path": "--use-drone"
}
```

```bash
cd build
./auto_navigation
```

**Expected:** ORB-SLAM3 viewer opens, frames come from webcam index 0, scan artifacts appear under `scans/DD.MM.YY/HH:MM:SS/` (`.xyz` point dumps). No motor commands sent.

**Stop:** Ctrl-C (ORB viewer Stop button may not work).

---

## Demo 2 — Record a SLAM map (`map_builder`)

**Goal:** Capture a map session for offline planning demos.

```bash
cd build
./map_builder ../Configuration/Vocabulary/ORBvoc.txt \
               ../Configuration/CalibrationFiles/tello_9F5EC2_640.yaml
```

On SIGINT (Ctrl-C), saves `point_data.xyz`, `aligned_point_data.xyz`, and related files into a timestamped directory.

---

## Demo 3 — Offline RRT visualization (`show_rrt`)

**Goal:** Reproduce the screenshots in `images/` without flying.

### 3a. Produce inputs from a live run

After Demo 1 or a real Tello session, locate your scan folder:

```text
scans/13.09.23/14:19:16/
├── map_for_path1.xyz
├── plane_points.xyz
├── start.xyz
├── tree.xyz
└── ...
```

Convert to PCD:

```bash
cd scans/<DD.MM.YY>/<HH:MM:SS>
pcl_xyz2pcd map_for_path1.xyz map.pcd
pcl_xyz2pcd plane_points.xyz plane_points.pcd
pcl_xyz2pcd start.xyz start.pcd
```

### 3b. Run the visualizer

```bash
cd build
./show_rrt ../scans/<DD.MM.YY>/<HH:MM:SS>/map.pcd \
            ../scans/<DD.MM.YY>/<HH:MM:SS>/plane_points.pcd \
            ../scans/<DD.MM.YY>/<HH:MM:SS>/start.pcd \
            0.1
```

**Expected:** PCL 3D viewer with point cloud, RRT path spheres/lines (similar to `images/Screenshot*.png`).

**Optional:** Open `tree.xyz` + map in [MeshLab](https://www.meshlab.net/) to inspect the full search tree.

---

## Demo 4 — Exit-point test (`test_runner`)

**Goal:** Test goal-finding logic on saved clouds without the full navigator.

```bash
cd build
./test_runner map.xyz plane.pcd start.pcd
```

Arguments: `(1) map.xyz`, `(2) plane PCD (3 points)`, `(3) start PCD (1 point)`. Writes `../exit.xyz`.

---

## Demo 5 — Plotly 3D analysis (`plot.py`)

**Goal:** Interactive browser visualization of saved paths.

```bash
pip install plotly scipy numpy
python3 plot.py   # edit paths at bottom of script first
```

---

## Demo 6 — Live Tello navigation (lab only)

**Goal:** Full autonomous exploration (see project state caveats in README).

```json
{
  "fake_drone": false,
  "use_webcam": false,
  "map_path": "--use-drone"
}
```

```bash
cd build
./auto_navigation
```

**Known issues:** Exit-path detection can fail; drone may circle in-room. Navigation layer is more reliable than exit finding.

---

## Artifact reference

| File | Produced by | Used in |
|---|---|---|
| `aligned_points.xyz` | Navigator / SLAM alignment | Maps, offline viz |
| `plane_points.xyz` | Flight-plane fit | `show_rrt`, `test_runner` |
| `map_for_pathN.xyz` | Path planning iteration | `show_rrt` |
| `start.xyz` / `end.xyz` | RRT start/end | `show_rrt` |
| `tree.xyz` | RRT search tree | MeshLab, `plot.py` |
| `pathN.xyz` | Best path chosen | Analysis |
| `exitN.xyz` | Goal finder output | Validation |

# RBD-SLAM

A system that navigates the drone automatically and finds points of interest to navigate

## Dependencies

1. [PCL](https://pointclouds.org/) - Point Cloud Library
2. [Eigen](https://eigen.tuxfamily.org/) < 3.4.0 - Linear Algebra
3. [LEMON](https://lemon.cs.elte.hu/trac/lemon) - Graph Library for DFS
4. OpenCV
5. [GEOS](https://libgeos.org/) - Geometry Library for Convexhulls etc.
6. [Boost](https://www.boost.org/) - Used for thread-safe camera streaming

## Applications

The system contains multiple applications

1. `auto_navigation` - The main app, finds points of interest and paths to them, navigates the drone
2. `show_rrt` - Shows our path finding algorithm for a given map
3. `map_builder` - Builds a map manually, without flying the drone (Not in active use)

## Compilation

Compilation instructions are located in [COMPILATION.md](COMPILATION.md)

## Running the applications

For running `auto_navigation`, with the options `--use-webcam` for using the webcam instead of the drone's camera, and `--fake-drone` for not sending commands to the drone, execute:

```bash
./auto_navigation ../Configuration/Vocabulary/ORBvoc.txt ../Configuration/CalibrationFiles/tello_9F5EC2_640.yaml [--use-webcam] [--fake-drone]
```

For running `show_rrt`, in a folder with files `map.pcd` containing the map, `plane_points.pcd` containing 3 points for the plane, `start.pcd` containing a single point to start the RRT from, and `radius` being radius of points to show (a small floating point number) execute:

```bash
./show_rrt map.pcd plane_points.pcd start.pcd radius
```

## Converting .xyz files to .pcd

```bash
pcl_xyz2pcd file.xyz file.pcd
```

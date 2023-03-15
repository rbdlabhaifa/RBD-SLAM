# RBD-SLAM

A system that navigates the drone automatically and finds points of interest to navigate

## Project State

At the moment, the algorithms we implemented can control the drone and exit a room (~60-70% of the time).

There are some "magic numbers" in the code:

1. K for KMeans - we use KMeans to close off obstacles as polygons, to decrease sparsity.
2. Minimum cluster size - the minimal cluster size for KMeans. We drop the clusters smaller than this size.

The general approach for finding a "good" path in the RRT is a path that distances itself from the polygons in the map.

## Future Work

Future work for the project will be adjusting these "magic numbers" according to the data in the map supplied. Also, because we use RRT, we need to check that "cutting off" "good" paths in the tree works as expected, and this will prevent hitting a wall etc. Adjusting the said parameters will also close off the room better and won't leave holes in the map which the RRT can exit from.

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

NOTE: Stop button in ORB_SLAM is not always functional, and Ctrl-C is required to stop the program

## Converting .xyz files to .pcd

```bash
pcl_xyz2pcd file.xyz file.pcd
```

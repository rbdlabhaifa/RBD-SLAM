# RBD-SLAM

A system that navigates the drone automatically and finds points of interest to navigate

## Project State

At the moment, the algorithms we implemented have problems and don't find a correct exit path, the drone keeps getting stuck and flies around in circles in the room, most likely due to noise around the map, navigation works good.

There are some "magic numbers" in the code:

1. K for KMeans - we use KMeans to close off obstacles as polygons, to decrease sparsity.
2. Minimum cluster size - the minimal cluster size for KMeans. We drop the clusters smaller than this size.

The general approach for finding a "good" path in the RRT is a path that distances itself from the polygons in the map.

## Future Work

Future work for the project will be adjusting these "magic numbers" according to the data in the map supplied. Also, because we use RRT, we need to check that "cutting off" "good" paths in the tree works as expected, and this will prevent hitting a wall etc. Adjusting the said parameters will also close off the room better and won't leave holes in the map which the RRT can exit from.
also we need to reduce the noise around the map so the algorithm can choose the correct path, as for continuation after finding the first exit door, we need to close the gap in the polygons after the exit in order to do a second scan in the outside room.

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

## Compilation

Compilation instructions are located in [COMPILATION.md](COMPILATION.md)

## Converting .xyz files to .pcd (used for offline run only in Show RRT)

```bash
pcl_xyz2pcd file.xyz file.pcd
```


## Running the applications

First, extract the `ORBVoc.txt.tar.gz` file for the ORB vocabulary:

```bash
cd Configuration/Vocabulary
tar xvfz ORBvoc.txt.tar.gz
```

For running `auto_navigation`, with the options `--use-webcam` for using the webcam instead of the drone's camera, and `--fake-drone` for not sending commands to the drone, execute:
for regular usage just use the [--use-webcam] argument.

```bash
./auto_navigation ../Configuration/Vocabulary/ORBvoc.txt ../Configuration/CalibrationFiles/tello_9F5EC2_640.yaml [--use-webcam] [--fake-drone]
```

For doing an offline run to see the algorithmic side of things, use `show_rrt`, in a folder with files `map.pcd` containing the map, `plane_points.pcd` containing 3 points for the plane, `start.pcd` containing a single point to start the RRT from, and `radius` being radius of points to show (a small floating point number)
first we need to do an online run using the drone in order to get a map of the lab, plane points and starting point, once we have that then we can do an offline run using :
```bash
./show_rrt map.pcd plane_points.pcd start.pcd radius
```

to do that, execute:

```bash
cd ~/RBD-SLAM/build/
cd *to a currently run directory using the drone*
pcl_xyz2pcd map_for_path1.xyz map.pcd
pcl_xyz2pcd start.xyz start.pcd
pcl_xyz2pcd plane_points.xyz plane_points.pcd
../show_rrt map.pcd plane_points.pcd start.pcd 0.1
```
an opengl plot should open and show the RRT results on the map, you can always choose a smaller radius for the points.
The result should be something like this:-

tree.xyz in the directory holds the whole RRT tree and can be viewed with the map in meshlab likewise.


NOTE: Stop button in ORB_SLAM is not always functional, and Ctrl-C is required to stop the program


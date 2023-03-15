# Compilation

To compile the project, first install all of the dependencies specified in [README.md](README.md), then execute the following:

```bash
git clone --recursive https://github.com/rbdlabhaifa/RBD-SLAM

cd RBD-SLAM
cd external/ORB_SLAM3
./build.sh

cd ../..
mkdir build
cd build
cmake ..
cmake --build . -j(number_of_cores)
```

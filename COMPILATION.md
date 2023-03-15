# Compilation

## Dependencies Installation

To install the dependencies in [README.md](README.md), either install them manually or run the script `./install_dependencies.sh` for Ubuntu

To compile the project, execute the following:

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

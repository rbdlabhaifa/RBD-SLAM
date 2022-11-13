# Drone navigation and charging 24/7

## Prerequisites
to run the project needs opencv 3.4.16 compiled to incloode the contrib repo, and some dependecies.
to set up all of the above you can use the build_opencv.sh and the pd_dep_build.sh files, as seen below.
to clone and compile opencv 3.4.16 place the opencv_build.sh file in the folder you want opencv to be compiled at, run "chmod +x build_opencv.sh" and then run "./build_opencv.sh"
to install all of the PatrolingDrone project dependencies run "chmod +x pd_dep_build.sh" and then run ./pd_dep_build.sh

### Build map.
### Choose checkpoints.
#### Drone will navigate from chkpnt_1-chkpnt_2-...-chkpnt_last-...-chkpnt_2-chkpnt_1-chkpnt_2-... in a loop (and charge when needed).


## LAST UPDATE NOTES:
the pd_dep_build.sh file now installes some dependencies needed by the project.
the build_opencv.sh is a file that clones and compiles opencv with its dependencies.

If running on different computer: there is some hardcoded paths in python and cpp (/home/rbdlab2/...) make sure to find them and change that.

## How to use:

### tello_mapping:  
Connect to tello with wifi and build the map by moving the tello manually.  
Try to do loop closure especially near the checkpoints.   
After building the map (Don't close ORBSLAM yet):  
Turn on localization mode, save the checkpoints by putting the tello at each checkpoint and clicking the "Save Destination" button.  
**For the drone to be able to go the charger when moving from the second checkpoint to the first checkpoint the drone should look at the charger.
First checkpoint needs to be very close to charger (Tello needs to be close in order to see aruco).**  
When finished click ShutDown.   
Change the created Slam_latest_Map.bin and drone_destinations.txt to Slam_latest_Map_morning.bin and drone_destinations_morning.txt
or Slam_latest_Map_evening.bin and drone_destinations_evening.txt according to the time of day.

### tello_main:  
Only insert the battery to the raspberry pi and then run tello_main.  
It should wait for the raspberry to power on, connect to it and turn on the drone.  
Everything should happen automatically now.  

### RaspberryPi:
The raspberry pi on the drone is responsible for turning the drone on and off when charging and at the start by communicating with bluetooth.  
You need to compile Drone/BluetoothServer on the raspberry, and add the executable to ~/.bashrc so it will run when booting.

### camera_calib:
Can be used to get a few pictures from the drone.  
Connect to the tello and run the program.

## Parameters to change:  

### tello_navigation.cc:  
tello_name - Name of the tello WIFI  
rpi_bluetooth_address - bluetooth address of the raspberry pi  

### aliases.txt:  
tello_mapping line - TUM file to the drone camera calibration  

### tello_main.cc:
tello_navigation line - TUM file to the drone camera calibration  


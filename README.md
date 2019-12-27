
Camera Calibration on Duckietown simulator
==================
![alt text](https://img.shields.io/badge/python-2.7-green.svg "Python2.7")

This repository provides general instructions to perform camera calibration on duckietown simulator.

# Installation

Steps:
-------------
1) Clone the LF duckietown baseline

```
git clone https://github.com/duckietown/challenge-aido_LF-baseline-duckietown
```

2) Change the simulation submodule to [ai404/gym-duckietown](https://github.com/ai404/gym-duckietown) on daffy
3) Change the dt-core submodule to [ai404/dt-core](https://github.com/ai404/dt-core) on branch daffy
4) Update submodules:
```
git submodule init
git submodule update --recursive
```
5) Switch to daffy branch on all submodules
```
git submodule foreach "(git checkout daffy; git pull)"
```
6) Pick the `calibration_map` in `1_develop/utils/ros_helpers.py`

7) Build the docker images:
```
cd 1_develop
docker-compose build
docker-compose up
```

8) On the simulator container open a terminal and install missing packages:

```
pip install quickapp
```
```
pip uninstall systemCmd
git clone --single-branch --branch p3 https://github.com/AndreaCensi/system_cmd
cd system_cmd
python setup.py install
```
```
pip install -U git+https://github.com/AndreaCensi/compmake/
```
9) Build and Source `catkin_ws`
```
catkin build --workspace catkin_ws
source catkin_ws/devel/setup.bash
```

# Extrinsic Calibration

1) The default name of the robot will interfere with default homography, we rename the default homography file to encounter the issue:
```
cp /data/config/calibrations/camera_extrinsic/default.yaml /data/config/calibrations/camera_extrinsic/default_homography.yaml
```
2) Also we need to specify the project root folder in which the calibration files will be saved:
```
export DUCKIETOWN_ROOT=/duckietown
rosrun complete_image_pipeline calibrate_extrinsics
```
expected results:

![alt text](https://github.com/ai404/sim_calibration_duckietown/raw/master/images/homography.jpg "calibrate_extrinsics")

3) Change the map to `loop_empty` and restart the simulator
```
export DUCKIETOWN_ROOT=/duckietown
rosrun complete_image_pipeline single_image_pipeline
```
expected results:

![alt text](https://github.com/ai404/sim_calibration_duckietown/raw/master/images/projections.jpg "single_image_pipeline")

# Intrinsic Calibration

1) Disable access control or add localhost 
```
xhost + 
```
2) Connect to docker cont
docker exec -it -u root -e DISPLAY=host.docker.internal:0 931cbcb4bc42 /bin/bash

3) run the camera service
roslaunch pi_camera camera.launch veh:=default

4) perform intrinsic calibration
roslaunch pi_camera intrinsic_calibration.launch veh:=default

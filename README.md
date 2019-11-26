
Camera Calibration on Duckietown simulator
==================
![alt text](https://img.shields.io/badge/python-2.7-green.svg "Python2.7")

This repository provides general instructions to perform camera calibration on duckietown simulator.

# Extrinsic Calibration

Steps:
-------------
1) Clone the LF duckietown baseline

```
git clone https://github.com/duckietown/challenge-aido_LF-baseline-duckietown
```

2) Change the simulation submodule to ai404/gym-duckietown on daffy
3) Change the dt-core submodule to ai404/dt-core on branch daffy
4) Update submodules:
```
git submodule init
git submodule update
```
5) Switch to daffy branch on all submodules

6) Pick the `calibration_map` in res_helpers.py

7) Install missing packages
```
pip install quickapp
```
```
pip uninstall systemCmd
git clone --single-branch --branch p3 https://github.com/AndreaCensi/system_cmd
Cd system_cmd
Python setup.py install
```
```
pip install -U git+https://github.com/AndreaCensi/compmake/
```
8) Build and Source `catkin_ws`
```
catkin build --workspace catkin_ws
source catkin_ws/devel/setup.bash
```
9) The default name of the robot will interfere with default homography, we rename the default tomography file to encounter the issue:
```
cp /data/config/calibrations/camera_extrinsic/default.yaml /data/config/calibrations/camera_extrinsic/default_homography.yaml
```
10) Also we need to specify the project root folder in which the calibration files will be saved:
```
export DUCKIETOWN_ROOT=/duckietown
rosrun complete_image_pipeline calibrate_extrinsics
```

11) Change the map to `loop_empty` and restart the simulator
```
export DUCKIETOWN_ROOT=/duckietown
rosrun complete_image_pipeline single_image_pipeline
```

# TODO

- [x] Extrinsic Calibration
- [ ] Intrinsic Calibration

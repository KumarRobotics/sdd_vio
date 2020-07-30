# Semi-Dense Direct Visual Inertial Odometry

[Low Texture Dataset](https://drive.google.com/drive/folders/1U8gvH7T0CkGYPr03miqyINjqM0vNap5n?usp=sharing) (5 Rosbags, 842MB) \
[Video - EuRoC Dataset](https://drive.google.com/file/d/1wPehEBmDj-Cq0U8H5JSmuaKKSnddE69D/view?usp=sharing) \
[Video - Low Texture Dataset](https://drive.google.com/file/d/1c6ZSkqv6xGx2S8TW6gCO22NPK98iA_ho/view?usp=sharing) \
[Video - Flight](https://drive.google.com/file/d/1nz2vDpdt20K8VT4DuHf5OBYkbVfet-z6/view?usp=sharing)

### Instructions

In ROS environment, compile the catkin package in RELEASE mode. To run the nodelet using low texture dataset
```
roslaunch sdd_vio vio_nodelet.launch
roslaunch sdd_vio visualization.launch
```
then play the bag files.
To run with EuRoC Dataset
```
roslaunch sdd_vio vio_nodelet_euroc.launch
```
To use your own configuration change the config yaml file and use calibration procedures below.

### Calibration file
This package takes in a stereo calibration yaml file as input, and [Kalibr](https://github.com/ethz-asl/kalibr) format is accepted.
An example calibration file looks like follows:
```
cam0:
  T_cam_imu:
  - [-0.010944628415755053, 0.999681932600619, 0.022721107823566122, 0.02204409545771752]
  - [-0.24433953077738618, -0.024707336115557976, 0.9693749229485756, -0.004840907304875826]
  - [0.9696279744357241, 0.005057783502870783, 0.2445322269511941, -0.014140552900908787]
  - [0.0, 0.0, 0.0, 1.0]
  camera_model: pinhole
  distortion_coeffs: [-0.00966481, -0.00611693, -0.00043361, 0.00247824]
  distortion_model: radtan
  intrinsics: [438.327, 438.04, 326.966, 244.717]
  resolution: [640, 480]
  rostopic: /cam0/image_raw
cam1:
  T_cam_imu:
  - [-0.0034837082769738625, 0.999800943534273, 0.019645281484841126, -0.05783642507968116]
  - [-0.23039691655783784, -0.01991935443843984, 0.9728928410464314, -0.004976419765158304]
  - [0.973090501760901, -0.0011369374360561402, 0.23042044778184256, -0.01442186798130469]
  - [0.0, 0.0, 0.0, 1.0]
  T_cn_cnm1:
  - [0.9999674298999375, -0.00480756708912238, 0.0064827801488079, -0.07981140545]
  - [0.004713793040877623, 0.9998851514964509, 0.014403609705481836, -3.630473e-05]
  - [-0.006551281931192101, -0.014372582094518168, 0.9998752470130735, -0.00020823841]
  - [0.0, 0.0, 0.0, 1.0]
  camera_model: pinhole
  distortion_coeffs: [-0.015658, -0.00162092, -0.000711559, 0.000227274]
  distortion_model: radtan
  intrinsics: [432.493, 432.041, 319.181, 230.679]
  resolution: [640, 480]
  rostopic: /cam1/image_raw
```
`cam0` is the left camera and `cam1` is the right camera. `T_cn_cnm1` is the relative transformation from the right to the left camera.

### Param file
The parameters for VIO are specified in `vo_param.yaml`. 

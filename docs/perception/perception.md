# Introduction

The perception team will work on the eyes of our car. This team will make sure, we can detect the cones. These detections are then passed through to the mapping subteam. The detection of the cones can be done using RGB cameras, stereo RGB camera or LiDAR's.

> Cone Detection > Keypoint Regression > Perspective n Point (PnP)

# Perception node

The perception node does three things:
* take in data from a specific source (right now only camera sources)
* detect cones
* detect keypoints

You can also specify whether you want to enable CUDA support.

## Image input

_Right now only camera sources_

The image input can come from three sources, based on the ROS parameter set in the launch files.

### Camera 
(Set the parameter to `camera` to use the Baumer camera, provided it is in the same subnet as the pc.) By default this will use the `UserSet1` which uses an automatic exposure time. You can add your own UserSets in the Baumer Explorer software by setting the parameters as preferred and then writing the configuration to the camera. There is a special menu for this.
*We will start switching to in-code configuration, because this is a lot easier*

The required Neoapi for the camera is included in the `requirements.txt` file. All useful Baumer APIs can be found [here](https://ugentracing.sharepoint.com/:f:/s/UGR9/EjE9w_0oC3BLhY7aYIjS-fwBTLD4tUGScFrbzY3JpvrQoA?e=bQkaN6), including the package to install the Python API on the Jetson (the ARM tar.gz). The documentation folder can also be found seperately on the SharePoint and has extensive documentation on programming, connecting, setting up a camera.

### Simulator
_Right now only camera sources_

Set the parameter to `sensor`. To accept input from the simulator, the sim itself will publish its camera data according to the defined cameras inside the `settings.json`. By default we use `cam1`, which means the camera data will be published to `/fsds/cam1`. The launchfile converts this topic to our internal `/sensor/rgb/mono`.

### Video file
_Right now only camera sources_

Set the parameter to a video file name (including the extension) to replay that file as camera input. The file itself should be located inside `autonomous_binaries/dummy_data/visual_pipeline`. Example parameter value: `track_walk.mp4`.

## Cone detection
The cone detection is a standard YOLOv5 network, optimised using a TensorRT engine model.

## Keypoint detection
To save on computations, only those cones with a height taller than 30px in the image (to ensure good keypoints) and those that are taller than they are wide (to eliminate fallen cones) will be passed through to keypoint detection. The keypoint detector is a Rektnet implemented in PyTorch Lightning.


## Training the networks

**We should look at a way to optimise this pipeline**

## Supervisely
We currently use Supervisely to label images. First cones should be labelled for detection, after which you can use the ´convert_yolo.py´ to convert the output of Supervisely into images and labels for YOLOv5 and cropped images for keypoint detection. (The code does the yolo part, but the keypoint part should be changed slightly because we should no longer be labeling cone and keypoint at the same time.)

These cropped images should then be uploaded to another Supervisely session for keypoint labeling. We should use a keypoint labeler as is built-in in Supervisely instead of seperate Point objects because this was a very dumb idea of me in the first place.

the order of the keypoints is [top, band_top_left, band_top_right, band_bottom_left, band_bottom_right, bottom_left, bottom right]
![image](https://user-images.githubusercontent.com/78904872/163577166-2d52a36e-ac25-46e9-b46c-4db6c7097d13.png)


## YOLOv5
We currently use YOLOv5 for cone detections. The data used for training is available on the SharePoint (the zip iles with YOLOv5 in their name).

Inside the handy_scripts folder you can find the ´swap_MIT_cone_colours.py´ file which swaps the class ids because MIT uses an incorrect order. This is already fixed for the ´MIT_YOLOV5.zip´ on the SharePoint.

The ´MIT_generate_split.py´ is made to split MIT data into train-val-test sets. Because every 'run' has a different amount of images and we don't want a run to be in multiple splits, we have to create three sets of runs that are approximately the right amounts of data (70-15-15). The ´split_helper´ tries out random splits and spits out the run ids for each split. This is a brute-force method.

The ´image_mover´ actually moves the data to the right folders.

## Rektnet
We use Rektnet for keypoint detection. The data used is also available on SharePoint.

´convert_keypoints.py´ isn't that useful because it assumes that the points are labeled as Point objects, while we should start using real keypoint labeling inside Supervisely. 

# Perspective-n-point (the cone_pnp package)
This document describes how the PnP works (and how to determine the required parameters, etc..). See the code documentation for more details.

## How it works

PnP takes a set of cone detections and their cone keypoints. Then it tries to map those keypoint to an actual 3D-model of the same cone. This way it can calculate the position of the cone relative to the camera frame.

The transformation might seem straightforward, but the difficulty lies in the fact that camera sensors and especially camera lenses are not perfect. This is why this package also needs a camera calibration file, which contains a "camera matrix" and a "distortion matrix", which describe how the lens and the sensor distort the image.

This node also propagates the timestamp that was inserted at the moment the image was taken. It signifies the end of perception pipeline, because the result of pnp are locations relative to a specific frame. It is up to mapping to select the correct frame (based on the tf2 package and the timestamp) and to feed this to SLAM.

## Parameters required by the node

Here is a table of supported parameters and defaults:

| Parameter name | Default | Description |
|---|---|---|
| `/cone_pnp/max_distance` | `20.0` | Any distance from camera to cone that is bigger than this value is filtered out from the detectons. This is to prevent bad detections coming from bad cone keypoints (because of the high distance) |
| `/cone_pnp/scale` | `1.0` | PnP is relative to the cone models provided. The end result must be meters, so when you provide a cone model in mm, you must set this parameter to `0.001` |
| `/cone_pnp/cones_location`  | `cones.npz` | The filename of the file that contains the cone models. |
| `/cone_pnp/camcal_location` | `camera_calibration.npz` | The filename of the camera calibration file. |


## Calculating the camera matrix
In order to calculate the camera matrix you must calibrate your camera. You can do this by making use of the tool at /tools/camera-calibration. Check below for more information. The result must be put in `autonomous_binaries/pnp/camera_calibration.npz`. Note that you can name the calibration file as you want, but then you must supply the name in the launch file as a `rosparam` or in the `config.yaml` file of the config package. The default is `camera_calibration.npz`.

## Providing cone models
You must also provide a model of all the cones. By default you must place it in `autonomous_binaries/pnp/cones.npz`.

# Camera calibration
## How to use

 1. Print a [chessboard pattern](https://github.com/opencv/opencv/blob/master/doc/pattern.png) and lay it on a flat surface
 2. Take about 15 pictures of that chessboard while changing the distance and the angle every time. Make sure the chessboard is clearly visible!
 3. Run `python3 camera_calibration {img} {size} {length} {out}` where:
	 1. **{img}** is a mask for all the .jpg images
	 2. **{size}** is the grid size of the chessboard (wxh)
	 3. **{length}** is the physical length of a side of a square from the chessboard
	 4. **{out}** is the .npz output file  

## Demo
There are demo pictures of the Baumer camera in /data/baumer_1 folder. To run the demo, run `python3 camera_calibration "data/baumer_1/*.jpg" 6x9 21.5 output.npz`

If you see something like this in the end (it may be on a different picture, but the point is that the axes should match the square like this) then it is ok.
![image](https://user-images.githubusercontent.com/22452540/127037049-d02955c4-097e-4e00-a0ec-4f4d57d4952e.png)

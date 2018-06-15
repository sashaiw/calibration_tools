# calibration_tools
A utility for aligning point clouds from two separate RGBD cameras for the UMass Lowell HRI lab "Frankenscooter"

## Calibration Wizard
The calibration wizard determines the transform between two cameras based off of the extrinsics provided by OpenCV's `stereoCalibrate`, using a chessboard in view of both cameras. This transform can be used to align two pointclouds generated by RGBD cameras.

By default, `calibration_wizard.cpp` subscribes to the following topics:
 * `/rgbd_cam_2/rgb/image_raw`
 * `/rgbd_cam_2/rgb/camera_info`
 * `/rgbd_cam_1/rgb/image_raw`
 * `/rgbd_cam_1/rgb/camera_info`
 
 It also assumes a 6x7 chessboard with 6cm squares.

## //TODO
 * Add arguments, remove hard-coded topics/chessboard features, number of images. Create sane defaults.
 * Publish transform?
 * Add icp routine to calibration_wizard
 * Add other popular RGBD cameras to list_cams.sh
 
## Usage
Install ROS package as normal.

``rosrun calibration_tools calibration_wizard``

You will now be prompted to select several images in which the chessboard is visible to both cameras.
With one of the two OpenCV windows highlighted, press any key on the keyboard to take a snapshot.
A snapshot will only be taken if the chessboard is visible to both cameras.
Calibration will be improved by varying the position of the chessboard in x, y, size, and skew.
Once this is done (by default it will take 4 snapshots), it should return a transform in the format `x y z qx qy qz qw`.
This transform can be used by tf, eg. 

`tf static_transform_publisher x y z qx qy qz qw frame_id child_frame_id period_in_ms`

It can also be used in a launch file as such:

```xml
<launch>
<node pkg="tf" type="static_transform_publisher" name="link1_broadcaster" args="1 0 0 0 0 0 1 link1_parent link1 100" />
</launch>
``` 

`launch/two_cam.launch` is a launchfile that can be edited to serve as a static publisher for the Frankenscooter. 
Edit the following line to use the transform generated by the wizard:

`<node pkg="tf" type="static_transform_publisher" name="r_cam_center_bcast" args="-0.0342407 0.798076 -0.108612 0.192606 0.622208 -0.2296    91 0.723189 l_cam_center r_cam_center 100"/>`

## Finding USB cameras
The camera driver provided by OpenNI and used by two_cam.launch requires the device ID of each RGBD cam to be passed as arguments.
`scripts/list_cams.sh` can be used to list connected PrimeSense cameras by looking for the vendor ID `1d27`. 
In the future this can be extended to other popular cameras. Run this script with:

`rosrun calibration_tools list_cams.sh`

It will output the device IDs of each connected camera, one per line.
These device IDs can now be placed into the launch file `launch/two_cam.launch`.

## Calibrating intrinsics
`calibration_wizard` assumes that the intrinsics for both cameras are already calibrated and published in the `camera_info` message.
Unfortunately, this may not be the case with recent OpenNI drivers and certain cameras, as there is a bug that does not allow the serial number to be read from the cameras.
If you wish to use a particular calibration file for a particular camera, you can edit its entry in `launch/two_cam.launch` to include the following args:

```xml
<arg name="rgb_camera_info_url"
     value="file:///path/to/rgb_calibration.yaml" />
<arg name="depth_camera_info_url"
     value="file:///path/to/depth_calibration.yaml" />
```

Instructions for calibrating intrinsics of RGBD cameras can be found here: http://wiki.ros.org/openni_launch/Tutorials/IntrinsicCalibration
(incomplete) attempt at 3D position estimation using a pair of cameras.
============================================================================


Requires ROS (tested on indigo) and OpenCV 2.4.

Camera setup
--------------

Two PS3Eye webcams mounted 0.5m apart. Original lenses removed, F/2.8 lenses with 
IR band-pass filters (the floppy plastic from floppy disk).
These cameras are a great starting point for tracking application, supporting 
60FPS @ 640x480 and up to 187FPS @ 320x240.
Tracking markers can be either reflective IR spheres (if you have IR illumination), or active markers (an IR-emitting diode).

![Camera setup](https://github.com/festlv/ir-marker-tracking/raw/master/doc/camera-setup.jpg)


Usage
--------

Install in local ROS workspace, build with `catkin_make`.


```
roslaunch marker_tracker all_tracker.launch
...
started roslaunch server http://workstation:42336/
...

[ INFO] [1448314229.278482451]: L: (413.75, 272.66), R: (110.68, 259.75)
[ INFO] [1448314229.278588326]: PT: ( 0.16,  0.04,  0.85)
[ INFO] [1448314230.295143776]: L: (502.56, 208.02), R: (192.68, 202.40)
[ INFO] [1448314230.295243326]: PT: ( 0.31, -0.06,  0.84)

```
![Screenshot of output](https://github.com/festlv/ir-marker-tracking/raw/master/doc/stereo-screenshot.jpg)
Pose topic containing position is also published at /pose and can be visualized
with rviz.


Problems
--------------

At this point, the code isn't even research-quality code (that has been proven
to work at least once).

Observed error was ~0.1m for first 1m but at 3m distance the distance reading was 2.3m 
(0.70m error in depth).

The cause for this might be:

* Calibration was performed with too small checkerboard pattern (A4 sized);
* Extrinsic parameter matrices were constructed by hand (assumed no rotation
  and measured translation in just one axis);
* Resolution is just too small (doubt it);


TODO
---------

* Stereo calibration to find out correct projection matrices for both cameras;
* Support for multiple marker tracking/identification;
* Support for attitude estimation;
* Support for more cameras;

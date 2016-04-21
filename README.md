# BioTracker Camera Calibration
A "tracker" for automatic camera calibration within the [Biotracker](https://github.com/BioroboticsLab/biotracker_core).
It utilizes the camera calibration functions of OpenCV ([OpenvCV example](http://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html)).

![Screenshot](https://cloud.githubusercontent.com/assets/613557/14709343/6c430a5e-07d0-11e6-8c75-5082c32c1de5.png)

## Usage
#. Download and print a chessboard, e.g. the [chessboard](http://docs.opencv.org/2.4/_downloads/pattern.png) pattern of the OpenCV example
#. Load video file, a series of images or a camera device with this chessboard
#. Select the "Chessboard" view to determine whether the chessboad corners are correctly detected
#. Select at least 10 frames (this number is arbitraty and hardcoded :P) by pausing at the frame and pressing the "Add Frame" button
#. Push the "Calibrate" button and selec the "Rectified Image" view to control whether calibration is good enough
#. Export camera matrix by pushing "Save Calibration"

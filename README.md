# multiple objects tracking using Kalman Filter
Xiaoguang Dong mm/year 04/2016 - 04/2020

## Dependencies
* OpenCV <https://opencv.org/releases/>
	* https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html
* image_transport
* cv_bridge
* usb_cam 
* Eigen3

## Tested on
* ros indigo
* ubuntu 14.04


## Demo 1
* Tracking two objects using blob-based tracking (manual feature)
* [image](../data/tracking.png)

* codes
'''
roslaunch mag_tracking test_tracker_video.launch
'''

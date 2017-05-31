# Yet another Visual Odometer (YaVO)

A SLAM algorithm written in C++ for ego-motion estimation and environment reconstruction with a RGB-D camera.

Video demonstration:

[![YaVO video demo](https://media.giphy.com/media/koiSDVWKM4MBG/giphy.gif)](https://youtu.be/MSN_w3eFP0I)

## Installation

First, make sure you have the following packages installed:

* OpenCV 3
* PCL >= 1.8
* [G2O](https://github.com/RainerKuemmerle/g2o)
* Eigen 3

Then clone this repo into a local folder. Then run the good 'ol stuff:

    mkdir build
	cd build
	cmake ..
	make

Now you can find the executable in bin/ folder of our project root.

## Usage

We recommend evaluating our algorithm with [TUM dataset](http://vision.in.tum.de/data/datasets/rgbd-dataset). Our program reads the associated image sequence generated by [associate.py](http://vision.in.tum.de/data/datasets/rgbd-dataset/tools).

To use our algorithm, simply do:

    RGBDSlamApp path_to_associate_txt start_seq end_seq parameters.txt

Explanation:

`path_to_associate_txt`: associated image sequence file generated by `associate.py`

`start_seq`/`end_seq`: Image sequence range used for reconstruction

`parameters.txt`: program configurations(see below)

### Parameter File Format

We have provided you a template parameter file located in `misc/parameters.txt`. The content of this file is very straight-forward. However, there might be a few more things you need to notice before beginning.

* If you are using the images extracted from your own Kinect / Xtion / etc, you have to fill in the calibrated intrinsics beforehand.
* Standard feature extractors (SIFT SURF ORB FAST) and descriptors (SIFT SURF ORB BRISK) etc are supported

## Note

Currently I have not implemented the interface with ROS, therefore online SLAM is not supported yet.

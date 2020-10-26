# RGBD data Parser: RGB + depth to png respectively on ROS

Parse RGB-D data from Azure Kinect to png files.

Basically, this repository is to run *[StaticFusion](https://github.com/raluca-scona/staticfusion)-ImageSequenceAssoc* since they need offline png files, e.g. RGB png and depth png respectively.

Original author: Hyungtae Lim (shapelim@kaist.ac.kr)

======

## Objective

An RGB image and a depth whose types are `sensors_msgs::CompressedImage` is saved as a png file. 

## How To Use

1. Revise the rosparams in `launch/parse_rgbd.launch`.

2. Play your own rosbag file that contains corresponding RGB and depth data.

3. ```roslaunch rgbdsaver parse_rgbd.launch```

4. Thus, the organisational structure of the dataset is as follows:

``` 
/$savedir$/rgb/   - folder containing all color images
/$savedir$/depth/ - folder containing all depth images
/$savedir$/rgbd_assoc.txt - folder sequences for StaticFusion (may not necessary). 
```

## Descriptions

***rgb***

* The decoded image is originally BGR, yet png for StaticFusion is RGB-order. 
* So, if you need a BGR-ordered png file, then annotate 52nd line in `node/src/RGBDSaver.cpp`

***depth***

* It is saved as 16 bit monochrome in PNG. That is, a depth image is saved to png file whose type is **CV_16UC1**, or `unsigned short` on millimeter scale. 
* In line 65 in `node/src/RGBDSaver.cpp`, a depth is originally **CV_32FC1** which is on meter scale.
* However, **CV_32FC1** is automatically casted to **8UC1** when saving the data using `cv::imwrite()` :(. That's why we save the depth to **CV_16UC1**.
* `nan` data is substituted with 0 according to `std::isnan()`.


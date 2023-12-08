<div align="center">
  <h1>kitti2rosbag2</h1>
  <a href="https://github.com/bharadwajsirigadi/kitti2rosbag2/tree/main"><img src="https://img.shields.io/badge/ROS-humble-blue" /></a>
</div>

`kitti2rosbag2` is designed to convert the KITTI Odometry dataset to ROS2 bag format, emphasizing manual control over message publishing and bag recording. It allows more control to users who want more control over the process of converting KITTI data to ROS2 bags.

## KITTI car setup
<div align="center">
  <img width="800" alt="Screenshot 2023-12-08 at 12 47 56 PM" src="https://github.com/bharadwajsirigadi/kitti2rosbag2/assets/105838762/42cd202f-2a14-418a-b576-bf5c55ea9d26">
</div>

## Usage
#### 1. Clone Repository
```
cd ~/ros2_ws/src
```
```
git clone https://github.com/bharadwajsirigadi/kitti2rosbag2.git
```
#### 2. Parameters Input
Open ```<pkg_dir>/config/params.yaml```
Enter Following directories
```
kitti_pub:
  ros__parameters:
    sequence: <sequence_no> 
    data_dir: <data_dir> 
    odom_dir: '<odometry_data_dir>
    bag_dir : <bag_dir>/<bag_name>
    odom : [True/False] 
```
Example:
```
kitti_rec:
  ros__parameters:
    sequence: 2
    data_dir: '/media/psf/SSD/kitti_dataset/dataset'
    odom_dir: '/media/psf/SSD/kitti_dataset/dataset_2'
    bag_dir : '/home/parallels/Desktop/my_bag'
    odom : True
```
#### 3. Building Package
```
cd ~/ros2_ws
```
```
colcon build kitti_odometry_bag_generator --symlink-install
```
#### 4. Converting to bag
```
ros2 launch kitti_odometry_bag_generator kitti2rosbag2.launch
```
[KITTI Odometry Dataset Folder Structure](https://github.com/bharadwajsirigadi/kitti2rosbag2/wiki)

## ROS Topics Info
* `car/base/odom`--> Odometry of Car
* `camera2/left/image_raw`--> Color Images from Left Camera.
* `camera3/right/image_raw`--> Color Images from Right Camera.
* `camera2/left/camera_info`--> Left Camera Information.
* `camera3/right/camera_info`--> Right Camera Information.

## Requirements
* Python 3.x
* ROS2 installed(tested on ROS2 humble)

## Contributors
* [Sai Bharadwaj Sirigadi](https://github.com/bharadwajsirigadi/kitti2rosbag2/graphs/contributors)

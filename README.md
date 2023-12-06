<div align="center">
  <h1>kitti2rosbag2</h1>
  <a href="https://github.com/bharadwajsirigadi/kitti2rosbag2/tree/main"><img src="https://img.shields.io/badge/ROS-humble-blue" /></a>
</div>

#### KITTI Odometry Dataset Folder Structure
```
├── Dataset Directory
│   ├── sequences
│   │   ├── 00
|   |   |   ├──calib.txt
|   |   |   ├──image_2
|   |   |   |   ├── 000000.png
|   |   |   |   ├── 000001.png
|   |   |   |   |       |
|   |   |   ├──image_3
|   |   |   |   ├── 000000.png
|   |   |   |   ├── 000001.png
|   |   |   |   |       |
|   |   |   ├──times.txt
|   |   |   |
```
### Parameters Input
Open ```<pkg_dir>/config/params.yaml```
Enter Following directories
```
kitti_pub:
  ros__parameters:
    sequence: <sequence_no> 
    data_dir: <data_dir> 
    odom_dir: '<odometry_data_dir>
    odom : [True/False]

kitti_rec:
  ros__parameters:
    bag_dir : <bag_dir>
```
Example:
```
kitti_pub:
  ros__parameters:
    sequence: 2
    data_dir: '/media/psf/SSD/kitti_dataset/dataset'
    odom_dir: '/media/psf/SSD/kitti_dataset/dataset_2'
    odom : True

kitti_rec:
  ros__parameters:
    bag_dir : '/home/parallels/Desktop/my_bag'
```
### Recording bag
```
cd ~/ros2_ws
```
```
colcon build kitti_odometry_bag_generator --symlink-install
```
```
roslaunch kitti_odometry_bag_generator kitti2rosbag2.launch
```

### Authors
* [Sai Bharadwaj Sirigadi](https://github.com/bharadwajsirigadi/kitti2rosbag2/graphs/contributors)

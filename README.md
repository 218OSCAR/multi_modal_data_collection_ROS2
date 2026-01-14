# multi_modal_data_collection_ROS2

This document describes the setup process for synchronize data acquisition from multiple sensors with **ROS 2**, and it's easy to extend to meet your own needs.


---

## Environment

- OS: Ubuntu 22.04
- ROS 2 (Humble)

---

## Installation

### Vive tracker
Follow the instructions from **vive_tracker_publisher**:
install the vive_tracker_publisher under the ros2_ws/src:

👉 https://github.com/218OSCAR/vive_tracker_publisher.git

### Realsense Camera
Follow the instructions from **realsense-ros**:
install the realsense-ros under the ros2_ws/src:

👉 https://github.com/realsenseai/realsense-ros

### Gelsight Mini tactile sensor
Follow the instructions from **gelsight_ros2_publisher**:
install the gelsight_ros2_publisher under the ros2_ws/src:

👉 https://github.com/218OSCAR/gelsight_ros2_publisher.git

### Manus Gloves
Follow the instructions from Manus official website:
install everything you need for the manus gloves:

👉 https://docs.manus-meta.com/3.1.0/Resources/
👉 https://docs.manus-meta.com/3.0.0/Plugins/SDK/Linux/
👉 https://docs.manus-meta.com/3.0.0/Plugins/SDK/

---
## Install ffmpeg
You can directly install ffmpeg using:
```
sudo apt install ffmpeg
```
Or you can choose to do it by creating a new conda env:
```
conda create -n gelsight python=3.10 ffmpeg opencv
conda activate gelsight
```


## Clone this repo, run the node and publish ros2 topic 
You have to first clone this repo to your ros2_ws:

```
cd ~/ros2_ws/src/
git clone https://github.com/218OSCAR/gelsight_ros2_publisher.git
```
After that, change the 'config_path'  in ros2_ws/src/gelsight_ros2_publisher/launch/gelsight_publisher.launch.py to your own.
Then connect the gelsight_mini tactile sensor to your PC and run the following code:
```
ros2 launch gelsight_ros2_publisher gelsight_publisher.launch.py
```

## Visulization
You can visulize the tactile image lively by using rqt_image_view:
```
ros2 run rqt_image_view rqt_image_view
```

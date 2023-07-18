# rosbag_to_json_converter
GUI tool to convert topics in rosbag file to json file.  

## Supported versions
Currently, this tool supports Python2 and behavior was confirmed on ROS Melodic.

## How to use
### 1. Installation
Clone this repository in your ROS workspace and execute the following commands.  If you have not initialized rosdep yet, please run rosdep init and update commends before installation.  
```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/ShisatoYano/rosbag_to_json_converter.git
$ cd ~/catkin_ws
$ rosdep install -r --ignore-src --from-paths src
$ catkin_make
```